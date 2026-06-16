extends Node3D
class_name RosArm

# =============================================================================
#  RCLGD UR5e Arm Demo  -  ros2_control "plant" + manual TF broadcasting
# -----------------------------------------------------------------------------
#  In the control loop this node is the HARDWARE / PLANT:
#
#    * SUBSCRIBES to /joint_commands (sensor_msgs/JointState, position) -- the
#      setpoints written by the controller in move.py.
#
#    * ACTUATES the arm. Two modes:
#        - PHYSICS (default): the six Jolt HingeJoint3D motors are driven by an
#          inner velocity servo toward the commanded angle, so the arm has real
#          gravity / inertia and the loop closes over genuine physics state.
#        - KINEMATIC (use_physics = false): the links are frozen and posed by
#          forward kinematics (exact, no dynamics) -- a safe fallback.
#
#    * PUBLISHES its measured state on /joint_states (sensor_msgs/JointState),
#      read straight from the joints, closing the loop for the controller and
#      feeding the rest of ROS (RViz / MoveIt).
#
#    * BROADCASTS the whole TF tree itself with the modern rclgd API
#      (RosNode.create_tf_broadcaster() -> RosTfBroadcaster.send_transform()),
#      reconstructed from the live link poses. The deprecated in-scene
#      RosTfBroadcaster3D nodes are gone. Because Godot publishes /tf, you do
#      NOT run robot_state_publisher for the arm.
#
#  Frame / joint names follow the standard UR5e URDF.
# =============================================================================

@export_group("ROS 2 Topics")
## Position setpoints from the controller (sensor_msgs/JointState).
@export var command_topic: String = "/joint_commands"
## Measured joint state published back to the controller / ROS.
@export var feedback_topic: String = "/joint_states"

@export_group("Actuation")
## Drive the Jolt hinge motors (real dynamics). Off = kinematic FK fallback.
@export var use_physics: bool = true
## Inner velocity-servo gain on each joint (1/s).
@export var joint_kp: float = 12.0
@export var joint_ki: float = 0.5
@export var joint_kd: float = 0.1
## Per-joint speed cap for the motor servo (rad/s).
@export var joint_max_vel: float = 3.0
## Hinge motor strength (Godot PARAM_MOTOR_MAX_IMPULSE); raise if the arm sags.
@export var motor_max_impulse: float = 400.0
## Damping applied to the links so contact oscillations settle instead of jitter.
@export var link_angular_damp: float = 2.0
@export var link_linear_damp: float = 0.5

@export_group("TF Settings")
## Broadcast the TF tree from Godot (turn off if a robot_state_publisher runs).
@export var publish_tf: bool = true
## Root frame the arm base is anchored to (RViz "Fixed Frame").
@export var root_frame: String = "world"
## Frame of the robot base; root of the robot's own TF subtree.
@export var base_frame: String = "base_link"
## Dynamic TF broadcast rate (Hz).
@export var tf_rate: float = 50.0

@export_group("Feedback")
## Measured /joint_states publish rate (Hz).
@export var feedback_rate: float = 100.0

# The kinematic chain, base -> tip. Each entry binds the Godot hinge / link
# nodes to the standard UR5e ROS joint and child-link frame names.
#   hinge:  HingeJoint3D at the joint origin; its local Z is the rotation axis.
#   body:   the RigidBody3D (link) driven by that joint.
#   parent: the link the joint hangs off (the hinge's node_a).
#   sign:   flip if a joint turns opposite to the URDF convention. The same sign
#           is applied to both command and feedback, so the servo stays stable.
const CHAIN := [
	{"hinge": "ShoulderPan",  "body": "ShoulderLink", "parent": "BaseLink",     "joint": "shoulder_pan_joint",  "frame": "shoulder_link",  "sign": 1.0},
	{"hinge": "ShoulderLift", "body": "UpperArmLink",  "parent": "ShoulderLink", "joint": "shoulder_lift_joint", "frame": "upper_arm_link",  "sign": 1.0},
	{"hinge": "Elbow",        "body": "ForearmLink",   "parent": "UpperArmLink", "joint": "elbow_joint",         "frame": "forearm_link",   "sign": 1.0},
	{"hinge": "WristTilt",    "body": "Wrist1",        "parent": "ForearmLink",  "joint": "wrist_1_joint",       "frame": "wrist_1_link",   "sign": 1.0},
	{"hinge": "WristSwing",   "body": "Wrist2",        "parent": "Wrist1",       "joint": "wrist_2_joint",       "frame": "wrist_2_link",   "sign": 1.0},
	{"hinge": "WristRoll",    "body": "Wrist3",        "parent": "Wrist2",       "joint": "wrist_3_joint",       "frame": "wrist_3_link",   "sign": 1.0},
]

# --- ROS components ---
var _node: RosNode
var _cmd_sub: RosSubscriber
var _state_pub: RosPublisher
var _tf: RosTfBroadcaster

# --- Cached node references ---
var _hinges: Array[HingeJoint3D] = []
var _bodies: Array[Node3D] = []
var _parents: Array[Node3D] = []

# --- Captured rest geometry (filled on the first physics frame) ---
var _base_rest: Transform3D                  # base_link frame at rest (Godot global)
var _pivot_offset: Array[Transform3D] = []   # pivot frame expressed in its link body
var _rest_rel: Array[Transform3D] = []       # child body in parent body, at rest
var _hinge_axis: Array[Vector3] = []         # hinge axis in parent frame (for angle read)
# Kinematic-mode extras:
var _pivot_rest: Array[Transform3D] = []
var _body_rest: Array[Transform3D] = []
var _rel_rest: Array[Transform3D] = []

# --- Live state ---
var _target_angles := PackedFloat64Array()   # commanded angles [rad], ROS sign, CHAIN order
var _measured_angles := PackedFloat64Array()  # measured angles [rad], ROS sign
var _error_integral := PackedFloat64Array()
var _prev_error := PackedFloat64Array()
var _name_to_index := {}
var _joint_names := PackedStringArray()
var _state_msg: RosSensorMsgsJointState
var _initialized := false
var _tf_accum := 0.0
var _fb_accum := 0.0


func _ready() -> void:
	# Force these values in case the .tscn scene file has old exported values saved
	# Tune down the aggressive parameters now that it has full error access
	joint_kp = 6.0
	joint_ki = 0.1
	joint_kd = 0.2
	motor_max_impulse = 10000.0

	_target_angles.resize(CHAIN.size())
	_measured_angles.resize(CHAIN.size())
	_error_integral.resize(CHAIN.size())
	_prev_error.resize(CHAIN.size())
	_error_integral.fill(0.0)
	_prev_error.fill(0.0)
	for i in CHAIN.size():
		var link: Dictionary = CHAIN[i]
		_hinges.append(get_node(NodePath(link.hinge)) as HingeJoint3D)
		_bodies.append(get_node(NodePath(link.body)) as Node3D)
		_parents.append(get_node(NodePath(link.parent)) as Node3D)
		_name_to_index[link.joint] = i
		_joint_names.append(link.joint)

	_configure_actuators()

	# --- ROS setup ---
	_node = RosNode.new()
	_node.init("arm")
	_tf = _node.create_tf_broadcaster()
	_cmd_sub = _node.create_subscriber(command_topic, "sensor_msgs/msg/JointState", _on_command)
	_state_pub = _node.create_publisher(feedback_topic, "sensor_msgs/msg/JointState")

	_state_msg = RosSensorMsgsJointState.new()
	_state_msg.name = _joint_names


## Set the links up for either physics (motorised hinges) or kinematic posing.
func _configure_actuators() -> void:
	for i in CHAIN.size():
		var body := _bodies[i] as RigidBody3D
		var hinge := _hinges[i]
		if use_physics:
			if body:
				body.freeze = false
				body.can_sleep = false
				body.angular_damp = link_angular_damp
				body.linear_damp = link_linear_damp
			hinge.set_flag(HingeJoint3D.FLAG_USE_LIMIT, false)
			hinge.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
			hinge.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, motor_max_impulse)
		else:
			# Kinematic fallback: freeze the bodies, joints go inert.
			if body:
				body.freeze = true
				body.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
			hinge.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, false)


func _on_command(msg: RosMsg) -> void:
	# Runs on the ROS executor thread: only touch plain data here.
	var names = msg.name
	var positions = msg.position
	var count: int = min(names.size(), positions.size())
	for i in count:
		var idx = _name_to_index.get(names[i], -1)
		if idx != -1:
			_target_angles[idx] = positions[i]


func _physics_process(delta: float) -> void:
	if not _initialized:
		_capture_rest_pose()
		_initialized = true
		if publish_tf:
			_tf.send_transform(_base_rest, base_frame, root_frame, true, _node.now())

	if use_physics:
		_drive_motors(delta)
	else:
		_pose_kinematic()

	if publish_tf:
		_tf_accum += delta
		if _tf_accum >= 1.0 / max(tf_rate, 1.0):
			_tf_accum = 0.0
			_broadcast_tf()

	_fb_accum += delta
	if _fb_accum >= 1.0 / max(feedback_rate, 1.0):
		_fb_accum = 0.0
		_publish_feedback()


## Capture the authored rest configuration once tree transforms have settled.
func _capture_rest_pose() -> void:
	_base_rest = ($BaseLink as Node3D).global_transform

	_pivot_offset.clear(); _rest_rel.clear(); _hinge_axis.clear()
	_pivot_rest.clear(); _body_rest.clear(); _rel_rest.clear()

	var prev_pivot := _base_rest
	for i in CHAIN.size():
		var pivot := _hinges[i].global_transform
		var body := _bodies[i].global_transform
		var parent := _parents[i].global_transform

		# TF reconstruction: pivot frame rigidly attached to its link body.
		_pivot_offset.append(body.affine_inverse() * pivot)
		# Measured angle: rest pose of child in parent, plus the hinge axis there.
		_rest_rel.append(parent.affine_inverse() * body)
		_hinge_axis.append((parent.affine_inverse() * pivot).basis.z.normalized())

		# Kinematic-mode FK data.
		_pivot_rest.append(pivot)
		_body_rest.append(body)
		_rel_rest.append(prev_pivot.affine_inverse() * pivot)
		prev_pivot = pivot


## PHYSICS: read each hinge angle from the bodies, then command the motor with a
## full PID controller toward the target angle.
func _drive_motors(delta: float) -> void:
	for i in CHAIN.size():
		var js: float = float(CHAIN[i].sign)
		var measured_g := _read_joint_angle(i)
		_measured_angles[i] = measured_g * js

		var error: float = _target_angles[i] * js - measured_g
		
		# Anti-windup: limit integral contribution to max velocity
		if joint_ki > 0.0:
			var max_i: float = joint_max_vel / joint_ki
			_error_integral[i] = clampf(_error_integral[i] + error * delta, -max_i, max_i)
		else:
			_error_integral[i] = 0.0
			
		var derivative: float = 0.0
		if delta > 0.0:
			derivative = (error - _prev_error[i]) / delta
		_prev_error[i] = error

		var pid_out: float = (joint_kp * error) + (joint_ki * _error_integral[i]) + (joint_kd * derivative)
		var desired: float = clampf(pid_out, -joint_max_vel, joint_max_vel)

		# Continuously enforce the max impulse so the physics solver doesn't drop it
		_hinges[i].set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, motor_max_impulse)
		# Godot Jolt motor velocity direction is inverted relative to the kinematic basis
		_hinges[i].set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, -desired)


## Signed rotation of child link relative to parent, about the hinge axis.
func _read_joint_angle(i: int) -> float:
	var cur_rel := _parents[i].global_transform.affine_inverse() * _bodies[i].global_transform
	var delta := _rest_rel[i].affine_inverse() * cur_rel
	var q := delta.basis.get_rotation_quaternion()
	return 2.0 * atan2(Vector3(q.x, q.y, q.z).dot(_hinge_axis[i]), q.w)


## KINEMATIC fallback: forward kinematics about each pivot's local Z.
func _pose_kinematic() -> void:
	var g_parent := _base_rest
	for i in CHAIN.size():
		var angle: float = _target_angles[i] * float(CHAIN[i].sign)
		var rel := _rel_rest[i] * Transform3D(Basis(Vector3(0, 0, 1), angle), Vector3.ZERO)
		var g_pivot := g_parent * rel
		var motion := g_pivot * _pivot_rest[i].affine_inverse()
		(_bodies[i] as Node3D).global_transform = motion * _body_rest[i]
		_measured_angles[i] = _target_angles[i]
		g_parent = g_pivot


## Rebuild the URDF-aligned TF tree from the live link poses (works in both
## modes): each pivot frame rides along with its link body.
func _broadcast_tf() -> void:
	var stamp := _node.now()
	var prev_pivot := _base_rest
	var parent_frame := base_frame
	for i in CHAIN.size():
		var pivot_live := _bodies[i].global_transform * _pivot_offset[i]
		var rel := prev_pivot.affine_inverse() * pivot_live
		_tf.send_transform(rel, CHAIN[i].frame, parent_frame, false, stamp)
		prev_pivot = pivot_live
		parent_frame = CHAIN[i].frame


func _publish_feedback() -> void:
	_state_msg.header.stamp = _node.now()
	_state_msg.position = _measured_angles
	_state_pub.publish(_state_msg)
