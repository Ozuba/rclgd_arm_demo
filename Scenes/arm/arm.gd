extends Node3D

var _ros_node : RosNode
var _sub : RosSubscriber

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	_ros_node = RosNode.new()
	_ros_node.init("Arm")
	_sub = _ros_node.create_subscriber("/joint_states", "sensor_msgs/JointState", _on_joints_received)
	
	move_joint_to_angle("ShoulderPan", 0)
	move_joint_to_angle("ShoulderLift", 0)
	move_joint_to_angle("Elbow", 0)
	move_joint_to_angle("ForeArmLink", 0)
	move_joint_to_angle("WristTilt", 0)
	move_joint_to_angle("WristSwing", 0)
	move_joint_to_angle("WristRoll", 0)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
func _physics_process(_delta):
	pass

func _on_joints_received(msg :RosMsg):
	var joint_names = msg.name
	var positions = msg.position
	for i in range(joint_names.size()):
		var j_name = joint_names[i]
		var target_rad = positions[i]
		# 3. Convert ROS radians to Godot degrees and apply
		move_joint_to_angle(j_name, target_rad)
	
# Universal function to move any joint to a target angle
func move_joint_to_angle(joint_name: String, target_rad: float, stiffness: float = 100.0):
	var joint = get_node_or_null(joint_name) as HingeJoint3D
	
	if joint:
		joint.set_param(HingeJoint3D.PARAM_LIMIT_LOWER, target_rad)
		joint.set_param(HingeJoint3D.PARAM_LIMIT_UPPER, target_rad)
		joint.set_flag(HingeJoint3D.FLAG_USE_LIMIT, true)
		
		joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, 0) 
		joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, stiffness)
		
		# KEY FIX: Wake up the physics bodies. 
		# If they aren't moving, they might have gone to "sleep" to save CPU.
		var body_b = joint.get_node(joint.node_b) as RigidBody3D
		if body_b:
			body_b.sleeping = false 
	else:
		# Avoid spamming errors if ROS sends extra joint names (like 'world')
		pass
