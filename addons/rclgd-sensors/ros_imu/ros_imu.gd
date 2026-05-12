extends Node3D
class_name RosImu

# --- Configuration ---
@export_group("ROS 2 Settings")
@export var imu_rate: float = 100.0 
@export var frame_id: String = "~/imu_link"
@export var parent_frame_id: String = "~/base_link"

@export_group("Physical Constants")
@export var gravity_constant: float = 9.81 

@export_group("Noise Model")
@export var accel_noise_std: float = 0.05
@export var gyro_noise_std: float = 0.005
@export var bias_drift_std: float = 0.0001
@export var lpf_tau: float = 0.15 # Low-pass filter for physics jitter

# --- Internal State ---
var is_initialized: bool = false
var _last_velocity: Vector3 = Vector3.ZERO
var _lpf_accel: Vector3 = Vector3.ZERO

var _accel_bias: Vector3 = Vector3.ZERO
var _gyro_bias: Vector3 = Vector3.ZERO

var noisy_accel: Vector3
var noisy_gyro: Vector3
var current_quat: Quaternion

# --- ROS Components ---
var _node: RosNode
var _imu_pub: RosPublisher
var _tf_broadcaster: RosTfBroadcaster
var _timer: RosTimer
var _msg: RosSensorMsgsImu

func _ready() -> void:
	_node = RosNode.new()
	_node.init(name.to_snake_case())
	_imu_pub = _node.create_publisher("~/data", "sensor_msgs/msg/Imu")
	_timer = _node.create_timer(1.0 / imu_rate, _publish_imu)
	
	_tf_broadcaster = _node.create_tf_broadcaster()
	_tf_broadcaster.send_transform(transform, frame_id, parent_frame_id, true)
	
	_msg = RosSensorMsgsImu.new()
	_fill_static_covariances()
	
	is_initialized = true

func _physics_process(delta: float) -> void:
	if not is_initialized: return
	var parent_body = get_parent() as RigidBody3D
	if not parent_body: return

	# 1. Calculate Proper Acceleration (Global Space)
	var current_velocity = parent_body.linear_velocity
	var raw_accel = (current_velocity - _last_velocity) / delta
	_last_velocity = current_velocity
	var proper_accel_g = raw_accel + Vector3(0, gravity_constant, 0)
	
	# 2. Transform to local sensor-space
	var inv_basis = global_transform.basis.inverse()
	var local_accel = inv_basis * proper_accel_g
	var local_gyro = inv_basis * parent_body.angular_velocity
	
	# 3. Apply Low-Pass Filter (LPF)
	# Physics engines create "spikes" that break Factor Graphs; LPF smooths them.
	_lpf_accel = _lpf_accel.lerp(local_accel, 1.0 - lpf_tau)

	# 4. Update Random Walk Biases
	# CRITICAL: Drift must scale with sqrt(delta) for mathematical consistency.
	var drift_scale = sqrt(delta)
	_accel_bias += _get_noise_vec(bias_drift_std * drift_scale)
	_gyro_bias += _get_noise_vec(bias_drift_std * drift_scale)

	# 5. Final Noisy Readings
	noisy_accel = _lpf_accel + _accel_bias + _get_noise_vec(accel_noise_std)
	noisy_gyro = local_gyro + _gyro_bias + _get_noise_vec(gyro_noise_std)
	current_quat = Quaternion(global_transform.basis)

func _publish_imu() -> void:
	_msg.header.stamp = _node.now()
	_msg.header.frame_id = _node.resolve_frame(frame_id)
	
	# --- Coordinate Mapping: (Godot x,y,z) -> (ROS z,x,y) ---
	# As per your request for the specific mapping:
	_msg.linear_acceleration.x = noisy_accel.z
	_msg.linear_acceleration.y = noisy_accel.x
	_msg.linear_acceleration.z = noisy_accel.y
	
	_msg.angular_velocity.x = noisy_gyro.z
	_msg.angular_velocity.y = noisy_gyro.x
	_msg.angular_velocity.z = noisy_gyro.y
	
	# --- Orientation Mapping ---
	var ros_quat = Quaternion(
		-current_quat.z,
		-current_quat.x,
		current_quat.y,
		current_quat.w
	)
	#print(ros_basis.get_euler())
	
	_msg.orientation.x = ros_quat.x
	_msg.orientation.y = ros_quat.y
	_msg.orientation.z = ros_quat.z
	_msg.orientation.w = ros_quat.w
	
	_imu_pub.publish(_msg)

func _fill_static_covariances() -> void:
	# GTSAM requires non-zero diagonals in the covariance matrix
	var a_var = pow(accel_noise_std, 2)
	var g_var = pow(gyro_noise_std, 2)
	
	for i in range(3):
		_msg.linear_acceleration_covariance[i * 4] = a_var
		_msg.angular_velocity_covariance[i * 4] = g_var

func _get_noise_vec(std: float) -> Vector3:
	return Vector3(randfn(0, std), randfn(0, std), randfn(0, std))
