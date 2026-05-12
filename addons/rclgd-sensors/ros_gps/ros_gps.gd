extends Node3D
class_name RosGps

# --- Configuration ---
@export_group("ROS 2 Settings")
@export var gps_rate: float = 10.0 
@export var frame_id: String = "~/gps_link"
@export var parent_frame_id: String = "~/base_link" # Path to the car's center

@export_group("GPS Origin (WGS84)")
@export var origin_lat: float = 42.8125 
@export var origin_lon: float = -1.6458 
@export var origin_alt: float = 450.0   

@export_group("Noise Model (Standard Deviation)")
@export var position_std: float = 0.5    # Horizontal jitter (meters)
@export var altitude_std: float = 1.2    # Vertical jitter (meters)

# --- WGS84 Constants ---
const WGS84_A = 6378137.0
const WGS84_F = 1.0 / 298.257223563
const WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

# --- Internal State ---
var is_initialized: bool = false
var noisy_pos: Vector3 = Vector3.ZERO

# --- ROS Components ---
var _node: RosNode
var _gps_pub: RosPublisher
var _tf_broadcaster: RosTfBroadcaster # New: For static transform
var _timer: RosTimer
var _msg: RosSensorMsgsNavSatFix

func _ready() -> void:
	_node = RosNode.new()
	_node.init(name.to_snake_case())
	_gps_pub = _node.create_publisher("~/fix", "sensor_msgs/msg/NavSatFix")
	_timer = _node.create_timer(1.0 / gps_rate, _publish_gps)
	
	# --- Static Transform Publisher ---
	# This tells ROS exactly where the antenna is relative to the car body
	_tf_broadcaster = _node.create_tf_broadcaster()
	_tf_broadcaster.send_transform(transform, frame_id, parent_frame_id, true) 
	
	_msg = RosSensorMsgsNavSatFix.new()
	_fill_static_covariances()
	
	is_initialized = true

func _physics_process(_delta: float) -> void:
	if not is_initialized: return
	var gt = global_position
	
	noisy_pos.x = gt.x + randfn(0, position_std)
	noisy_pos.y = gt.y + randfn(0, altitude_std)
	noisy_pos.z = gt.z + randfn(0, position_std)

func _publish_gps() -> void:
	_msg.header.stamp = _node.now()
	_msg.header.frame_id = _node.resolve_frame(frame_id)
	
	# Mapping: Godot Z -> ROS X, Godot X -> ROS Y, Godot Y -> ROS Z
	var d_north = noisy_pos.z
	var d_east = noisy_pos.x
	var d_up = noisy_pos.y
	
	var phi = deg_to_rad(origin_lat)
	var den = sqrt(1.0 - WGS84_E2 * sin(phi) * sin(phi))
	var M = WGS84_A * (1.0 - WGS84_E2) / (den * den * den)
	var N = WGS84_A / den
	
	_msg.latitude = origin_lat + rad_to_deg(d_north / M)
	_msg.longitude = origin_lon + rad_to_deg(d_east / (N * cos(phi)))
	_msg.altitude = origin_alt + d_up
	
	_msg.status.status = 0 
	_msg.status.service = 1
	
	_gps_pub.publish(_msg)

func _fill_static_covariances() -> void:
	var p_var = pow(position_std, 2)
	var a_var = pow(altitude_std, 2)
	_msg.position_covariance[0] = p_var
	_msg.position_covariance[4] = p_var
	_msg.position_covariance[8] = a_var
	_msg.position_covariance_type = 2
