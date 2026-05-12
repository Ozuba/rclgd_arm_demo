extends Node3D 
class_name RosCamera

# --- Configuration ---
@export var resolution: Vector2i = Vector2i(640, 480)
@export var publish_rate: float = 15.0
@export var frame_id: String = "camera_link"
@export var parent_frame_id: String = "base_link"
@export var optical_frame_id: String = "camera_optical"

# --- Internal Nodes ---
var _viewport: SubViewport
var _camera: Camera3D
var _remote_transform: RemoteTransform3D # The bridge node

# --- ROS Components ---
var _node: RosNode
var _camera_pub: RosPublisher
var _camera_info_pub: RosPublisher
var _tf_broadcaster: RosTfBroadcaster
var _ros_timer: RosTimer

# --- Internal Variables ---
var _cached_image_msg: RosSensorMsgsImage
var _cached_info_msg: RosSensorMsgsCameraInfo
var rd: RenderingDevice
var is_requesting: bool = false
var _resolved_optical_frame: String
var _current_stamp: RosMsg

var optical_tf = Transform3D(Basis(Vector3(0, -1, 0), Vector3(0, 0, 1), Vector3(-1, 0, 0)).orthonormalized(), Vector3.ZERO)

func _ready() -> void:
	_setup_internal_nodes()
	
	_node = RosNode.new()
	_node.init(name.to_snake_case())
	
	_camera_pub = _node.create_publisher("~/image_raw", "sensor_msgs/msg/Image")
	_camera_info_pub = _node.create_publisher("~/camera_info", "sensor_msgs/msg/CameraInfo")
	_tf_broadcaster = _node.create_tf_broadcaster()
	_resolved_optical_frame = _node.resolve_frame(optical_frame_id)

	rd = RenderingServer.get_rendering_device()

	_cache_message_templates(_camera)
	
	# Initial TF publish
	_tf_broadcaster.send_transform(optical_tf, optical_frame_id, frame_id, true)
	
	var interval: float = 1.0 / publish_rate
	_ros_timer = _node.create_timer(interval, _request_capture)

func _setup_internal_nodes() -> void:
	# 1. Create the Viewport (The rendering container)
	_viewport = SubViewport.new()
	_viewport.name = "RosViewport"
	_viewport.size = resolution
	_viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	add_child(_viewport)
	
	# 2. Create the Camera (Inside the viewport)
	_camera = Camera3D.new()
	_camera.name = "RosCamera3D"
	_viewport.add_child(_camera)

	# 3. Create the RemoteTransform3D (The sync bridge)
	# This ensures the internal camera follows this Node3D's movement
	_remote_transform = RemoteTransform3D.new()
	_remote_transform.name = "CameraSync"
	add_child(_remote_transform)
	
	# Link the bridge to the camera
	_remote_transform.remote_path = _remote_transform.get_path_to(_camera)

func _request_capture() -> void:
	if is_requesting: return 
		
	is_requesting = true 
	_current_stamp = _node.now()
	
	# Sync the TF right before we render to ensure the "photo" is taken at the correct spot
	_tf_broadcaster.send_transform(global_transform, frame_id, parent_frame_id,false, _node.now())
	
	_viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
	
	if not RenderingServer.frame_post_draw.is_connected(_on_frame_drawn):
		RenderingServer.frame_post_draw.connect(_on_frame_drawn, CONNECT_ONE_SHOT)

func _on_frame_drawn() -> void:
	var tex = _viewport.get_texture()
	var rid = RenderingServer.texture_get_rd_texture(tex.get_rid())
	
	if rid.is_valid():
		rd.texture_get_data_async(rid, 0, _on_data_received)
	else:
		is_requesting = false

func _on_data_received(data: PackedByteArray) -> void:
	if not data.is_empty():
		_cached_image_msg.header.stamp = _current_stamp
		_cached_image_msg.data = data
		_cached_info_msg.header.stamp = _current_stamp
		_camera_pub.publish(_cached_image_msg)
		_camera_info_pub.publish(_cached_info_msg)
	is_requesting = false

func _cache_message_templates(camera: Camera3D) -> void:
	_cached_image_msg = RosSensorMsgsImage.new()
	_cached_image_msg.height = resolution.y
	_cached_image_msg.width = resolution.x
	_cached_image_msg.encoding = "rgba8"
	_cached_image_msg.step = resolution.x * 4
	_cached_image_msg.header.frame_id = _resolved_optical_frame
	
	_cached_info_msg = RosSensorMsgsCameraInfo.new()
	_cached_info_msg.header.frame_id = _resolved_optical_frame
	_cached_info_msg.width = resolution.x
	_cached_info_msg.height = resolution.y
	_cached_info_msg.distortion_model = "plumb_bob"
	_cached_info_msg.d = PackedFloat64Array([0.0, 0.0, 0.0, 0.0, 0.0])
	
	var fovy_rad = deg_to_rad(camera.fov)
	var fy = resolution.y / (2.0 * tan(fovy_rad / 2.0))
	var fx = fy 
	var cx = resolution.x / 2.0
	var cy = resolution.y / 2.0
	
	_cached_info_msg.k = PackedFloat64Array([fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0])
	_cached_info_msg.r = PackedFloat64Array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
	_cached_info_msg.p = PackedFloat64Array([fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
