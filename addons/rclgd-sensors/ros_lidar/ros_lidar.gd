extends Node3D
class_name RosLidar

# --- Constants ---
const LIDAR_EFFECT_CLASS = preload("res://addons/rclgd-sensors/ros_lidar/lidar_compositor.gd")

# --- Configuration ---
@export var resolution: Vector2i = Vector2i(600, 125)
@export var publish_rate: float = 10.0
@export var lidar_topic: String = "~/lidar"
@export var frame_id: String = "~lidar"
@export var parent_frame_id: String = "~base_link"
@export var noise_std_dev: float = 0.005

# --- Internal Nodes ---
var _viewport: SubViewport
var _camera: Camera3D
var _remote_sync: RemoteTransform3D # <--- The missing piece

# --- Resource Retention (Crucial for RIDs) ---
var _compositor: Compositor
var _effect: RenderingEffectDepthCapture

# --- ROS Components ---
var _node: RosNode
var _lidar_pub: RosPublisher
var _tf_broadcaster: RosTfBroadcaster
var _ros_timer: RosTimer

# --- Rendering Components ---
var _rd: RenderingDevice
var _texture_rid: RID
var is_sampling: bool = false
var _cached_msg: RosSensorMsgsPointCloud2
var _current_stamp: RosMsg

func _ready() -> void:
	# 1. Pipeline hierarchy
	_setup_rendering_pipeline()
	
	# 2. ROS 2 Init
	_node = RosNode.new()
	_node.init(name.to_snake_case())
	_lidar_pub = _node.create_publisher(lidar_topic, "sensor_msgs/msg/PointCloud2")
	_tf_broadcaster = _node.create_tf_broadcaster()

	# 3. Setup Hardware Rendering
	_rd = RenderingServer.get_rendering_device()
	_create_storage_texture()
	_setup_compositor()

	# 4. Start Loop
	_prepare_msg_template()
	var interval: float = 1.0 / publish_rate
	_ros_timer = _node.create_timer(interval, _on_timer_timeout)

func _setup_rendering_pipeline() -> void:
	_viewport = SubViewport.new()
	_viewport.name = "LidarViewport"
	_viewport.size = resolution
	_viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	add_child(_viewport)
	
	_camera = Camera3D.new()
	_camera.name = "LidarCamera"
	_viewport.add_child(_camera)

	# Setup RemoteTransform3D to sync the camera to THIS node
	_remote_sync = RemoteTransform3D.new()
	_remote_sync.name = "LidarSyncBridge"
	add_child(_remote_sync)
	_remote_sync.remote_path = _remote_sync.get_path_to(_camera)

func _create_storage_texture() -> void:
	var fmt : RDTextureFormat = RDTextureFormat.new()
	fmt.format = RenderingDevice.DATA_FORMAT_R32G32B32A32_SFLOAT 
	fmt.width = resolution.x
	fmt.height = resolution.y
	fmt.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | \
					 RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT | \
					 RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
	
	_texture_rid = _rd.texture_create(fmt, RDTextureView.new())

func _setup_compositor() -> void:
	# Keep a class-level reference so it doesn't get garbage collected (Freeing RIDs)
	_effect = LIDAR_EFFECT_CLASS.new()
	_effect.target_tex = _texture_rid
	_effect.texture_size = Vector2(resolution)
	_effect.std_dev = noise_std_dev
	
	_compositor = Compositor.new()
	_compositor.compositor_effects = [_effect]
	_camera.compositor = _compositor

func _on_timer_timeout():
	if is_sampling or not _texture_rid.is_valid(): return
		
	is_sampling = true
	_current_stamp = _node.now()
	
	# Update TF before render
	_tf_broadcaster.send_transform(global_transform, frame_id, parent_frame_id, false)

	# Trigger Render
	_viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
	
	# WAIT for the frame to be drawn before requesting data (Fixes "Tex is Null")
	if not RenderingServer.frame_post_draw.is_connected(_on_frame_drawn):
		RenderingServer.frame_post_draw.connect(_on_frame_drawn, CONNECT_ONE_SHOT)

func _on_frame_drawn():
	# Now it is safe to request data
	var err = _rd.texture_get_data_async(_texture_rid, 0, _on_texture_data_ready)
	if err != OK: is_sampling = false

func _on_texture_data_ready(raw_bytes: PackedByteArray):
	is_sampling = false
	if raw_bytes.is_empty(): return 
	
	_cached_msg.header.stamp = _current_stamp
	_cached_msg.width = raw_bytes.size() / 16
	_cached_msg.row_step = raw_bytes.size()
	_cached_msg.data = raw_bytes 
	
	_lidar_pub.publish(_cached_msg)

func _prepare_msg_template():
	_cached_msg = RosSensorMsgsPointCloud2.new()
	_cached_msg.height = 1
	_cached_msg.is_dense = true
	_cached_msg.point_step = 16 
	_cached_msg.header.frame_id = frame_id
	_cached_msg.fields = [
		_create_field("x", 0, 7), _create_field("y", 4, 7),
		_create_field("z", 8, 7), _create_field("intensity", 12, 7)
	]

func _create_field(fname: String, offset: int, datatype: int) -> RosSensorMsgsPointField:
	var f = RosSensorMsgsPointField.new()
	f.name = fname; f.offset = offset; f.datatype = datatype; f.count = 1
	return f
