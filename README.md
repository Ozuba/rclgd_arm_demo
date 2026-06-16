# RCLGD Arm Demo

A tech demonstrator featuring a 6-DoF UR5e arm connected to the ROS 2 ecosystem
using [rclgd](https://github.com/Ozuba/rclgd).

![Arm](Assets/arm.png)

## What it does

A full closed control loop, with the control logic implemented by hand in
[`move.py`](move.py) (no `ros2_control` / `controller_manager` / hardware
interface required):

```
 /arm_controller/joint_trajectory ──▶ move.py ───/joint_commands──▶  Godot (plant)
   (MoveIt / RViz / CLI goals)        controller  position setpoints   UR5e arm
                                          ▲                                │
                                          └──────── /joint_states ─────────┘
                                                    measured state
```

- **Godot is the plant** ([`Scenes/arm/arm.gd`](Scenes/arm/arm.gd)): it
  subscribes to `/joint_commands` and actuates the arm by driving the six Jolt
  `HingeJoint3D` motors with an inner velocity servo, so the arm has real
  gravity and inertia (set `use_physics = false` on the Arm node for an exact
  kinematic fallback). It reads the joint angles back from the physics and
  publishes them as `/joint_states` (the "sensors"). It also **broadcasts the
  full TF tree** itself using the modern rclgd API
  (`RosNode.create_tf_broadcaster()` → `RosTfBroadcaster.send_transform()`),
  reconstructed from the live link poses. The old in-scene `RosTfBroadcaster3D`
  nodes are gone; transforms are now managed by hand.
- **`move.py` is the controller**: every cycle it reads `/joint_states`,
  interpolates the active trajectory, computes a velocity-limited proportional
  command closed over the measured state, and writes `/joint_commands`. It
  exposes a `trajectory_msgs/JointTrajectory` interface on
  `/arm_controller/joint_trajectory` — the same topic interface
  `joint_trajectory_controller` offers — so MoveIt 2 / RViz / the CLI can drive
  it unchanged.

Frame and joint names follow the **standard UR5e URDF**, so a stock
`ur_moveit_config` or an RViz *RobotModel* display lines up out of the box:

| ROS joint            | child link frame |
|----------------------|------------------|
| `shoulder_pan_joint` | `shoulder_link`  |
| `shoulder_lift_joint`| `upper_arm_link` |
| `elbow_joint`        | `forearm_link`   |
| `wrist_1_joint`      | `wrist_1_link`   |
| `wrist_2_joint`      | `wrist_2_link`   |
| `wrist_3_joint`      | `wrist_3_link`   |

The TF tree is rooted at `world → base_link → … → wrist_3_link`, with the wrist
camera (`camera_link`/`camera_optical`) and lidar (`lidar`) published in `world`.

> Because Godot publishes `/tf` itself, **do not** also run
> `robot_state_publisher` for this arm — that would fight over the same frames.

## Running

1. Open the project in Godot 4.6 and press Play (the `ROS` autoload starts the
   rclgd context automatically). The arm holds its rest pose and starts
   publishing `/joint_states`; it waits for `/joint_commands` before moving.

2. Start the controller. Standalone it self-drives a demo sweep:

   ```bash
   python3 move.py
   ```

3. Visualize in RViz 2 (set **Fixed Frame** to `world`, add a *TF* display):

   ```bash
   ros2 run rviz2 rviz2
   ```

### Sending your own motion

Publish a trajectory on the controller's topic interface (this also turns the
auto-demo off):

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
  points: [{positions: [1.0, -1.2, 1.0, -0.5, 0.5, 0.0], time_from_start: {sec: 3}}]
}'
```

Run it without the demo and command it purely externally:

```bash
python3 move.py --ros-args -p auto_demo:=false
```

Tunables (ROS params): `control_rate` (Hz), `kp`, `max_vel` (rad/s),
`demo_segment_time` (s), `auto_demo`.

### MoveIt 2

MoveIt's trajectory execution publishes to a `joint_trajectory_controller`'s
topic/action interface. Point its controller's topic at
`/arm_controller/joint_trajectory` and `move.py` will execute the planned path;
Godot provides the state feedback and TF. Keep MoveIt's `robot_state_publisher`
disabled (or set `publish_tf = false` on the Arm node) so there is a single
`/tf` source.

## Calibration & tuning

- If a joint turns the wrong way relative to the URDF convention, flip its
  `sign` in the `CHAIN` table at the top of
  [`Scenes/arm/arm.gd`](Scenes/arm/arm.gd) (the same sign is applied to the
  command and the feedback, so the servo stays stable).
- **Colliders:** each link has a `ConvexPolygonShape3D` baked into
  [`Scenes/arm/arm.tscn`](Scenes/arm/arm.tscn) from the dedicated UR5e collision
  meshes in [`Assets/Models/ur5e/collision/`](Assets/Models/ur5e/collision)
  (the STL hulls, transformed into each link's local frame). Godot derives
  inertia and centre of mass from these shapes; only per-link `mass` is set in
  the scene. Links collide with the ground but not each other
  (`collision_layer = 2`, `collision_mask = 1`). To re-bake after changing the
  meshes, re-run the hull generation (see the commit that added the
  `Convex_*` sub-resources).
- Physics-mode knobs on the Arm node: `joint_kp` / `joint_max_vel` (inner
  servo) and `motor_max_impulse` (raise it if the arm sags under gravity). If
  the dynamics need tuning you can't get to right away, set `use_physics =
  false` for the exact kinematic mode.
