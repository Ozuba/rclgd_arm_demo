#!/usr/bin/env python3
"""Self-contained joint controller for the rclgd UR5e arm demo.

This implements the control loop by hand (no ros2_control / controller_manager /
hardware interface needed). It plays the role the joint_trajectory_controller +
joint_state_broadcaster + hardware interface would normally play:

    /arm_controller/joint_trajectory   (in)  trajectory goals  -- MoveIt/RViz/CLI
    /joint_states                       (in)  measured state    -- from Godot (plant)
    /joint_commands                     (out) position setpoints-> to Godot (plant)

Every control cycle it: READS the measured state (sensor), interpolates the
active trajectory to a reference, computes a velocity-limited proportional
command closed over the measured state, and WRITES the command to the plant.

Godot subscribes to /joint_commands, actuates the arm kinematically, and
publishes /joint_states back -- closing the loop.

Run standalone (`python3 move.py`) and it self-drives a demo sweep. Publish a
trajectory_msgs/JointTrajectory on /arm_controller/joint_trajectory (the same
topic interface joint_trajectory_controller exposes) and it follows that
instead -- so MoveIt 2 or `ros2 topic pub` can drive it without changes.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# Standard UR5e joint names (must match arm.gd / the URDF).
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
N = len(JOINTS)

# Poses cycled through in the built-in demo (one 6-vector per waypoint, radians).
DEMO_POSES = [
    [0.0, -1.0, 0.5, 0.0, 0.0, 0.0],
    [1.5, -0.8, 1.2, -0.5, 0.8, 1.0],
    [-1.5, -1.4, 0.8, 0.6, -0.8, -1.0],
    [0.0, -0.6, 1.6, -1.0, 0.0, 1.5],
]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")

        # --- Tunables ---
        self.control_rate = self.declare_parameter("control_rate", 100.0).value
        self.kp = self.declare_parameter("kp", 8.0).value           # 1/s
        self.max_vel = self.declare_parameter("max_vel", 2.0).value  # rad/s per joint
        self.use_demo = self.declare_parameter("auto_demo", True).value
        self.demo_time = self.declare_parameter("demo_segment_time", 3.0).value
        self.dt = 1.0 / self.control_rate

        # --- Loop state ---
        self._meas = {}          # joint name -> measured position
        self.measured = None     # ordered list once all joints are known
        self.command = None      # ordered list of current commanded positions
        self.traj = None         # active trajectory: {start, times[], points[]}
        self.demo_idx = 0

        # --- ROS interfaces ---
        self.cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.create_subscription(
            JointTrajectory, "/arm_controller/joint_trajectory", self.on_trajectory, 10
        )
        self.create_timer(self.dt, self.control_tick)

        self.get_logger().info(
            "arm_controller up: reading /joint_states, writing /joint_commands "
            "(auto_demo=%s)" % self.use_demo
        )

    # -- time helper ---------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- sensor read ---------------------------------------------------------
    def on_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._meas[name] = pos
        if all(j in self._meas for j in JOINTS):
            self.measured = [self._meas[j] for j in JOINTS]

    # -- trajectory interface (MoveIt / RViz / CLI) --------------------------
    def on_trajectory(self, msg: JointTrajectory):
        if self.command is None or not msg.points:
            return
        # Map the goal's joint order onto our fixed JOINTS order.
        order = [msg.joint_names.index(j) if j in msg.joint_names else None for j in JOINTS]
        times, points = [], []
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            pos = []
            for k in range(N):
                src = order[k]
                if src is not None and src < len(pt.positions):
                    pos.append(pt.positions[src])
                else:
                    pos.append(self.command[k])  # unspecified joint: hold
            times.append(t)
            points.append(pos)
        self.use_demo = False  # external command takes over
        self.set_trajectory(times, points)
        self.get_logger().info("Following trajectory with %d point(s)." % len(points))

    # -- trajectory bookkeeping ---------------------------------------------
    def set_trajectory(self, times, points):
        # Prepend the current command at t=0 so we interpolate smoothly from it.
        self.traj = {
            "start": self.now(),
            "times": [0.0] + list(times),
            "points": [list(self.command)] + [list(p) for p in points],
        }

    def sample_trajectory(self):
        """Return (reference_positions, finished)."""
        t = self.now() - self.traj["start"]
        times, pts = self.traj["times"], self.traj["points"]
        if t >= times[-1]:
            return pts[-1], True
        for i in range(1, len(times)):
            if t <= times[i]:
                t0, t1 = times[i - 1], times[i]
                a = (t - t0) / max(t1 - t0, 1e-6)
                return [pts[i - 1][j] + (pts[i][j] - pts[i - 1][j]) * a for j in range(N)], False
        return pts[-1], True

    def load_demo(self):
        target = DEMO_POSES[self.demo_idx % len(DEMO_POSES)]
        self.demo_idx += 1
        self.set_trajectory([self.demo_time], [target])

    # -- the control loop: read -> compute -> write --------------------------
    def control_tick(self):
        if self.measured is None:
            return  # wait until the plant reports its state
        if self.command is None:
            self.command = list(self.measured)  # start from where the plant is

        # Reference setpoint for this cycle.
        if self.traj is None and self.use_demo:
            self.load_demo()
        if self.traj is not None:
            ref, finished = self.sample_trajectory()
            if finished:
                self.traj = None
        else:
            ref = self.command  # hold last commanded pose

        # Forward the reference setpoint directly. Godot's PID controller handles the rest!
        cmd = []
        for j in range(N):
            cmd.append(ref[j])
        self.command = cmd

        self.write_command()

    # -- actuator write ------------------------------------------------------
    def write_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [float(x) for x in self.command]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
