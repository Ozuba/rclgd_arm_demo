import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class ArmDemoPublisher(Node):
    def __init__(self):
        super().__init__('arm_demo_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()
        
        # Ensure these names match your Godot node names EXACTLY
        self.joint_names = [
            'ShoulderPan', 'ShoulderLift', 'Elbow', 
            'ForeArmLink', 'WristTilt', 'WristSwing', 'WristRoll'
        ]

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Calculate a simple sine wave for movement
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Base rotation (Pan)
        pan = math.sin(elapsed * 0.5) * 1.5
        # Arm lift (Lift/Elbow)
        lift = math.sin(elapsed * 0.8) * 0.5 - 0.5
        elbow = math.sin(elapsed * 1.2) * 1.0
        
        # Assign positions (order must match self.joint_names)
        msg.position = [pan, lift, elbow, 0.0, lift, 0.0,0.0]
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmDemoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()