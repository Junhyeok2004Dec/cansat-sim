import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry

class ParafoilController(Node):
    def __init__(self):
        super().__init__('parafoil_controller')
        # 1. 상태(위치, 속도) 구독
        self.sub_odom = self.create_subscription(Odometry, '/model/cansat/odometry', self.odom_callback, 10)
        # 2. 제어력(Wrench) 발행
        self.pub_wrench = self.create_publisher(Wrench, '/model/cansat/force_torque', 10)
        
        self.timer = self.create_timer(0.02, self.control_loop) # 50Hz

    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        self.current_vel = msg.twist.twist.linear

    def control_loop(self):
        
        # u = [delta_L, delta_R]
        
        wrench_msg = Wrench()
        # yaw moment
        # tau_z = k * (delta_R - delta_L)
        wrench_msg.torque.z = self.calculate_steering_torque()
        
        self.pub_wrench.publish(wrench_msg)

def main():
    rclpy.init()
    node = ParafoilController()
    rclpy.spin(node)
    rclpy.shutdown()
