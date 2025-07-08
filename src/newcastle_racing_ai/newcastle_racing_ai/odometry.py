import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance
import numpy as np
from math import hypot, atan2 as math
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from .parameters import PARAMETERS

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        imu_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, self.get_parameter("imu_topic").get_parameter_value().string_value, self.imu_callback, imu_qos)
        self.odom_publisher = self.create_publisher(Odometry, self.get_parameter("odom_topic").get_parameter_value().string_value, 10)
        self.timer = self.create_timer(0.01, self.publish_odometry)
        self.prev_time = None
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.imu = Imu()
        self.vehicle_state

    def imu_callback(self, msg):
        self.imu = msg


    def publish_odometry(self):
        imu = self.imu
        #self.get_logger().info(f"Received IMU data: {imu.header.stamp.sec}.{imu.header.stamp.nanosec} - Acceleration: {imu.linear_acceleration.x}, {imu.linear_acceleration.y}, {imu.linear_acceleration.z}")
        current_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        acc = np.array([
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z
        ])

        self.velocity += acc * dt
        self.position += self.velocity * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = imu.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
        odom_msg.pose.pose.orientation = imu.orientation
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]
        odom_msg.twist.twist.angular = imu.angular_velocity

        self.odom_publisher.publish(odom_msg)

    """
    Things to research/understand
    - Does ROS keep scope with old values are all values volatile upon each message
        - maybe publish message data then retrieve old data from /odom or /imu 
        - learn how ROS actually works
    - What data is available from odometries such as IMU or magnetometers
    - Particle and Kalman filters for error correction
    """

    def vehicle_state_calc(self):
        imu = self.imu
        
        # Point of the 3 position coordinates
        position_old = Point(x=0,y=0,z=0)
        position = Point(x=0, y=0, z=0)
        current_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        dt = current_time - self.prev_time
       
        angular_velocity = imu.angular_velocity
        orientation = imu.orientation
       
        acceleration = np.array([
           imu.linear_acceleration.x,
           imu.linear_acceleration.y,
           imu.linear_acceleration.z
        ])
       
        # rough velocity calculation assuming v0 is always = 0
        # linear_velocity_x = imu.linear_acceleration.x * dt
        # linear_velocity_y = imu.linear_acceleration.y * dt
        # linear_velocity_z = imu.linear_acceleration.z * dt
        # velocity = math.hypot(linear_velocity_x, linear_velocity_y)
        
        # slightly better velocity calculation
        linear_velocity_x_old = 0
        linear_velocity_y_old = 0
        linear_velocity_z_old = 0
        # V.old values for velocity
        linear_velocity_x_old = acceleration[0] * dt
        linear_velocity_y_old = acceleration[1] * dt
        linear_velocity_z_old = acceleration[2] * dt
        
        # V.new values for velocity
        linear_velocity_x_new = linear_velocity_x_old + acceleration[0] * dt
        linear_velocity_y_new = linear_velocity_y_old + acceleration[1] * dt
        linear_velocity_z_new = linear_velocity_z_old + acceleration[2] * dt     
  
        # rough position calculation assuming position is always from 0
        # position.x = linear_velocity_x * dt
        # position.y = linear_velocity_y * dt
        # position.z = linear_velocity_z * dt
        
        # slightly better position calculation
        position_old.x = linear_velocity_x_old * dt
        position_old.y = linear_velocity_y_old * dt
        position_old.z = linear_velocity_z_old * dt
        
        position.x = position_old.x + linear_velocity_x_old * dt + 0.5*acceleration[0]*(dt*dt)
        position.y = position_old.y + linear_velocity_y_old * dt + 0.5*acceleration[1]*(dt*dt)
        position.z = position_old.z + linear_velocity_z_old * dt + 0.5*acceleration[2]*(dt*dt)
        
        # overwriting old variables
        linear_velocity_x_old = linear_velocity_x_new
        linear_velocity_y_old = linear_velocity_y_new
        linear_velocity_z_old = linear_velocity_z_new
        position_old.x = position.x
        position_old.y = position.y
        position_old.z = position.z
        
        # calculating yaw from sin(y)+cos(p) and cos(y)+cos(p)
        siny_cosp = 2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y))
        cosy_cosp = -1.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z))
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # linear_velocity = np.array([
        #     linear_velocity_x,
        #     linear_velocity_y,
        #     linear_velocity_z
        # ])
        
        self.vehicle_state = [position, yaw]
    
def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
