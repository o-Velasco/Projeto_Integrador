import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np

class AutonomousCrazyflieFlight(Node):
    def __init__(self):
        super().__init__('target_seeking')

        # Declare parameters
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.declare_parameter('delay', 2.0)
        self.delay = self.get_parameter('delay').value
        self.declare_parameter('use_position_control', True)
        self.use_position_control = self.get_parameter('use_position_control').value
        self.declare_parameter('takeoff_flag', False)
        self.takeoff_flag = self.get_parameter('takeoff_flag').value
        self.declare_parameter('ganho', 2.0)
        self.ganho = self.get_parameter('ganho').value
        self.declare_parameter('target', [2.0, 2.0, 0.0]) # [x, y, z]
        self.target = self.get_parameter('target').value
        self.declare_parameter('max_speed', 0.5)
        self.max_speed = self.get_parameter('max_speed').value
        self.declare_parameter('minimun_height', 0.5)
        self.minimun_height = self.get_parameter('minimun_height').value

        # Publisher for velocity commands
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for odometry to get position data
        self.odom_subscriber = self.create_subscription(
            Odometry, 
            f'{self.robot_prefix}/odom', 
            self.odom_callback, 
            10
        )
        
        # Initialize position tracking
        self.current_position = [0.0, 0.0, 0.0]
        self.has_position_data = False
        
        # Flight parameters
        self.takeoff_height = 0.1   
        self.position_threshold = 0.05  # How close we need to be to target position
               
        # Log initialization
        self.get_logger().info(f"Rise and land test initialized for {self.robot_prefix}")
        self.get_logger().info(f"Will execute flight sequence after {self.delay} seconds")
        if self.use_position_control:
            self.get_logger().info("Using position-based state transitions")
        else:
            self.get_logger().info("Using time-based state transitions")
        
        # Create a timer to start the flight sequence after the delay
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        self.flight_stage = 0
        self.timer = self.create_timer(0.1, self.flight_sequence_timer)

    def odom_callback(self, msg):
        """Process odometry data to get current position"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        self.current_position[2] = msg.pose.pose.position.z
        self.has_position_data = True

    def distance_to_x_target(self, x_target):
        """Distance between drone and target on the x axis"""
        if not self.has_position_data:
            return float('inf')
        return (x_target - self.current_position[0])
    
    def distance_to_y_target(self, y_target):
        """Distance between drone and target on the y axis"""
        if not self.has_position_data:
            return float('inf')
        return (y_target - self.current_position[1])
    
    def distance_to_z_target(self, z_target):
        """Distance between drone and target on the z axis"""
        if not self.has_position_data:
            return float('inf')
        return (z_target - self.current_position[2])
    
    def is_at_x_target(self, x_target):
        """Check if drone is at target x position (within threshold)"""
        if not self.has_position_data:
            return False
        return abs(self.current_position[0] - x_target) < self.position_threshold
    
    def is_at_y_target(self, y_target):
        """Check if drone is at target y position (within threshold)"""
        if not self.has_position_data:
            return False
        return abs(self.current_position[1] - y_target) < self.position_threshold
    
    def is_at_z_target(self, z_target):
        """Check if drone is at target z position (within threshold)"""
        if not self.has_position_data:
            return False
        return abs(self.current_position[2] - z_target) < self.position_threshold
    
    def flight_sequence_timer(self):
        """Flight sequence executed via timer callbacks"""
        # Get current time
        current_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time = current_time - self.start_clock
        
        # Wait for delay before starting
        if elapsed_time < self.delay:
            return
            
        # Debug position info occasionally
        if int(elapsed_time * 10) % 10 == 0:  # Approximately once per second
            if self.has_position_data:
                self.get_logger().info(f"Current position: {self.current_position[0]:.2f}m, {self.current_position[1]:.2f}m, {self.current_position[2]:.2f}m, Flight stage: {self.flight_stage}")
        
        # Execute flight sequence based on stages
        if self.flight_stage == 0:
            # Takeoff
            if not self.takeoff_flag:
                self.get_logger().info('🚁 Takeoff initiated...')
                self.takeoff_flag = True
            
            msg = Twist()
            msg.linear.z = 0.01  # Slow initial ascent
            self.twist_publisher.publish(msg)
            
            # Check if we have reached the takeoff height
            position_condition = self.use_position_control and self.is_at_z_target(self.takeoff_height)
            if position_condition:
                self.get_logger().info(f'Position threshold reached: {self.current_position[2]:.2f}m')
                                
                # Proceed to target seeking stage
                self.get_logger().info('🎯 Seeking target...')
                self.flight_stage = 1
                self.start_clock = current_time  # Reset timer for next stage
        
        elif self.flight_stage == 1:
            # Target seeking logic
            if self.has_position_data:
                msg = Twist()
                
                # Calculate errors in x, y, z directions
                erro_x = self.distance_to_x_target(self.target[0])
                erro_y = self.distance_to_y_target(self.target[1])
                erro_z = self.distance_to_z_target(self.target[2])
                erro_z_minimum = self.distance_to_z_target(self.minimun_height)

                # Check if we are close to the target position
                position_condition_x = self.is_at_x_target(self.target[0])
                position_condition_y = self.is_at_y_target(self.target[1])
                position_condition_z = self.is_at_z_target(self.target[2])
                
                # Apply proportional control
                msg.linear.x = np.clip(self.ganho * erro_x, -self.max_speed, self.max_speed)
                msg.linear.y = np.clip(self.ganho * erro_y, -self.max_speed, self.max_speed)
                if self.target[2]<=0.5:
                    msg.linear.z = np.clip(self.ganho * erro_z_minimum, -self.max_speed, self.max_speed)
                    if position_condition_x and position_condition_y:
                        msg.linear.z = np.clip(self.ganho * erro_z, -self.max_speed, self.max_speed)    
                else:
                    msg.linear.z = np.clip(self.ganho * erro_z, -self.max_speed, self.max_speed)
                         
                # Publish the velocity command
                self.twist_publisher.publish(msg)

                if self.use_position_control:
                    
                    if position_condition_x and position_condition_y and position_condition_z:
                        self.get_logger().info(f'Position threshold reached: {self.current_position[0]:.2f}m, {self.current_position[1]:.2f}m, {self.current_position[2]:.2f}m')
                        msg = Twist()
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.linear.z = 0.0
                        self.twist_publisher.publish(msg)
                        self.get_logger().info('Arrived at target position')
                        self.get_logger().info('✅ Flight complete.')
                        
                        # Cancel the timer
                        self.timer.cancel()
                        # Final stop command
                        msg = Twist()
                        self.twist_publisher.publish(msg)
                        
def main(args=None):
    rclpy.init(args=args)
    autonomous_flight = AutonomousCrazyflieFlight()
    
    try:
        rclpy.spin(autonomous_flight)
    except KeyboardInterrupt:
        autonomous_flight.get_logger().info('Flight interrupted by user')
    finally:
        # Stop all motion before shutting down
        stop_msg = Twist()
        autonomous_flight.twist_publisher.publish(stop_msg)
        autonomous_flight.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()