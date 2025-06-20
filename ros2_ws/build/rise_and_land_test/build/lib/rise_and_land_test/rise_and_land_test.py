"""
# Basic run with default parameters (position control enabled)
ros2 run your_package rise_and_land_test

# Run with time-based control only
ros2 run your_package rise_and_land_test --ros-args -p use_position_control:=false

# Customize all parameters
ros2 run your_package rise_and_land_test --ros-args \
  -p robot_prefix:=/crazyflie \
  -p delay:=3.0 \
  -p use_position_control:=true
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class AutonomousCrazyflieFlight(Node):
    def __init__(self):
        super().__init__('rise_and_land_test')

        # Declare parameters
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.declare_parameter('delay', 2.0)
        self.delay = self.get_parameter('delay').value
        self.declare_parameter('use_position_control', True)
        self.use_position_control = self.get_parameter('use_position_control').value
        self.declare_parameter('takeoff_flag', False)
        self.takeoff_flag = self.get_parameter('takeoff_flag').value

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
        self.target_height = 0.8
        self.landing_height = 0.05
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

    def is_at_target_height(self, target_height):
        """Check if drone is at target height (within threshold)"""
        if not self.has_position_data:
            return False
        return abs(self.current_position[2] - target_height) < self.position_threshold

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
                self.get_logger().info(f"Current height: {self.current_position[2]:.2f}m, Flight stage: {self.flight_stage}")
        
        
        # Execute flight sequence based on stages
        if self.flight_stage == 0:
            # Takeoff
            if not self.takeoff_flag:
                self.get_logger().info('🚁 Takeoff initiated...')
                self.takeoff_flag = True
            
            msg = Twist()
            msg.linear.z = 0.1  # Slow initial ascent
            self.twist_publisher.publish(msg)
            
            time_condition = elapsed_time >= 1000.0
            position_condition = self.use_position_control and self.is_at_target_height(self.takeoff_height)
            if position_condition:
                self.get_logger().info(f'Position threshold reached: {self.current_position[2]:.2f}m')
            if time_condition:
                self.get_logger().info(f'Takeoff time threshold reached: {elapsed_time:.2f}s')
            # Check conditions for next stage
            if time_condition or position_condition:
                self.get_logger().info('⬆️ Ascending higher...')
                self.flight_stage = 1
                self.start_clock = current_time  # Reset timer for next stage
            
        elif self.flight_stage == 1:
            # Transition: Ascend higher
            msg = Twist()
            msg.linear.z = 0.3  # Moderate ascent rate
            self.twist_publisher.publish(msg)
            
            time_condition = elapsed_time >= 1000.0
            position_condition = self.use_position_control and self.is_at_target_height(self.target_height)
            if position_condition:
                self.get_logger().info(f'Position threshold reached: {self.current_position[2]:.2f}m')
            if time_condition:
                self.get_logger().info(f'Takeoff time threshold reached: {elapsed_time:.2f}s')
            # Check conditions for next stage
            if time_condition or position_condition:
                self.get_logger().info('🛑 Hovering...')
                self.flight_stage = 2
                self.start_clock = current_time  # Reset timer for next stage
            
        elif self.flight_stage == 2:
            # Transition: Hover
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            self.twist_publisher.publish(msg)

            time_condition = elapsed_time >= 15.0
            if time_condition:
                self.get_logger().info(f'Hover time threshold reached: {elapsed_time:.2f}s')
            # Check conditions for next stage
            if time_condition:
                self.get_logger().info('⬇️ Descending...')
                self.flight_stage = 3
                self.start_clock = current_time

        elif self.flight_stage == 3:
            # Transition: Begin descent
            msg = Twist()
            msg.linear.z = -0.5  # Moderate descent rate
            self.twist_publisher.publish(msg)

            time_condition = elapsed_time >= 1000.0
            position_condition = self.use_position_control and self.is_at_target_height(self.landing_height)
            if position_condition:
                self.get_logger().info(f'Position threshold reached: {self.current_position[2]:.2f}m')
            if time_condition:
                self.get_logger().info(f'Descent time threshold reached: {elapsed_time:.2f}s')
            # Check conditions for next stage
            if time_condition or position_condition:
                self.get_logger().info('🚁 Final landing...') 
                self.get_logger().info('🚁 Landing complete.')
                self.get_logger().info(f'Landed at height: {self.current_position[2]:.2f}m')
                self.get_logger().info('✅ Flight complete.')

                # Cancel the timer
                self.timer.cancel()
                # Final stop command
                msg = Twist()
                self.twist_publisher.publish(msg)
                """
                self.flight_stage = 4
                self.start_clock = current_time
                """

        """
        elif self.flight_stage == 4:
            # Transition: Final landing phase   
            msg = Twist()
            msg.linear.z = -0.1  # Slow final descent
            self.twist_publisher.publish(msg)
            
            time_condition = elapsed_time >= 60.0
            position_condition = self.use_position_control and self.current_position[2] == 0.0
            if position_condition:
                self.get_logger().info(f'Position threshold reached: {self.current_position[2]:.2f}m')
            if time_condition:
                self.get_logger().info(f'Final landing time threshold reached: {elapsed_time:.2f}s')
            # Check conditions for next stage
            if time_condition or position_condition:
                self.get_logger().info('🚁 Landing complete.')
                self.get_logger().info(f'Landed at height: {self.current_position[2]:.2f}m')
                self.get_logger().info('✅ Flight complete.')

                # Cancel the timer
                self.timer.cancel()
                # Final stop command
                msg = Twist()
                self.twist_publisher.publish(msg)
        """

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

if __name__ == '__main__':
    main()
