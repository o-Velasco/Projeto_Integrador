import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np

class MultiDroneTargetSeeking(Node):
    def __init__(self):
        super().__init__('multi_drone_target_seeking')

        # Define drone configs: (prefix, target)
        self.drones = [
            {'prefix': '/crazyflie0', 'target': [2.0, 2.0, 2.0]},
            {'prefix': '/crazyflie1', 'target': [1.0, 2.0, 0.3]},
            {'prefix': '/crazyflie2', 'target': [2.0, 1.0, 0.0]},
            {'prefix': '/crazyflie3', 'target': [1.0, -1.0, 1.0]},
            {'prefix': '/crazyflie4', 'target': [1.5, 1.5, 0.5]},
        ]
        self.current_drone_idx = 0

        self.declare_parameter('delay', 2.0)
        self.delay = self.get_parameter('delay').value
        self.declare_parameter('use_position_control', True)
        self.use_position_control = self.get_parameter('use_position_control').value
        self.declare_parameter('ganho', 2.0)
        self.ganho = self.get_parameter('ganho').value
        self.declare_parameter('max_speed', 0.5)
        self.max_speed = self.get_parameter('max_speed').value
        self.declare_parameter('minimun_height', 0.5)
        self.minimun_height = self.get_parameter('minimun_height').value

        # Per-drone publishers, subscribers, and state
        self.twist_publishers = []
        self.odom_subscribers = []
        self.positions = [[0.0, 0.0, 0.0] for _ in self.drones]
        self.has_position_data = [False for _ in self.drones]

        for idx, drone in enumerate(self.drones):
            pub = self.create_publisher(Twist, f'{drone["prefix"]}/cmd_vel', 10)
            self.twist_publishers.append(pub)
            sub = self.create_subscription(
                Odometry,
                f'{drone["prefix"]}/odom',
                lambda msg, i=idx: self.odom_callback(msg, i),
                10
            )
            self.odom_subscribers.append(sub)

        self.takeoff_height = 0.1
        self.position_threshold = 0.05
        self.takeoff_flag = False
        self.flight_stage = 0

        self.get_logger().info("Multi-drone target seeking initialized.")
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(0.1, self.flight_sequence_timer)

    def odom_callback(self, msg, idx):
        self.positions[idx][0] = msg.pose.pose.position.x
        self.positions[idx][1] = msg.pose.pose.position.y
        self.positions[idx][2] = msg.pose.pose.position.z
        self.has_position_data[idx] = True

    def distance_to_target(self, idx, axis):
        if not self.has_position_data[idx]:
            return float('inf')
        return self.drones[idx]['target'][axis] - self.positions[idx][axis]

    def is_at_target(self, idx):
        if not self.has_position_data[idx]:
            return False
        return all(
            abs(self.positions[idx][i] - self.drones[idx]['target'][i]) < self.position_threshold
            for i in range(3)
        )

    def is_at_z(self, idx, z):
        if not self.has_position_data[idx]:
            return False
        return abs(self.positions[idx][2] - z) < self.position_threshold

    def flight_sequence_timer(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time = current_time - self.start_clock

        if self.current_drone_idx >= len(self.drones):
            self.get_logger().info("All drones have reached their targets. ✅")
            self.timer.cancel()
            return

        idx = self.current_drone_idx
        drone = self.drones[idx]
        pub = self.twist_publishers[idx]

        if elapsed_time < self.delay:
            return

        if int(elapsed_time * 10) % 10 == 0 and self.has_position_data[idx]:
            self.get_logger().info(
                f"[{drone['prefix']}] Position: {self.positions[idx][0]:.2f}, {self.positions[idx][1]:.2f}, {self.positions[idx][2]:.2f}, Stage: {self.flight_stage}"
            )

        if self.flight_stage == 0:
            # Takeoff
            if not self.takeoff_flag:
                self.get_logger().info(f"[{drone['prefix']}] 🚁 Takeoff initiated...")
                self.takeoff_flag = True

            msg = Twist()
            msg.linear.z = 0.01
            pub.publish(msg)

            if self.use_position_control and self.is_at_z(idx, self.takeoff_height):
                self.get_logger().info(f"[{drone['prefix']}] Takeoff height reached.")
                self.flight_stage = 1
                self.start_clock = current_time

        elif self.flight_stage == 1:
            if self.has_position_data[idx]:
                msg = Twist()
                erro_x = self.distance_to_target(idx, 0)
                erro_y = self.distance_to_target(idx, 1)
                erro_z = self.distance_to_target(idx, 2)
                erro_z_minimum = self.minimun_height - self.positions[idx][2]

                # Proportional control
                msg.linear.x = np.clip(self.ganho * erro_x, -self.max_speed, self.max_speed)
                msg.linear.y = np.clip(self.ganho * erro_y, -self.max_speed, self.max_speed)
                if drone['target'][2] <= 0.5:
                    msg.linear.z = np.clip(self.ganho * erro_z_minimum, -self.max_speed, self.max_speed)
                    if abs(erro_x) < self.position_threshold and abs(erro_y) < self.position_threshold:
                        msg.linear.z = np.clip(self.ganho * erro_z, -self.max_speed, self.max_speed)
                else:
                    msg.linear.z = np.clip(self.ganho * erro_z, -self.max_speed, self.max_speed)

                pub.publish(msg)

                if self.use_position_control and self.is_at_target(idx):
                    self.get_logger().info(f"[{drone['prefix']}] Arrived at target: {self.positions[idx]}")
                    stop_msg = Twist()
                    pub.publish(stop_msg)
                    self.flight_stage = 0
                    self.takeoff_flag = False
                    self.current_drone_idx += 1
                    self.start_clock = current_time
                    self.get_logger().info(f"Moving to next drone (index {self.current_drone_idx})")

def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneTargetSeeking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Flight interrupted by user')
    finally:
        # Stop all drones before shutting down
        stop_msg = Twist()
        for pub in node.twist_publishers:
            pub.publish(stop_msg)
        rclpy.shutdown()

if __name__ == '__main__':
    main()