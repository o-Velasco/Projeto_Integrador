import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class MultiDroneRelativeTarget(Node):
    def __init__(self):
        super().__init__('multi_drone_relative_target')

        self.objectives = [
            [5.0, -3.0, 1.0], # Objetivo 1
            [6.0, 4.0, 2.0], # Objetivo 2
            [-5.5, 5.5, 0.0] # Landing Site
        ]

        self.num_drones = 5 #(Drones 0 through 4)
        self.leader_idx = 2  # crazyflie2 is the leader
        self.drones = [{'prefix': f'/crazyflie{i}'} for i in range(self.num_drones)]

        self.ganho = 2.0
        self.max_speed = 0.3
        self.takeoff_height = 0.1
        self.position_threshold = 0.07

        self.obstacles = [
            (np.array([2.0, -2.0, 1.0]), 1.0),
            (np.array([5.0, 1.0, 2.0]), 1.0)
        ]
        self.x_lim = [-7, 7]
        self.y_lim = [-7, 7]
        self.k1o = 2.0
        self.c = 1.0

        self.stage = [0 for _ in range(self.num_drones)]
        self.last_stage = [0 for _ in range(self.num_drones)]
        self.positions = [None for _ in range(self.num_drones)]
        self.has_pos = [False for _ in range(self.num_drones)]

        self.cmd_publishers = []
        for drone in self.drones:
            pub = self.create_publisher(Twist, f"{drone['prefix']}/cmd_vel", 10)
            self.cmd_publishers.append(pub)

        # Subscribe to each drone's odometry
        for i in range(self.num_drones):
            self.create_subscription(Odometry, f'/crazyflie{i}/odom', self.make_odom_cb(i), 10)

        self.get_logger().info("Multi-drone absolute target-following script started.")
        self.timer = self.create_timer(0.1, self.control_loop)

    def make_odom_cb(self, idx):
        def cb(msg):
            self.positions[idx] = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]
            self.has_pos[idx] = True
        return cb

    def get_absolute_position(self, idx):
        if not self.has_pos[idx]:
            return None
        return self.positions[idx]

    def at_position(self, idx, target):
        pos = self.get_absolute_position(idx)
        if pos is None:
            return False
        return all(abs(pos[i] - target[i]) < self.position_threshold for i in range(3))

    def at_height(self, idx, z):
        pos = self.get_absolute_position(idx)
        if pos is None:
            return False
        return abs(pos[2] - z) < self.position_threshold

    def obstacle_avoidance(self, pos, vel):
        avoidance = np.zeros(3)
        for center, radius in self.obstacles:
            d_vec = pos - center
            d = np.linalg.norm(d_vec)
            d1 = d - radius
            if d1 < 1.5:
                k1o = self.k1o * np.exp(-self.c * d1**2)
                if d > 1e-3:
                    avoidance += (k1o / (d1**2 + 1e-6)) * (d_vec / d)
        return avoidance

    def saturate_area(self, pos, cmd):
        x, y, z = pos
        if x < self.x_lim[0] + 0.5 and cmd[0] < 0:
            cmd[0] = 0
        if x > self.x_lim[1] - 0.5 and cmd[0] > 0:
            cmd[0] = 0
        if y < self.y_lim[0] + 0.5 and cmd[1] < 0:
            cmd[1] = 0
        if y > self.y_lim[1] - 0.5 and cmd[1] > 0:
            cmd[1] = 0
        return cmd

    def saturate_inter_drone_distance(self, idx, pos, cmd):
        min_dist = 0.2 # Increased minimum distance between drones
        for j in range(self.num_drones):
            if j == idx:
                continue
            other_pos = self.get_absolute_position(j)
            if other_pos is None:
                continue
            rel_vec = np.array(pos) - np.array(other_pos)
            dist = np.linalg.norm(rel_vec)
            if dist < min_dist:
                rel_dir = rel_vec / (dist + 1e-6)
                cmd_proj = np.dot(cmd, rel_dir)
                if cmd_proj < 0:
                    cmd = cmd - cmd_proj * rel_dir
        return cmd

    def control_loop(self):
        for idx in range(self.num_drones):
            pos = self.get_absolute_position(idx)
            if pos is None:
                msg = Twist()
                msg.linear.z = 0.2
                self.publish_cmd(idx, msg)
                continue

            msg = Twist()
            pos_np = np.array(pos)
            if self.stage[idx] == 0:
                if not self.at_height(idx, self.takeoff_height):
                    msg.linear.z = 0.2
                    self.publish_cmd(idx, msg)
                else:
                    self.stage[idx] = 1
                    self.get_logger().info(f"{self.drones[idx]['prefix']} reached takeoff height and knows its position.")
            elif self.stage[idx] in [1, 2]:
                if idx == self.leader_idx:
                    target = self.objectives[self.stage[idx] - 1]
                    min_z = 0.5
                    xy_threshold = 0.1
                    error = np.array([target[i] - pos[i] for i in range(3)])
                    if target[2] < 0.3:
                        xy_dist = np.linalg.norm(error[:2])
                        if xy_dist > xy_threshold:
                            error[2] = min_z - pos[2]
                        else:
                            error[2] = target[2] - pos[2]
                    vel_cmd = self.ganho * error
                else:
                    leader_pos = self.get_absolute_position(self.leader_idx)
                    if leader_pos is None:
                        continue
                    rel_vec = np.array(pos) - np.array(leader_pos)
                    error = -rel_vec
                    vel_cmd = self.ganho * error

                avoid = self.obstacle_avoidance(pos_np, vel_cmd)
                total_cmd = vel_cmd + avoid
                total_cmd = np.clip(total_cmd, -self.max_speed, self.max_speed)
                total_cmd = self.saturate_area(pos_np, total_cmd)
                total_cmd = self.saturate_inter_drone_distance(idx, pos_np, total_cmd)
                msg.linear.x = total_cmd[0]
                msg.linear.y = total_cmd[1]
                msg.linear.z = total_cmd[2]
                # Clamp final command norm
                max_cmd = self.max_speed  # or your safe value
                cmd_norm = np.linalg.norm([msg.linear.x, msg.linear.y, msg.linear.z])
                if cmd_norm > max_cmd:
                    scale = max_cmd / (cmd_norm + 1e-6)
                    msg.linear.x *= scale
                    msg.linear.y *= scale
                    msg.linear.z *= scale
                # NaN/Inf safeguard
                import math
                if not all(math.isfinite(v) for v in [msg.linear.x, msg.linear.y, msg.linear.z]):
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                self.publish_cmd(idx, msg)
                if idx == self.leader_idx and self.at_position(idx, self.objectives[self.stage[idx] - 1]):
                    self.get_logger().info(f"{self.drones[idx]['prefix']} reached target {self.stage[idx]}")
                    self.stage[idx] += 1
                    if self.stage[idx] != self.last_stage[idx]:
                        self.get_logger().info(f"cf{idx} moved to stage{self.stage[idx]}")
                        self.last_stage[idx] = self.stage[idx]
                elif idx != self.leader_idx and self.stage[self.leader_idx] > self.stage[idx]:
                    self.stage[idx] = self.stage[self.leader_idx]
                    if self.stage[idx] != self.last_stage[idx]:
                        self.get_logger().info(f"cf{idx} moved to stage{self.stage[idx]}")
                        self.last_stage[idx] = self.stage[idx]
            elif self.stage[idx] == 3:
                if idx == self.leader_idx:
                    # Leader: go to landing site (third target)
                    target = self.objectives[2]
                    min_z = 0.5
                    xy_threshold = 0.1
                    error = np.array([target[i] - pos[i] for i in range(3)])
                    if target[2] < 0.3:
                        xy_dist = np.linalg.norm(error[:2])
                        if xy_dist > xy_threshold:
                            error[2] = min_z - pos[2]
                        else:
                            error[2] = target[2] - pos[2]
                    vel_cmd = self.ganho * error
                else:
                    # Followers: always move toward leader
                    leader_pos = self.get_absolute_position(self.leader_idx)
                    if leader_pos is None:
                        continue
                    rel_vec = np.array(pos) - np.array(leader_pos)
                    error = -rel_vec
                    vel_cmd = self.ganho * error

                avoid = self.obstacle_avoidance(pos_np, vel_cmd)
                total_cmd = vel_cmd + avoid
                total_cmd = np.clip(total_cmd, -self.max_speed, self.max_speed)
                total_cmd = self.saturate_area(pos_np, total_cmd)
                total_cmd = self.saturate_inter_drone_distance(idx, pos_np, total_cmd)
                msg.linear.x = total_cmd[0]
                msg.linear.y = total_cmd[1]
                msg.linear.z = total_cmd[2]
                # Clamp final command norm
                max_cmd = 0.3
                cmd_norm = np.linalg.norm([msg.linear.x, msg.linear.y, msg.linear.z])
                if cmd_norm > max_cmd:
                    scale = max_cmd / (cmd_norm + 1e-6)
                    msg.linear.x *= scale
                    msg.linear.y *= scale
                    msg.linear.z *= scale
                import math
                if not all(math.isfinite(v) for v in [msg.linear.x, msg.linear.y, msg.linear.z]):
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                self.publish_cmd(idx, msg)
                if idx == self.leader_idx and self.at_position(idx, self.objectives[2]):
                    self.get_logger().info(f"{self.drones[idx]['prefix']} reached landing site and is waiting for followers.")
                    self.stage[idx] = 4
                    if self.stage[idx] != self.last_stage[idx]:
                        self.get_logger().info(f"cf{idx} moved to stage{self.stage[idx]}")
                        self.last_stage[idx] = self.stage[idx]
                elif idx != self.leader_idx and self.stage[self.leader_idx] > self.stage[idx]:
                    self.stage[idx] = self.stage[self.leader_idx]
                    if self.stage[idx] != self.last_stage[idx]:
                        self.get_logger().info(f"cf{idx} moved to stage{self.stage[idx]}")
                        self.last_stage[idx] = self.stage[idx]
            elif self.stage[idx] == 4:
                if idx == self.leader_idx:
                    # Leader hovers in place
                    msg = Twist()
                    pos = self.get_absolute_position(idx)
                    min_z = 0.5
                    if pos is not None and pos[2] < min_z:
                        msg.linear.z = 0.2
                    else:
                        msg.linear.z = 0.0
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    self.publish_cmd(idx, msg)

                    # Check if all followers are within 1m in xy
                    all_within_radius = False
                    pos = self.get_absolute_position(idx)
                    if pos is not None:
                        all_within_radius = True
                        for j in range(self.num_drones):
                            if j == self.leader_idx:
                                continue
                            follower_pos = self.get_absolute_position(j)
                            if follower_pos is None:
                                all_within_radius = False
                                break
                            rel_vec = np.array(follower_pos) - np.array(pos)
                            dist_xy = np.linalg.norm(rel_vec[:2])
                            if dist_xy > 1.0:
                                all_within_radius = False
                                break
                    if all_within_radius:
                        self.get_logger().info("All followers are within 1m radius. Leader will start landing.")
                        self.stage[idx] = 5
                        if self.stage[idx] != self.last_stage[idx]:
                            self.get_logger().info(f"cf{idx} moved to stage{self.stage[idx]}")
                            self.last_stage[idx] = self.stage[idx]
                else:
                    # Followers approach their offset positions around the leader in xy, do not descend
                    leader_pos = self.get_absolute_position(self.leader_idx)
                    if leader_pos is not None:
                        offset = np.array([0.0, 0.0, 0.0])
                        if idx == 0:
                            offset = np.array([0.3, 0.0, 0.0])   # +x
                        elif idx == 1:
                            offset = np.array([-0.3, 0.0, 0.0])  # -x
                        elif idx == 3:
                            offset = np.array([0.0, 0.3, 0.0])   # +y
                        elif idx == 4:
                            offset = np.array([0.0, -0.3, 0.0])  # -y
                        target = np.array(leader_pos) + offset
                        pos = self.get_absolute_position(idx)
                        if pos is not None:
                            error = target - np.array(pos)
                            error[2] = 0  # Prevent descent in Stage 4
                            # Check if at offset position
                            if np.linalg.norm(error[:2]) < self.position_threshold:
                                self.stage[idx] = 5
                                if self.stage[idx] != self.last_stage[idx]:
                                    self.get_logger().info(f"cf{idx} moved to stage{self.stage[idx]}")
                                    self.last_stage[idx] = self.stage[idx]
                            else:
                                cmd = self.ganho * error
                                speed = np.linalg.norm(cmd)
                                if speed > self.max_speed:
                                    cmd = cmd / speed * self.max_speed
                                msg = Twist()
                                msg.linear.x = cmd[0]
                                msg.linear.y = cmd[1]
                                msg.linear.z = cmd[2]
                                self.publish_cmd(idx, msg)

            elif self.stage[idx] == 5:
                # All drones land immediately in stage 5, no need to reach offset
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = -0.2  # Command descent
                self.publish_cmd(idx, msg)

        # Remove or comment out the following lines to prevent stopping the simulation:
        # if all(s == 5 for s in self.stage):
        #     self.get_logger().info("All drones have completed their missions.")
        #     self.timer.cancel()

    def publish_cmd(self, idx, msg):
        self.cmd_publishers[idx].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneRelativeTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Flight interrupted by user')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()