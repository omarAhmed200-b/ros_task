#!/usr/bin/env python3


################################## ana ha7ot 3la ali hani ya ba4mohandes #################################
################################## da rkmy 01096009904 vcash m4 insta   #################################


import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

WALLS = {
    "F": "front_wall",  # first approach wall
    "B": "back_wall",
    "L": "left_wall",
    "R": "right_wall",
}


# second approach wall  win


class WallFaster(Node):
    def __init__(self):
        super().__init__(
            "wall_faster",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.heading = math.pi / 4
        self.forward_speed = 4.0
        self.reverse_speed = 3.0
        self.acceleration = 10.0
        self.heading_kp = 8.0
        self.cross_track_gain = 1.5
        self.engage_angle = math.radians(8.0)
        self.max_time = 18.0
        if self.heading >= 0.0:
            self.first_pair = ("F", "L")
            self.second_pair = ("B", "R")
        else:
            self.first_pair = ("F", "R")
            self.second_pair = ("B", "L")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Odometry, "/model/edges_robot/odometry", self.odom_callback, 20
        )
        self.angles = {key: None for key in WALLS}
        for key, name in WALLS.items():
            self.create_subscription(
                JointState,
                f"/world/walls_world/model/{name}/joint_state",
                lambda msg, wall=key: self.joint_callback(wall, msg),
                20,
            )
        self.create_timer(0.02, self.control)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.start_x = None
        self.start_y = None
        self.start_time = None
        self.last_time = None
        self.motion_start_real = None
        self.motion_start_sim = None
        self.mission_end_real = None
        self.mission_end_sim = None
        self.current_speed = 0.0
        self.state = "FORWARD"
        self.first_impact_time = None
        self.first_impact_yaw = None
        self.first_confirmed_time = None
        self.recovery_started = None
        self.second_impact_time = None
        self.second_impact_yaw = None
        self.max_deflection = 0.0
        self.fall_started = {key: None for key in WALLS}
        self.fall_times = {}
        self.finished = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),  # 2.5
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.have_odom = True

    def joint_callback(self, wall, msg):
        hinge = f"{WALLS[wall].replace('_wall', '')}_hinge"
        if hinge in msg.name:
            self.angles[wall] = msg.position[msg.name.index(hinge)]

    def elapsed(self):
        return 0.0 if self.start_time is None else self.now() - self.start_time

    def update_falls(self):
        current = self.now()
        for wall, angle in self.angles.items():
            if angle is None or wall in self.fall_times:
                continue
            if abs(angle) >= math.radians(60.0):
                if self.fall_started[wall] is None:
                    self.fall_started[wall] = current
                elif current - self.fall_started[wall] >= 0.05:
                    self.fall_times[wall] = self.elapsed()
                    if len(self.fall_times) == 4 and self.motion_start_real is not None:
                        self.mission_end_real = time.perf_counter()
                        self.mission_end_sim = self.now()
                    self.get_logger().info(
                        f"FALLEN wall={wall} time={self.fall_times[wall]:.3f}"
                    )
            else:
                self.fall_started[wall] = None

    def publish(self, target_speed):
        current = self.now()
        dt = (
            0.02
            if self.last_time is None
            else max(0.0, min(0.1, current - self.last_time))
        )
        self.last_time = current
        step = self.acceleration * dt
        self.current_speed += max(-step, min(step, target_speed - self.current_speed))
        base_error = self.wrap(self.heading - self.yaw)
        self.max_deflection = max(self.max_deflection, abs(base_error))
        cross_track = -math.sin(self.heading) * (self.x - self.start_x) + math.cos(
            self.heading
        ) * (self.y - self.start_y)
        correction_sign = 1.0 if target_speed < 0.0 else -1.0
        path_correction = max(
            -math.radians(12.0),
            min(
                math.radians(12.0),
                correction_sign * self.cross_track_gain * cross_track,
            ),
        )
        target_heading = self.heading + path_correction
        error = self.wrap(target_heading - self.yaw)
        msg = Twist()
        msg.linear.x = self.current_speed
        msg.angular.z = max(-2.0, min(2.0, self.heading_kp * error))
        if self.motion_start_real is None and (
            msg.linear.x != 0.0 or msg.angular.z != 0.0
        ):
            self.motion_start_real = time.perf_counter()
            self.motion_start_sim = self.now()
        self.publisher.publish(msg)

    def stop(self):
        self.current_speed = 0.0
        msg = Twist()
        self.publisher.publish(msg)

    def pair_engaged(self, pair):
        return all(abs(self.angles[wall]) >= self.engage_angle for wall in pair)

    def control(self):
        if self.finished:
            return
        if not self.have_odom or any(value is None for value in self.angles.values()):
            return
        if self.start_time is None:
            self.start_time = self.now()
            self.last_time = self.start_time
            self.start_x = self.x
            self.start_y = self.y
            self.get_logger().info(
                f"START heading_deg={math.degrees(self.heading):.3f} first={self.first_pair} second={self.second_pair}"
            )
        self.update_falls()
        if len(self.fall_times) == 4:
            self.finish(True, "all_walls_fallen")
            return
        if self.elapsed() >= self.max_time:
            self.finish(False, "timeout")
            return
        if self.state == "FORWARD":
            self.publish(self.forward_speed)
            if self.pair_engaged(self.first_pair):
                self.first_impact_time = self.elapsed()
                self.first_impact_yaw = self.yaw
                self.state = "VERIFY_FIRST"
                self.get_logger().info(
                    f"PAIR1_ENGAGED time={self.first_impact_time:.3f} yaw_deg={math.degrees(self.yaw):.3f} "
                    f"angles_deg={[round(math.degrees(self.angles[w]), 3) for w in self.first_pair]}"
                )
        elif self.state == "VERIFY_FIRST":
            self.publish(0.0)
            if all(wall in self.fall_times for wall in self.first_pair):
                self.first_confirmed_time = self.elapsed()
                self.state = "RECOVER"
                self.get_logger().info(
                    f"PAIR1_CONFIRMED time={self.first_confirmed_time:.3f} "
                    f"yaw_deg={math.degrees(self.yaw):.3f}"
                )
                self.get_logger().info(
                    f"REVERSE_START time={self.elapsed():.3f} yaw_deg={math.degrees(self.yaw):.3f}"
                )
        elif self.state == "RECOVER":
            self.publish(-0.5)
            aligned = abs(self.wrap(self.heading - self.yaw)) <= math.radians(3.0)
            if aligned:
                if self.recovery_started is None:
                    self.recovery_started = self.now()
                elif self.now() - self.recovery_started >= 0.1:
                    self.state = "REVERSE"
                    self.get_logger().info(
                        f"FULL_REVERSE time={self.elapsed():.3f} yaw_deg={math.degrees(self.yaw):.3f}"
                    )
            else:
                self.recovery_started = None
        else:
            self.publish(-self.reverse_speed)
            if self.second_impact_time is None and self.pair_engaged(self.second_pair):
                self.second_impact_time = self.elapsed()
                self.second_impact_yaw = self.yaw
                self.get_logger().info(
                    f"PAIR2_ENGAGED time={self.second_impact_time:.3f} yaw_deg={math.degrees(self.yaw):.3f} "
                    f"angles_deg={[round(math.degrees(self.angles[w]), 3) for w in self.second_pair]}"
                )

    def finish(self, success, reason):
        self.finished = True
        self.stop()
        real_end = (
            self.mission_end_real
            if self.mission_end_real is not None
            else time.perf_counter()
        )
        sim_end = (
            self.mission_end_sim if self.mission_end_sim is not None else self.now()
        )
        real_time = (
            real_end - self.motion_start_real
            if self.motion_start_real is not None
            else 0.0
        )
        simulation_time = (
            sim_end - self.motion_start_sim
            if self.motion_start_sim is not None
            else 0.0
        )
        first_count = sum(
            abs(self.angles[w]) >= self.engage_angle for w in self.first_pair
        )
        second_count = sum(
            abs(self.angles[w]) >= self.engage_angle for w in self.second_pair
        )
        self.get_logger().info(
            f"RESULT: success={success} | real_time={real_time:.3f} s"
        )
        self.get_logger().info(
            f"SIMULATION_TIME: {simulation_time:.3f} s | reason={reason} "
            f"max_deflection_deg={math.degrees(self.max_deflection):.3f} "
            f"first_count={first_count} second_count={second_count}"
        )
        for key in WALLS:
            value = self.fall_times.get(key)
            self.get_logger().info(
                f"WALL wall={key} status={'FALLEN' if value is not None else 'STANDING'} "
                f"time={value if value is not None else -1.0:.3f} angle_deg={math.degrees(self.angles[key]):.3f}"
            )


def start_bridge():
    arguments = [
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/model/edges_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
    ]
    for name in WALLS.values():
        arguments.append(
            f"/world/walls_world/model/{name}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        )
    return subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge", *arguments],
        start_new_session=True,
    )


def stop_bridge(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    bridge = start_bridge()
    rclpy.init()
    node = WallFaster()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_bridge(bridge)


if __name__ == "__main__":
    main()
