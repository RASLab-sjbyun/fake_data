#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


def quat_from_yaw_deg(yaw_deg: float) -> Quaternion:
    """ENU yaw(deg, CCW+) -> quaternion (roll=pitch=0)."""
    yaw = math.radians(yaw_deg)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ImuAbsHeadingPublisher(Node):
    """
    조건:
      - heading: North=0°, CW+
      - ENU yaw: East=0°, CCW+
      - yaw_deg = 90 - heading_deg

    동작:
      - 20초마다 heading이 0->360° 한 바퀴 (CW) 회전
      - 그러면 yaw는 90->89->88...처럼 감소하며 한 바퀴 돌아 원래로 복귀
    """

    def __init__(self):
        super().__init__("imu_abs_heading_pub")

        # ===== 고정 설정 =====
        self.frame_id = "imu_link"
        self.topic = "/imu/data"
        self.rate_hz = 50.0        # publish frequency
        self.period_sec = 20.0     # 1 revolution per 20 seconds

        self.pub = self.create_publisher(Imu, self.topic, 10)

        self.dt = 1.0 / self.rate_hz
        self.heading_deg = 0.0
        self.heading_rate_deg_s = 360.0 / self.period_sec  # CW+

        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"Publishing {self.topic} @ {self.rate_hz}Hz, 1 rev / {self.period_sec}s "
            f"(heading CW rate={self.heading_rate_deg_s:.2f} deg/s)."
        )

    @staticmethod
    def wrap_deg_180(deg: float) -> float:
        """Wrap degrees to [-180, 180)."""
        return (deg + 180.0) % 360.0 - 180.0

    def on_timer(self):
        # 1) heading 증가: North=0°, CW+
        self.heading_deg = (self.heading_deg + self.heading_rate_deg_s * self.dt) % 360.0

        # 2) heading -> ENU yaw (East=0°, CCW+)
        #    heading이 증가(CW)하면 yaw는 감소(CCW 기준)
        yaw_enu_deg = 90.0 - self.heading_deg

        # 보기 좋게 -180~180으로 wrap (RViz/로그에서 해석 쉬움)
        yaw_enu_deg = self.wrap_deg_180(yaw_enu_deg)

        # 3) quaternion 생성
        q = quat_from_yaw_deg(yaw_enu_deg)

        # 4) IMU 메시지 publish
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation = q

        # yaw rate (ROS CCW+). heading은 CW+이므로 부호 반대
        msg.angular_velocity.z = -math.radians(self.heading_rate_deg_s)

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuAbsHeadingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
