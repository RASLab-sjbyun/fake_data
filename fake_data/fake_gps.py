#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        self.frame_id = "gps_frame"
        self.lat0 = 37.5665      
        self.lon0 = 126.9780
        self.alt0 = 1.0

        self.radius_m = 10.0     # 10m 원
        self.omega = 0.1         # rad/s
        self.rate_hz = 5.0

        # publisher
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)

        # 시간 기준
        self.t0 = self.get_clock().now()

        # timer
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info("Fake GPS publisher started on /fix")

    def on_timer(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # ===== 원형 이동 (ENU) =====
        east_m = self.radius_m * math.cos(self.omega * t)
        north_m = self.radius_m * math.sin(self.omega * t)

        # meters → degrees (근사)
        dlat = north_m / 111_111.0
        dlon = east_m / (111_111.0 * math.cos(math.radians(self.lat0)))

        # ===== 메시지 생성 =====
        msg = NavSatFix()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = self.lat0 + dlat
        msg.longitude = self.lon0 + dlon
        msg.altitude = self.alt0

        # covariance (예시)
        msg.position_covariance = [
            2.25, 0.0, 0.0,
            0.0, 2.25, 0.0,
            0.0, 0.0, 9.0
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
