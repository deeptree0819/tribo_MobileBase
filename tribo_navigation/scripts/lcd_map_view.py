#!/usr/bin/env python3
"""매핑 중인 맵을 로봇 LCD 에 자동 맞춤으로 띄운다.

왜 RViz 가 아닌가: RViz 의 배율은 설정 파일에 박힌 고정값이라 실행 중에 바꾸려면
마우스 휠이 필요한데, 이 로봇에는 마우스도 키보드도 연결돼 있지 않다(전원 버튼과
HDMI CEC 만 잡힌다). 맵은 매핑 중에 계속 자라므로 고정 배율로는 곧 화면을 벗어난다.

이 뷰어는 /map 을 구독해 **점유된 영역의 경계에 맞춰 매 프레임 배율을 다시 계산**
한다. 맵이 자라면 자동으로 축소되고, 좁은 방이면 확대된다. 입력장치가 필요 없다.

표시:
  흰색   빈 공간(0)
  검정   장애물(100)
  회색   미탐색(-1)
  빨강   로봇 현재 위치 (map -> base_link TF, 없으면 생략)

사용법:
  python3 lcd_map_view.py [맵토픽]
  기본 토픽: /map

  종료: LCD 에서 q 또는 ESC. SSH 에서는 pkill -f "lcd_map_view[.]py"
"""

import math
import re
import subprocess
import sys

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

import tf2_ros

WINDOW = "tribo_lcd_map"

# 맵 가장자리에 남길 여백 비율. 0 이면 벽이 화면 끝에 딱 붙어 답답하다.
MARGIN = 0.94

# 아직 아무것도 안 그려졌을 때 보여줄 최소 범위(미터).
# 이게 없으면 맵이 비었을 때 배율이 무한대로 튄다.
MIN_SPAN_M = 4.0


def screen_size(default=(1024, 600)):
    """X 화면 해상도. fb0 는 프레임버퍼 콘솔 크기라 X 모드와 다를 수 있으므로
    xrandr 을 우선한다(기체 7b6a 에서 fb0 1024x768 vs X 1024x600)."""
    try:
        out = subprocess.run(["xrandr"], capture_output=True, text=True, timeout=5).stdout
        for line in out.splitlines():
            if " connected" in line:
                m = re.search(r"(\d+)x(\d+)\+\d+\+\d+", line)
                if m:
                    return int(m.group(1)), int(m.group(2))
    except Exception:
        pass
    return default


class LcdMapView(Node):
    def __init__(self, topic):
        super().__init__("lcd_map_view")
        self.dst_w, self.dst_h = screen_size()
        self.frames = 0

        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # 맵은 latched(TRANSIENT_LOCAL)로 발행된다. 기본 QoS 로 구독하면
        # 이미 발행된 맵을 못 받아 화면이 계속 비어 있다.
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, topic, self.on_map, qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 마지막으로 그린 화면을 들고 주기적으로 다시 표시한다.
        #
        # 왜 필요한가: 맵이 갱신될 때만 그리면, 갱신이 멈춘 사이에 창 크기가 바뀌어도
        # 다시 그려지지 않는다. 실제로 wmctrl 이 전체화면으로 키운 뒤 새 맵이 안 와서
        # 화면 좌상단 일부에만 그려진 채로 남는 것을 겪었다(map_server 는 맵을 한 번만
        # 발행한다). 로봇 위치도 이 주기로 갱신되어 맵이 멈춰도 움직임이 보인다.
        self.last_msg = None
        self.create_timer(0.2, self.redraw)

        self.get_logger().info(f"구독: {topic}  화면: {self.dst_w}x{self.dst_h}")

    def robot_xy(self, frame_id):
        """map -> base_link 위치. TF 가 아직 없으면 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                frame_id, "base_link", rclpy.time.Time()
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def on_map(self, msg):
        self.last_msg = msg
        self.render(msg)

    def redraw(self):
        if self.last_msg is not None:
            self.render(self.last_msg)

    def render(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return

        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)

        # 회색(미탐색) 바탕에 빈 공간은 희게, 장애물은 검게.
        img = np.full((h, w), 128, dtype=np.uint8)
        img[grid == 0] = 255
        img[grid > 50] = 0

        # OccupancyGrid 는 원점이 좌하단이고 y 가 위로 자란다. 화면은 위가 0 이므로 뒤집는다.
        img = np.flipud(img)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # ---- 자동 맞춤: 실제로 그려진 영역(미탐색이 아닌 셀)의 경계만 본다 ----
        # 전체 격자에 맞추면 slam_toolbox 가 크게 잡아둔 빈 영역까지 포함돼
        # 실제 맵이 화면 구석에 조그맣게 나온다.
        known = np.argwhere(np.flipud(grid) != -1)
        if known.size:
            r0, c0 = known.min(axis=0)
            r1, c1 = known.max(axis=0)
        else:
            r0, c0, r1, c1 = 0, 0, h - 1, w - 1

        span_w = max((c1 - c0 + 1) * res, MIN_SPAN_M)
        span_h = max((r1 - r0 + 1) * res, MIN_SPAN_M)
        scale = min(self.dst_w / span_w, self.dst_h / span_h) * MARGIN

        # 관심 영역의 중심을 화면 중심에 놓는다.
        cx_px, cy_px = (c0 + c1) / 2.0, (r0 + r1) / 2.0

        M = np.float32([
            [scale * res, 0, self.dst_w / 2.0 - cx_px * scale * res],
            [0, scale * res, self.dst_h / 2.0 - cy_px * scale * res],
        ])
        canvas = cv2.warpAffine(
            img, M, (self.dst_w, self.dst_h),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(40, 40, 40),
        )

        # ---- 로봇 위치 ----
        pos = self.robot_xy(msg.header.frame_id or "map")
        if pos is not None:
            # 월드(m) -> 격자 픽셀 -> 화면 픽셀. 격자는 위아래를 뒤집었으므로 y 도 뒤집는다.
            gx = (pos[0] - ox) / res
            gy = (h - 1) - (pos[1] - oy) / res
            sx = int(gx * scale * res + (self.dst_w / 2.0 - cx_px * scale * res))
            sy = int(gy * scale * res + (self.dst_h / 2.0 - cy_px * scale * res))
            if 0 <= sx < self.dst_w and 0 <= sy < self.dst_h:
                cv2.circle(canvas, (sx, sy), 6, (0, 0, 255), -1)

        # ---- 축척 안내 ----
        cv2.putText(
            canvas, f"{span_w:.1f} x {span_h:.1f} m", (12, self.dst_h - 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1, cv2.LINE_AA,
        )

        cv2.imshow(WINDOW, canvas)

        self.frames += 1
        if self.frames <= 3:
            # 전체화면 속성은 창에 내용이 그려지기 전에 걸면 무시될 때가 있다.
            cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        if self.frames == 1:
            self.get_logger().info(
                f"첫 맵: {w}x{h} 셀 x {res} m, 표시 {span_w:.1f}x{span_h:.1f} m"
            )

        if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
            raise KeyboardInterrupt


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/map"
    rclpy.init()
    node = LcdMapView(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
