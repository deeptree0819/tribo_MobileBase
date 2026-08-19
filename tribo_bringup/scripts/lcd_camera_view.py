#!/usr/bin/env python3
"""로봇 LCD 에 카메라 영상을 화면 가득 띄운다.

rqt_image_view 는 창 장식과 툴바가 화면을 먹고, mutter 가 SSH 에서 온 리사이즈
요청을 무시해서 크기를 맞추기도 어렵다. 그래서 OpenCV 전체화면 창에 영상만
그린다 — 장식도 여백도 없다.

화면(1024x600, 약 1.71:1)과 영상(1280x720, 1.78:1)의 비율이 다르므로
cover 방식으로 맞춘다: 짧은 쪽 기준으로 확대한 뒤 넘치는 부분을 잘라낸다.
비율을 유지하면서 여백 없이 꽉 채우기 위한 것 — 늘려 채우면 화면이 찌그러진다.

사용법:
  ros2 run 없이 직접 실행한다.
    python3 lcd_camera_view.py [토픽]
  기본 토픽: /camera/camera/color/image_raw

  종료: LCD 에서 q 또는 ESC. SSH 로 띄웠으면 pkill -f lcd_camera_view
"""

import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

WINDOW = "tribo_lcd_camera"


def cover_fit(img, dst_w, dst_h):
    """비율을 유지한 채 dst 크기를 완전히 덮도록 확대·중앙 크롭."""
    h, w = img.shape[:2]
    if w == 0 or h == 0 or dst_w <= 0 or dst_h <= 0:
        return img

    scale = max(dst_w / w, dst_h / h)
    new_w, new_h = int(w * scale + 0.5), int(h * scale + 0.5)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    x0 = max(0, (new_w - dst_w) // 2)
    y0 = max(0, (new_h - dst_h) // 2)
    return resized[y0:y0 + dst_h, x0:x0 + dst_w]


def screen_size(default=(1024, 600)):
    """프레임버퍼에서 화면 해상도를 읽는다.

    cv2.getWindowImageRect() 를 쓰면 안 된다. 그 함수는 창 크기가 아니라
    "마지막에 그린 이미지가 차지한 영역"을 돌려주기 때문에, 그 값으로 다시
    크기를 정하면 프레임마다 이미지가 줄어드는 되먹임이 생긴다(실측 확인).
    """
    try:
        with open("/sys/class/graphics/fb0/virtual_size") as f:
            w, h = f.read().strip().split(",")
            return int(w), int(h)
    except Exception:
        return default


class LcdCameraView(Node):
    def __init__(self, topic):
        super().__init__("lcd_camera_view")
        self.bridge = CvBridge()
        self.frames = 0
        self.dst_w, self.dst_h = screen_size()

        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # 이미지 토픽은 초당 30장이 들어온다. 큐를 길게 잡으면 지연만 쌓이므로
        # 1로 두고 최신 프레임만 그린다.
        self.create_subscription(Image, topic, self.on_image, 1)
        self.get_logger().info(f"구독: {topic}  화면: {self.dst_w}x{self.dst_h}")

    def on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:  # 인코딩이 예상과 다를 때
            self.get_logger().warn(f"변환 실패: {exc}")
            return

        cv2.imshow(WINDOW, cover_fit(img, self.dst_w, self.dst_h))

        self.frames += 1
        if self.frames <= 3:
            # 전체화면 속성은 창에 내용이 그려지기 전에 걸면 무시될 때가 있다.
            # 실제로 창이 400x263 으로 뜬 사례가 있어, 첫 몇 프레임 동안 다시 건다.
            cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        if self.frames == 1:
            self.get_logger().info(
                f"첫 프레임: {img.shape[1]}x{img.shape[0]} → {self.dst_w}x{self.dst_h}"
            )

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):  # q 또는 ESC
            raise KeyboardInterrupt


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/camera/camera/color/image_raw"

    rclpy.init()
    node = LcdCameraView(topic)
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
