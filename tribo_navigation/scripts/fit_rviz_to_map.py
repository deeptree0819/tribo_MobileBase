#!/usr/bin/env python3
"""맵 전체가 화면에 들어오도록 RViz 설정을 생성한다.

리포의 navigation.rviz 는 `Scale`(픽셀/미터)과 화면 중심이 고정값이라, 맵 크기가
바뀌면 일부만 보이거나 너무 작게 보인다. 맵 yaml 에서 실제 크기를 읽어 화면에
꼭 맞는 값을 계산해 새 설정 파일로 내보낸다.

원본은 건드리지 않는다 — 기체마다 맵이 다르므로 생성물을 넘겨 쓰는 방식이다.

사용법:
  fit_rviz_to_map.py <map.yaml> [-o 출력경로] [--base 원본rviz] [--size WxH] [--margin 0.95]

  ros2 launch tribo_navigation bringup_launch.xml \
      map:=/home/tribo/my_map.yaml \
      rviz_config:=/tmp/nav_fitted.rviz
"""

import argparse
import math
import os
import re
import subprocess
import sys

import yaml

# RViz 툴바·상태바가 세로로 먹는 대략적인 픽셀. 3D 뷰 영역을 추정하는 데 쓴다.
CHROME_H = 60


def screen_size(default=(1024, 600)):
    """X 화면 해상도. lcd_camera_view.py 와 같은 이유로 xrandr 을 우선한다
    (fb0 는 프레임버퍼 콘솔 크기라 X 모드와 다를 수 있다)."""
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


def read_pgm_size(path):
    """PGM 헤더에서 가로·세로 픽셀 수를 읽는다.

    헤더는 매직(P5) 다음에 주석(#)이 섞일 수 있으므로 토큰 단위로 훑는다.
    """
    with open(path, "rb") as f:
        head = f.read(256)
    tokens = []
    for raw in head.split(b"\n"):
        line = raw.split(b"#")[0].strip()
        if line:
            tokens.extend(line.split())
        if len(tokens) >= 3:
            break
    if len(tokens) < 3 or not tokens[0].startswith(b"P"):
        raise ValueError(f"PGM 헤더를 읽지 못했다: {path}")
    return int(tokens[1]), int(tokens[2])


def image_size(path):
    if path.lower().endswith(".pgm"):
        return read_pgm_size(path)
    from PIL import Image  # PGM 이 아닐 때만 필요

    with Image.open(path) as im:
        return im.size


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml")
    ap.add_argument("-o", "--out", default="/tmp/nav_fitted.rviz")
    ap.add_argument("--base", default=None, help="원본 rviz 설정 (기본: 패키지의 navigation.rviz)")
    ap.add_argument("--size", default=None, help="화면 크기 WxH (기본: xrandr 자동 감지)")
    ap.add_argument("--margin", type=float, default=0.95, help="여유 비율 (기본 0.95)")
    args = ap.parse_args()

    base = args.base
    if base is None:
        here = os.path.dirname(os.path.abspath(__file__))
        # 소스 트리에서 실행하는 경우와 install 된 경우를 모두 커버한다.
        for cand in (
            os.path.join(here, "..", "rviz", "navigation.rviz"),
            os.path.join(here, "..", "share", "tribo_navigation", "rviz", "navigation.rviz"),
        ):
            if os.path.exists(cand):
                base = os.path.normpath(cand)
                break
    if not base or not os.path.exists(base):
        sys.exit(f"원본 rviz 설정을 찾지 못했다: {base}")

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)

    res = float(meta["resolution"])
    origin = meta["origin"]
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(args.map_yaml)), img_path)

    px_w, px_h = image_size(img_path)
    extent_w, extent_h = px_w * res, px_h * res

    # 맵 중심. origin 은 맵 이미지의 좌하단이 월드 좌표계 어디인지를 가리킨다.
    cx = float(origin[0]) + extent_w / 2.0
    cy = float(origin[1]) + extent_h / 2.0

    if args.size:
        sw, sh = (int(v) for v in args.size.lower().split("x"))
    else:
        sw, sh = screen_size()

    # 도크를 접어 3D 뷰가 화면을 최대한 쓰게 한다. 1024x600 LCD 에서 Displays 패널이
    # 가로의 상당 부분을 먹기 때문에, 접지 않으면 맵이 실제보다 훨씬 작게 보인다.
    view_w, view_h = sw, max(1, sh - CHROME_H)

    with open(base) as f:
        cfg = yaml.safe_load(f)

    view = cfg["Visualization Manager"]["Views"]["Current"]

    # 뷰가 회전돼 있으면 화면에서 차지하는 가로·세로가 바뀐다.
    # navigation.rviz 는 Angle 이 -1.57(-90°)이라 맵의 가로·세로가 뒤집혀 보인다.
    # 이걸 무시하고 계산하면 긴 변이 화면 밖으로 잘린다(실측 확인).
    angle = float(view.get("Angle", 0.0) or 0.0)
    ca, sa = abs(math.cos(angle)), abs(math.sin(angle))
    screen_w_m = extent_w * ca + extent_h * sa
    screen_h_m = extent_w * sa + extent_h * ca

    # TopDownOrtho 의 Scale 은 "미터당 픽셀"이다. 가로·세로 중 빡빡한 쪽에 맞춘다.
    scale = min(view_w / screen_w_m, view_h / screen_h_m) * args.margin
    view["Scale"] = round(scale, 3)
    view["X"] = round(cx, 4)
    view["Y"] = round(cy, 4)

    win = cfg.setdefault("Window Geometry", {})
    win["Width"] = sw
    win["Height"] = sh
    win["Hide Left Dock"] = True
    win["Hide Right Dock"] = True

    with open(args.out, "w") as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    print(f"맵    : {px_w}x{px_h} px x {res} m = {extent_w:.2f} x {extent_h:.2f} m")
    print(f"중심  : ({cx:.3f}, {cy:.3f})")
    print(f"화면  : {sw}x{sh}  (뷰 영역 {view_w}x{view_h})")
    print(f"회전  : {math.degrees(angle):.1f}° → 화면상 {screen_w_m:.2f} x {screen_h_m:.2f} m")
    print(f"Scale : {scale:.2f} px/m  (원본 {yaml.safe_load(open(base))['Visualization Manager']['Views']['Current']['Scale']:.2f})")
    print(f"생성  : {args.out}")


if __name__ == "__main__":
    main()
