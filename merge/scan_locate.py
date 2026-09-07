#!/usr/bin/env python3
"""
scan_locate.py — 라이다 스캔 한 장으로 로봇의 map 좌표 찾기 (수동 재위치용)

AMCL이 잘못 수렴했을 때(충돌, 마스크 근처 스핀 등) rviz 클릭은 방향 오차가
크다. 이 도구는 스캔 덤프를 받아 후보 (x, y, yaw)를 격자 탐색하며 스캔
끝점이 맵의 "벽 경계선"에 가장 잘 얹히는 포즈를 찾는다.

주의: 채움이 아니라 경계선에 매칭한다. 마스크 사각형은 속이 꽉 차 있어
채움 매칭은 마스크 안 어디서나 점수가 나오는 퇴화 해를 만든다 (2026-08-11
실측). 가구가 많은 곳에선 점수 절대값이 낮아도(0.4~0.5) 정상이다.
결과의 *_match.png 에서 빨간 점(스캔)이 벽에 얹히는지 반드시 눈으로 확인.

사용법)
  # 1. 로봇에서 스캔 덤프 (BEST_EFFORT + full-length 필수):
  #    ros2 topic echo /scan --qos-reliability best_effort --once --full-length > scan.yaml
  # 2. 대략적 위치 범위를 주고 탐색:
  python3 scan_locate.py merged_ramp2.yaml scan.yaml \
      --xmin -7 --xmax -1 --ymin 0.3 --ymax 5
  # 출력된 initialpose 명령을 로봇에서 실행 (-w 1 --times 3 이 이미 포함됨)

의존성: numpy, opencv-python, pyyaml
"""

import argparse
import os
import re
import sys
import numpy as np
import cv2
import yaml

LASER_OFFSET_X = 0.103  # base_link → laser_link 전방 오프셋 (LIMO 실측)


def load_scan(path):
    txt = open(path).read()
    angle_min = float(re.search(r"angle_min: ([-\d.]+)", txt).group(1))
    angle_inc = float(re.search(r"angle_increment: ([-\d.]+)", txt).group(1))
    m = re.search(r"ranges:\n((?:- .*\n)+)", txt)
    vals = []
    for x in re.findall(r"- (.+)", m.group(1)):
        try:
            vals.append(float(x))
        except ValueError:  # .inf / .nan / '...'
            vals.append(np.inf)
    ranges = np.array(vals)
    if len(ranges) < 100:
        sys.exit("빔이 너무 적음 — --full-length 없이 덤프했는지 확인")
    angles = angle_min + np.arange(len(ranges)) * angle_inc
    ok = (ranges > 0.05) & (ranges < 8.0) & np.isfinite(ranges)
    r, a = ranges[ok], angles[ok]
    return r * np.cos(a) + LASER_OFFSET_X, r * np.sin(a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml")
    ap.add_argument("scan_yaml")
    ap.add_argument("--xmin", type=float, required=True)
    ap.add_argument("--xmax", type=float, required=True)
    ap.add_argument("--ymin", type=float, required=True)
    ap.add_argument("--ymax", type=float, required=True)
    args = ap.parse_args()

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(
        os.path.join(os.path.dirname(os.path.abspath(args.map_yaml)), meta["image"]),
        cv2.IMREAD_GRAYSCALE)
    occ = (img < 50).astype(np.uint8)
    edge = occ - cv2.erode(occ, np.ones((3, 3), np.uint8))   # 벽 경계선만
    edge_d = cv2.dilate(edge, np.ones((3, 3), np.uint8))
    H, W = occ.shape
    ox, oy = meta["origin"][0], meta["origin"][1]
    res = meta["resolution"]

    lx, ly = load_scan(args.scan_yaml)
    print(f"유효 빔 {len(lx)}개")

    def search(xs, ys, yaws):
        best = (-1, None)
        for yaw in yaws:
            c, s = np.cos(yaw), np.sin(yaw)
            rx, ry = c * lx - s * ly, s * lx + c * ly
            for x in xs:
                px = ((x + rx - ox) / res).astype(int)
                for y in ys:
                    py = (H - 1 - (y + ry - oy) / res).astype(int)
                    ok = (px >= 0) & (px < W) & (py >= 0) & (py < H)
                    if ok.sum() < len(lx) * 0.5:
                        continue
                    sc = edge_d[py[ok], px[ok]].mean()
                    if sc > best[0]:
                        best = (sc, (x, y, yaw))
        return best

    sc, (bx, by, byaw) = search(
        np.arange(args.xmin, args.xmax, 0.1),
        np.arange(args.ymin, args.ymax, 0.1),
        np.radians(np.arange(0, 360, 4)))
    print(f"코스 탐색: score={sc:.3f}  ({bx:.2f}, {by:.2f}, {np.degrees(byaw):.0f}°)")
    sc, (bx, by, byaw) = search(
        np.arange(bx - 0.2, bx + 0.21, 0.025),
        np.arange(by - 0.2, by + 0.21, 0.025),
        byaw + np.radians(np.arange(-8, 8.5, 1.0)))
    qz, qw = np.sin(byaw / 2), np.cos(byaw / 2)
    print(f"정밀 탐색: score={sc:.3f}  ({bx:.3f}, {by:.3f}, {np.degrees(byaw):.1f}°)")

    # 검증 이미지
    vis = cv2.cvtColor(255 - occ * 255, cv2.COLOR_GRAY2BGR)
    c, s = np.cos(byaw), np.sin(byaw)
    ex, ey = bx + c * lx - s * ly, by + s * lx + c * ly
    pxs = ((ex - ox) / res).astype(int)
    pys = (H - 1 - (ey - oy) / res).astype(int)
    ok = (pxs >= 0) & (pxs < W) & (pys >= 0) & (pys < H)
    vis[pys[ok], pxs[ok]] = (0, 0, 255)
    rpx, rpy = int((bx - ox) / res), int(H - 1 - (by - oy) / res)
    cv2.circle(vis, (rpx, rpy), 4, (0, 200, 0), -1)
    cv2.arrowedLine(vis, (rpx, rpy),
                    (int(rpx + 14 * c), int(rpy - 14 * s)), (0, 200, 0), 2)
    out = os.path.splitext(args.scan_yaml)[0] + "_match.png"
    crop = vis[max(0, rpy - 120):rpy + 120, max(0, rpx - 160):rpx + 160]
    cv2.imwrite(out, cv2.resize(crop, None, fx=3, fy=3,
                                interpolation=cv2.INTER_NEAREST))
    print(f"검증 이미지: {out}  ← 빨간 점이 벽에 얹히는지 확인!")

    print("\n로봇에서 실행할 initialpose 명령:")
    print(f'''ros2 topic pub -w 1 --times 3 /initialpose \\
  geometry_msgs/msg/PoseWithCovarianceStamped \\
  "{{header: {{frame_id: map}}, pose: {{pose: {{position: {{x: {bx:.3f}, y: {by:.3f}, z: 0.0}}, \\
    orientation: {{z: {qz:.4f}, w: {qw:.4f}}}}}, covariance: [0.1,0,0,0,0,0, 0,0.1,0,0,0,0, \\
    0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.05]}}}}"''')


if __name__ == "__main__":
    main()
