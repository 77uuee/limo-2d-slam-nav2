#!/usr/bin/env python3
"""
edit_mask.py — 관측 오버레이 기반 마스킹 수정 도구

문제: 마스크 사각형보다 실제 실습공간(장비·의자)이 더 넓으면, planner는
"마스크 옆은 빈 공간"으로 믿고 경로를 그리로 낸다. 로봇이 가서야 실물을
만나 갇히고, 이 일이 매번 반복된다.

해결: 주행 중 global costmap에 찍힌 실제 장애물 관측을 맵 위에 빨간색으로
겹쳐 보여준다. 마스크 밖으로 삐져나온 빨간 점들이 "실물이 더 넓다"는 증거
이므로, 그 범위를 덮도록 마스크를 다시 그리면 된다.

사용법)
  # 1. (Claude Code가) 주행 후 costmap을 받아둔다:
  #    ros2 topic echo /global_costmap/costmap --once --full-length > costmap.yaml
  # 2. 오버레이와 함께 편집:
  python3 edit_mask.py merged_ramp2.yaml -o merged_ramp3 --overlay costmap_20260811.yaml

조작)
  RECT  모드 (기본) — 드래그한 사각형을 장애물(검정)로 채움
  ERASE 모드        — 드래그한 사각형을 자유공간(흰색)으로 되돌림
  Tab / m : 모드 전환 (RECT ↔ ERASE)
  o       : 오버레이 표시 켜기/끄기
  + / -   : 확대/축소 (0.5 단위)
  u       : 마지막 작업 취소
  r       : 전부 취소
  s       : 저장 후 종료 (적용한 사각형 좌표를 픽셀/미터 단위로 출력)
  ESC     : 저장하지 않고 종료

의존성: numpy, opencv-python, pyyaml
"""

import argparse
import os
import re
import sys
import numpy as np
import cv2
import yaml


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta["image"])
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"맵 이미지를 열 수 없음: {pgm_path}")
    return img, meta


def load_costmap_overlay(dump_path, map_shape):
    """`ros2 topic echo /global_costmap/costmap --once --full-length` 덤프에서
    lethal(100) 셀만 뽑아 맵 이미지 좌표계(위가 +y 반대)의 마스크로 변환."""
    txt = open(dump_path).read()
    w = int(re.search(r"width: (\d+)", txt).group(1))
    h = int(re.search(r"height: (\d+)", txt).group(1))
    data = np.array(
        [int(v) for v in re.findall(r"- (-?\d+)", txt.split("data:")[1])],
        dtype=np.int16,
    )
    grid = data.reshape(h, w)  # OccupancyGrid: row 0 = 맵 아래쪽(y 최소)
    lethal = np.flipud(grid == 100)  # PGM: row 0 = 맵 위쪽 → 상하 반전
    if lethal.shape != map_shape:
        sys.exit(f"costmap 크기 {lethal.shape} 가 맵 {map_shape} 과 다름")
    return lethal


def px_to_map(meta, img_h, px, py):
    """이미지 픽셀 → map 좌표 (미터)"""
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]
    return ox + px * res, oy + (img_h - 1 - py) * res


def main():
    ap = argparse.ArgumentParser(description="관측 오버레이 기반 마스킹 수정")
    ap.add_argument("map_yaml", help="편집할 맵의 yaml 경로")
    ap.add_argument("-o", "--out", required=True, help="저장 이름 (확장자 없이)")
    ap.add_argument("--overlay", help="costmap 덤프 yaml (실제 관측 오버레이)")
    ap.add_argument("--scale", type=float, default=2.0, help="표시 배율 (기본 2)")
    args = ap.parse_args()

    img, meta = load_map(args.map_yaml)
    H, W = img.shape
    work = img.copy()

    overlay = None
    if args.overlay:
        overlay = load_costmap_overlay(args.overlay, img.shape)
        # 맵에 이미 장애물인 곳은 제외 → "맵에 없는데 실제로 관측된 것"만 남김
        overlay = overlay & (img > 200)
        print(f"오버레이: 맵에 없는 실측 장애물 {overlay.sum()}셀 "
              f"({overlay.sum()*meta['resolution']**2:.2f} m²)")

    scale = args.scale
    mode = "RECT"
    show_overlay = True
    undo_stack = []          # (y0,y1,x0,x1, 이전 패치)
    applied = []             # (mode, x0,y0,x1,y1) 픽셀 좌표 기록
    drag = {"on": False, "p0": None, "p1": None}

    win = "edit_mask  (s=저장  u=취소  Tab=모드  o=오버레이  ESC=종료)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    def render():
        vis = cv2.cvtColor(work, cv2.COLOR_GRAY2BGR)
        if overlay is not None and show_overlay:
            vis[overlay] = (0, 0, 255)  # 실측 장애물 = 빨강
        if drag["on"] and drag["p0"] and drag["p1"]:
            color = (0, 0, 0) if mode == "RECT" else (0, 200, 255)
            cv2.rectangle(vis, drag["p0"], drag["p1"], color, 1)
        vis = cv2.resize(vis, (int(W * scale), int(H * scale)),
                         interpolation=cv2.INTER_NEAREST)
        cv2.putText(vis, f"{mode}  scale x{scale:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        return vis

    def on_mouse(ev, x, y, flags, _):
        px, py = int(x / scale), int(y / scale)
        px, py = min(px, W - 1), min(py, H - 1)
        if ev == cv2.EVENT_LBUTTONDOWN:
            drag.update(on=True, p0=(px, py), p1=(px, py))
        elif ev == cv2.EVENT_MOUSEMOVE and drag["on"]:
            drag["p1"] = (px, py)
        elif ev == cv2.EVENT_LBUTTONUP and drag["on"]:
            drag["on"] = False
            x0, x1 = sorted((drag["p0"][0], px))
            y0, y1 = sorted((drag["p0"][1], py))
            if x1 - x0 < 1 or y1 - y0 < 1:
                return
            undo_stack.append((y0, y1 + 1, x0, x1 + 1,
                               work[y0:y1 + 1, x0:x1 + 1].copy()))
            work[y0:y1 + 1, x0:x1 + 1] = 0 if mode == "RECT" else 254
            applied.append((mode, x0, y0, x1, y1))
            mx0, my1 = px_to_map(meta, H, x0, y0)   # 주의: 이미지 y0(위) = map y 최대
            mx1, my0 = px_to_map(meta, H, x1, y1)
            print(f"{mode}: px({x0},{y0})-({x1},{y1})  "
                  f"map x {mx0:.2f}~{mx1:.2f}, y {my0:.2f}~{my1:.2f}")

    cv2.setMouseCallback(win, on_mouse)

    while True:
        cv2.imshow(win, render())
        k = cv2.waitKey(30) & 0xFF
        if k in (9, ord("m")):                       # Tab / m
            mode = "ERASE" if mode == "RECT" else "RECT"
        elif k == ord("o"):
            show_overlay = not show_overlay
        elif k in (ord("+"), ord("=")):
            scale = min(scale + 0.5, 6.0)
        elif k == ord("-"):
            scale = max(scale - 0.5, 1.0)
        elif k == ord("u") and undo_stack:
            y0, y1, x0, x1, patch = undo_stack.pop()
            work[y0:y1, x0:x1] = patch
            applied.pop()
        elif k == ord("r"):
            work[:] = img
            undo_stack.clear()
            applied.clear()
        elif k == ord("s"):
            out_dir = os.path.dirname(os.path.abspath(args.map_yaml))
            out_pgm = os.path.join(out_dir, args.out + ".pgm")
            out_yaml = os.path.join(out_dir, args.out + ".yaml")
            out_png = os.path.join(out_dir, args.out + ".png")
            cv2.imwrite(out_pgm, work)
            cv2.imwrite(out_png, work)
            new_meta = dict(meta)
            new_meta["image"] = args.out + ".pgm"
            with open(out_yaml, "w") as f:
                yaml.dump(new_meta, f, default_flow_style=None, sort_keys=False)
            print(f"\n저장: {out_pgm}")
            print(f"저장: {out_yaml}")
            print("\n적용 내역 (CLAUDE.md 기록용):")
            for m, x0, y0, x1, y1 in applied:
                print(f"  {m} ({x0},{y0},{x1},{y1})")
            break
        elif k == 27:                                # ESC
            print("저장하지 않고 종료")
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
