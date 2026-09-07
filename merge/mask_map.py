#!/usr/bin/env python3
"""
mask_map.py — occupancy grid map에서 진입 불가 구역을 검게(=벽) 칠한다

Nav2는 맵에서 흰색(자유공간)이면 경로를 만든다. 실제로는 못 가는 곳
(실습 장비, 낮은 턱, 유리벽 뒤 등)은 여기서 벽으로 막아줘야 한다.

사용법)
  python3 mask_map.py merged.yaml -o merged_masked

  마우스 왼쪽 드래그 → 사각형 영역 지정 (여러 개 가능)
  u  : 마지막 사각형 취소
  r  : 전부 취소
  s  : 저장하고 종료
  ESC: 저장하지 않고 종료

의존성: numpy, opencv-python, pyyaml
"""

import argparse
import os
import numpy as np
import cv2
import yaml


def load_map(yaml_path):
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(yaml_path))
    img_rel = meta["image"]
    img_path = img_rel if os.path.isabs(img_rel) else os.path.join(base, img_rel)
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"이미지를 못 읽음: {img_path}")
    return img, meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml", help="마스킹할 맵의 .yaml")
    ap.add_argument("-o", "--out", default=None,
                    help="출력 prefix (기본: <입력>_masked)")
    args = ap.parse_args()

    img, meta = load_map(args.map_yaml)
    H, W = img.shape
    res = float(meta["resolution"])
    out_prefix = args.out or (os.path.splitext(os.path.basename(args.map_yaml))[0]
                              + "_masked")

    print(f"맵: {W} x {H} px @ {res} m/pix  ({W*res:.1f} x {H*res:.1f} m)")
    print("\n왼쪽 드래그로 막을 영역을 지정하세요.")
    print("  u=마지막 취소  r=전체 취소  s=저장 후 종료  ESC=취소하고 종료\n")

    scale = min(1.0, 1000.0 / max(W, H))
    rects = []                      # 원본 픽셀 좌표계의 (x0, y0, x1, y1)
    drag = {"on": False, "x0": 0, "y0": 0, "x1": 0, "y1": 0}
    title = "MASK - drag to block area"

    def cb(event, x, y, flags, param):
        cx, cy = int(x / scale), int(y / scale)
        cx = max(0, min(W - 1, cx))
        cy = max(0, min(H - 1, cy))
        if event == cv2.EVENT_LBUTTONDOWN:
            drag.update(on=True, x0=cx, y0=cy, x1=cx, y1=cy)
        elif event == cv2.EVENT_MOUSEMOVE and drag["on"]:
            drag.update(x1=cx, y1=cy)
        elif event == cv2.EVENT_LBUTTONUP and drag["on"]:
            drag.update(on=False, x1=cx, y1=cy)
            x0, y0 = min(drag["x0"], cx), min(drag["y0"], cy)
            x1, y1 = max(drag["x0"], cx), max(drag["y0"], cy)
            if x1 - x0 > 2 and y1 - y0 > 2:
                rects.append((x0, y0, x1, y1))
                w_m, h_m = (x1 - x0) * res, (y1 - y0) * res
                print(f"  #{len(rects)} 추가: 픽셀({x0},{y0})-({x1},{y1}) "
                      f"= {w_m:.2f} x {h_m:.2f} m")

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, int(W * scale) + 40, int(H * scale) + 40)
    cv2.moveWindow(title, 60, 60)
    cv2.setMouseCallback(title, cb)

    while True:
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for (x0, y0, x1, y1) in rects:                 # 확정된 영역: 반투명 빨강
            overlay = vis.copy()
            cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 255), -1)
            vis = cv2.addWeighted(overlay, 0.45, vis, 0.55, 0)
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 0, 255), 2)
        if drag["on"]:                                  # 드래그 중: 초록 테두리
            cv2.rectangle(vis, (drag["x0"], drag["y0"]),
                          (drag["x1"], drag["y1"]), (0, 200, 0), 2)

        disp = cv2.resize(vis, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_NEAREST)
        cv2.imshow(title, disp)
        k = cv2.waitKey(20) & 0xFF

        if k == ord('u') and rects:
            rects.pop()
            print(f"  마지막 취소 (남은 {len(rects)}개)")
        elif k == ord('r'):
            rects.clear()
            print("  전체 취소")
        elif k == ord('s'):
            break
        elif k == 27:
            print("취소하고 종료합니다.")
            cv2.destroyAllWindows()
            return

    cv2.destroyAllWindows()
    for _ in range(5):
        cv2.waitKey(1)

    if not rects:
        print("지정된 영역이 없어 저장하지 않습니다.")
        return

    # 마스킹: 해당 영역을 0(=occupied, 검정)으로
    out_img = img.copy()
    for (x0, y0, x1, y1) in rects:
        out_img[y0:y1 + 1, x0:x1 + 1] = 0

    pgm = out_prefix + ".pgm"
    cv2.imwrite(pgm, out_img)

    new_meta = dict(meta)
    new_meta["image"] = os.path.basename(pgm)          # yaml은 파일명만 (같은 폴더 기준)
    with open(out_prefix + ".yaml", "w") as f:
        yaml.safe_dump(new_meta, f, default_flow_style=False, sort_keys=False)

    blocked = sum((x1 - x0 + 1) * (y1 - y0 + 1) for x0, y0, x1, y1 in rects)
    print(f"\n{len(rects)}개 영역, 총 {blocked * res * res:.1f} m² 를 벽으로 처리")
    print(f"저장 완료: {pgm}, {out_prefix}.yaml")
    print("\n재현용 좌표 (원본 픽셀 기준):")
    for i, r in enumerate(rects, 1):
        print(f"  #{i}: {r}")


if __name__ == "__main__":
    main()
