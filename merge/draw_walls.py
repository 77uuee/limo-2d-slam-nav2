#!/usr/bin/env python3
"""
draw_walls.py — occupancy grid map 편집 도구

SLAM으로 만든 맵은 관측이 부족한 벽이 끊겨 있는 경우가 많다.
그대로 두면 Nav2가 그 틈으로 경로를 만들어 로봇이 벽으로 돌진한다.
이 도구로 끊긴 벽을 잇고, 진입 불가 구역을 막는다.

사용법)
  python3 draw_walls.py merged.yaml -o merged_final

  # 기존 마스킹 결과에 이어서 작업
  python3 draw_walls.py merged_masked.yaml -o merged_final

조작)
  LINE  모드 (기본) — 끊긴 벽 잇기 (검정)
      드래그: 시작점에서 끝점까지 선을 그린다
  RECT  모드        — 구역 통째로 막기 (검정)
      드래그: 사각형 영역을 채운다
  ERASE 모드        — 잘못 막힌 곳을 뚫기 (흰색 = 자유공간)
      드래그: 사각형 영역을 자유공간으로 되돌린다
      ※ 로봇이 문 앞에 서서 스캔을 시작하면 문 밖이 관측되지 않아
        출입구가 벽으로 찍히는 경우가 있다. 그럴 때 사용한다.

  Tab / m : 모드 순환 (LINE → RECT → ERASE)
  + / -   : 선 두께 조절 (기본 3 px = 15 cm)
  z       : 확대/축소 토글 (2배)  ※ 확대 중에는 w/a/d/x 로 이동
  u       : 마지막 작업 취소
  r       : 전부 취소
  s       : 저장 후 종료
  ESC     : 저장하지 않고 종료

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


FREE_PX = 254   # 자유공간(흰색). map_saver_cli 규약
OCC_PX = 0      # 점유(검정)


def apply_ops(img, ops):
    """작업 목록을 순서대로 적용해 새 이미지를 만든다."""
    out = img.copy()
    for op in ops:
        if op["type"] == "line":
            cv2.line(out, op["p0"], op["p1"], OCC_PX, op["thick"])
        elif op["type"] == "rect":
            x0, y0, x1, y1 = op["rect"]
            out[y0:y1 + 1, x0:x1 + 1] = OCC_PX
        elif op["type"] == "erase":
            x0, y0, x1, y1 = op["rect"]
            out[y0:y1 + 1, x0:x1 + 1] = FREE_PX
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml", help="편집할 맵의 .yaml")
    ap.add_argument("-o", "--out", default=None, help="출력 prefix")
    ap.add_argument("--thickness", type=int, default=3,
                    help="초기 선 두께(px). 기본 3 (= 15 cm @ 0.05 m/pix)")
    args = ap.parse_args()

    img, meta = load_map(args.map_yaml)
    H, W = img.shape
    res = float(meta["resolution"])
    out_prefix = args.out or (
        os.path.splitext(os.path.basename(args.map_yaml))[0] + "_edited")

    print(f"맵: {W} x {H} px @ {res} m/pix  ({W*res:.1f} x {H*res:.1f} m)")
    print("\n[L] 선 모드로 시작합니다. 끊긴 벽을 드래그해서 이으세요.")
    print("  Tab=모드전환  +/-=두께  z=확대  u=취소  r=전체취소  s=저장  ESC=종료\n")

    base_scale = min(1.0, 1000.0 / max(W, H))
    state = {"mode": "line", "thick": max(1, args.thickness),
             "zoom": False, "ox": 0, "oy": 0}
    ops = []
    drag = {"on": False, "x0": 0, "y0": 0, "x1": 0, "y1": 0}
    title = "MAP EDITOR"

    def view_scale():
        return base_scale * (2.0 if state["zoom"] else 1.0)

    def to_img(x, y):
        """화면 좌표 -> 원본 픽셀 좌표"""
        s = view_scale()
        cx = int(x / s) + state["ox"]
        cy = int(y / s) + state["oy"]
        return max(0, min(W - 1, cx)), max(0, min(H - 1, cy))

    def cb(event, x, y, flags, param):
        cx, cy = to_img(x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            drag.update(on=True, x0=cx, y0=cy, x1=cx, y1=cy)
        elif event == cv2.EVENT_MOUSEMOVE and drag["on"]:
            drag.update(x1=cx, y1=cy)
        elif event == cv2.EVENT_LBUTTONUP and drag["on"]:
            drag.update(on=False, x1=cx, y1=cy)
            if state["mode"] == "line":
                p0, p1 = (drag["x0"], drag["y0"]), (cx, cy)
                length = np.hypot(p1[0] - p0[0], p1[1] - p0[1])
                if length >= 2:
                    ops.append({"type": "line", "p0": p0, "p1": p1,
                                "thick": state["thick"]})
                    print(f"  #{len(ops)} 선: {p0}-{p1}  길이 {length*res:.2f} m, "
                          f"두께 {state['thick']*res*100:.0f} cm")
            else:
                x0, y0 = min(drag["x0"], cx), min(drag["y0"], cy)
                x1, y1 = max(drag["x0"], cx), max(drag["y0"], cy)
                if x1 - x0 > 2 and y1 - y0 > 2:
                    kind = "erase" if state["mode"] == "erase" else "rect"
                    ops.append({"type": kind, "rect": (x0, y0, x1, y1)})
                    word = "지우기" if kind == "erase" else "사각형"
                    print(f"  #{len(ops)} {word}: ({x0},{y0})-({x1},{y1})  "
                          f"{(x1-x0)*res:.2f} x {(y1-y0)*res:.2f} m")

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, int(W * base_scale) + 40, int(H * base_scale) + 60)
    cv2.moveWindow(title, 40, 40)
    cv2.setMouseCallback(title, cb)

    while True:
        edited = apply_ops(img, ops)
        vis = cv2.cvtColor(edited, cv2.COLOR_GRAY2BGR)

        # 이번 세션의 변경분을 색으로 구분: 추가한 벽=빨강, 뚫은 곳=파랑
        vis[(edited == OCC_PX) & (img != OCC_PX)] = (0, 0, 255)
        vis[(edited == FREE_PX) & (img != FREE_PX)] = (255, 120, 0)

        if drag["on"]:                       # 드래그 중 미리보기
            color = (255, 160, 0) if state["mode"] == "erase" else (0, 200, 0)
            if state["mode"] == "line":
                cv2.line(vis, (drag["x0"], drag["y0"]),
                         (drag["x1"], drag["y1"]), color, state["thick"])
            else:
                cv2.rectangle(vis, (drag["x0"], drag["y0"]),
                              (drag["x1"], drag["y1"]), color, 2)

        s = view_scale()
        if state["zoom"]:                    # 확대 시 보이는 영역만 잘라내기
            vw, vh = int(W * base_scale / 1.0), int(H * base_scale / 1.0)
            crop_w, crop_h = int(vw / s * base_scale), int(vh / s * base_scale)
            crop_w = max(50, min(W, int(W / 2)))
            crop_h = max(50, min(H, int(H / 2)))
            state["ox"] = max(0, min(W - crop_w, state["ox"]))
            state["oy"] = max(0, min(H - crop_h, state["oy"]))
            vis = vis[state["oy"]:state["oy"] + crop_h,
                      state["ox"]:state["ox"] + crop_w]
        else:
            state["ox"] = state["oy"] = 0

        disp = cv2.resize(vis, None, fx=s, fy=s,
                          interpolation=cv2.INTER_NEAREST)

        label = (f"[{state['mode'].upper()}]  "
                 f"thick={state['thick']}px({state['thick']*res*100:.0f}cm)  "
                 f"ops={len(ops)}  {'ZOOM' if state['zoom'] else ''}")
        cv2.rectangle(disp, (0, 0), (disp.shape[1], 26), (40, 40, 40), -1)
        cv2.putText(disp, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)

        cv2.imshow(title, disp)
        k = cv2.waitKey(20) & 0xFF

        if k in (9, ord('m')):                       # Tab or m
            order = ["line", "rect", "erase"]
            state["mode"] = order[(order.index(state["mode"]) + 1) % 3]
            desc = {"line": "선(벽 그리기)", "rect": "사각형(벽 채우기)",
                    "erase": "지우개(자유공간으로)"}
            print(f"  모드: {state['mode'].upper()} — {desc[state['mode']]}")
        elif k in (ord('+'), ord('=')):
            state["thick"] = min(25, state["thick"] + 1)
            print(f"  두께 {state['thick']}px = {state['thick']*res*100:.0f}cm")
        elif k in (ord('-'), ord('_')):
            state["thick"] = max(1, state["thick"] - 1)
            print(f"  두께 {state['thick']}px = {state['thick']*res*100:.0f}cm")
        elif k == ord('z'):
            state["zoom"] = not state["zoom"]
            state["ox"] = state["oy"] = 0
        elif k == 81 or k == ord('a'):               # left
            state["ox"] -= 40
        elif k == 83 or k == ord('d'):               # right
            state["ox"] += 40
        elif k == 82 or k == ord('w'):               # up
            state["oy"] -= 40
        elif k == 84 or k == ord('x'):               # down
            state["oy"] += 40
        elif k == ord('u') and ops:
            removed = ops.pop()
            print(f"  취소: {removed['type']} (남은 {len(ops)}개)")
        elif k == ord('r'):
            ops.clear()
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

    if not ops:
        print("작업 내역이 없어 저장하지 않습니다.")
        return

    final = apply_ops(img, ops)
    pgm = out_prefix + ".pgm"
    cv2.imwrite(pgm, final)

    new_meta = dict(meta)
    new_meta["image"] = os.path.basename(pgm)
    with open(out_prefix + ".yaml", "w") as f:
        yaml.safe_dump(new_meta, f, default_flow_style=False, sort_keys=False)

    n_line = sum(1 for o in ops if o["type"] == "line")
    n_rect = sum(1 for o in ops if o["type"] == "rect")
    n_era = sum(1 for o in ops if o["type"] == "erase")
    added = int(((final == OCC_PX) & (img != OCC_PX)).sum())
    opened = int(((final == FREE_PX) & (img != FREE_PX)).sum())
    print(f"\n선 {n_line} / 사각형 {n_rect} / 지우기 {n_era} 적용")
    print(f"  추가된 벽 {added * res * res:.2f} m², "
          f"뚫은 공간 {opened * res * res:.2f} m²")
    print(f"저장 완료: {pgm}, {out_prefix}.yaml")

    # 재현용 기록 — CLAUDE.md 등에 남겨두면 맵 재생성 시 그대로 재적용 가능
    print("\n재현용 작업 목록:")
    for i, o in enumerate(ops, 1):
        if o["type"] == "line":
            print(f"  #{i} line {o['p0']} -> {o['p1']}  thick={o['thick']}")
        else:
            print(f"  #{i} {o['type']} {o['rect']}")


if __name__ == "__main__":
    main()
