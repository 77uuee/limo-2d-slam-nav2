#!/usr/bin/env python3
"""map_editor.py — 맵 마스킹 그리기/지우기 통합 편집 툴

용도: 매핑 때 미관측이라 "뚫려 보이는" 구간을 벽으로 메꾸거나,
잘못 그려진 마스킹을 지우는 등 맵을 직접 손보는 GUI.
draw_walls.py + edit_mask.py 기능 통합 + 확대/이동/좌표표시 추가.

사용법)
  python3 map_editor.py merged_ramp3.yaml -o merged_ramp4
  python3 map_editor.py merged_ramp3.yaml -o merged_ramp4 --overlay costmap.yaml

조작)
  마우스 왼쪽 드래그 : 현재 모드로 그리기
  마우스 오른쪽 드래그 : 화면 이동 (pan)
  휠 또는 +/-      : 확대/축소 (커서 위치 기준)
  1 : RECT  모드 — 드래그한 사각형을 벽(검정)으로 채움
  2 : LINE  모드 — 드래그한 직선을 벽(검정)으로 그림 (굵기 [ ] 로 조절)
  3 : ERASE 모드 — 드래그한 사각형을 자유공간(흰색)으로
  4 : UNKNOWN 모드 — 드래그한 사각형을 미지(회색 205)로
  [ / ] : LINE 굵기 감소/증가 (1~9 px)
  o : 오버레이 표시 켜기/끄기 (--overlay 지정 시)
  g : 격자 표시 켜기/끄기 (10px 간격, 50px 굵은 선)
  u : 마지막 작업 취소   r : 전부 취소
  s : 저장 후 종료 (적용 내역을 픽셀/맵 좌표로 출력)
  ESC : 저장하지 않고 종료

화면 하단에 커서 위치가 px(x,y) / map(x,y) 로 항상 표시됨.
의존성: numpy, opencv-python, pyyaml
"""

import argparse
import os
import re
import sys
import numpy as np
import cv2
import yaml

BLACK, WHITE, GRAY = 0, 254, 205
MODE_COLOR = {"RECT": (0, 0, 0), "LINE": (0, 0, 0),
              "ERASE": (0, 200, 255), "UNKNOWN": (160, 160, 160)}
MODE_VALUE = {"RECT": BLACK, "LINE": BLACK, "ERASE": WHITE, "UNKNOWN": GRAY}


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta["image"])
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"맵 이미지를 열 수 없음: {pgm_path}")
    return img, meta


def load_costmap_overlay(dump_path, map_img):
    """costmap 덤프에서 lethal(100)만 뽑아, 맵에 없는 실측만 남김"""
    txt = open(dump_path).read()
    w = int(re.search(r"width: (\d+)", txt).group(1))
    h = int(re.search(r"height: (\d+)", txt).group(1))
    data = np.array([int(v) for v in re.findall(r"- (-?\d+)", txt.split("data:")[1])],
                    dtype=np.int16)
    lethal = np.flipud(data.reshape(h, w) == 100)
    if lethal.shape != map_img.shape:
        sys.exit(f"costmap 크기 {lethal.shape} 가 맵 {map_img.shape} 과 다름")
    return lethal & (map_img > 200)


def main():
    ap = argparse.ArgumentParser(description="맵 마스킹 그리기/지우기 편집 툴")
    ap.add_argument("map_yaml", help="편집할 맵 yaml")
    ap.add_argument("-o", "--out", required=True, help="저장 이름 (확장자 없이)")
    ap.add_argument("--overlay", help="costmap 덤프 yaml (실측 오버레이, 빨강)")
    args = ap.parse_args()

    img, meta = load_map(args.map_yaml)
    H, W = img.shape
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]
    work = img.copy()

    overlay = load_costmap_overlay(args.overlay, img) if args.overlay else None
    if overlay is not None:
        print(f"오버레이: 맵에 없는 실측 장애물 {overlay.sum()}셀")

    # 뷰 상태: scale(배율), view_x/view_y(원본 좌표 기준 좌상단)
    st = {"scale": 2.0, "vx": 0, "vy": 0, "mode": "RECT", "thick": 2,
          "show_ov": True, "show_grid": False, "cursor": (0, 0)}
    VW, VH = 1280, 800          # 표시 창 크기(px)
    drag = {"on": False, "p0": None, "p1": None}
    pan = {"on": False, "start": None, "v0": None}
    undo_stack = []             # (y0,y1,x0,x1, 이전 패치)
    applied = []                # (mode, x0,y0,x1,y1, thick)

    def clamp_view():
        s = st["scale"]
        st["vx"] = max(0, min(st["vx"], max(0, W - VW / s)))
        st["vy"] = max(0, min(st["vy"], max(0, H - VH / s)))

    def scr2img(sx, sy):
        s = st["scale"]
        px = int(st["vx"] + sx / s)
        py = int(st["vy"] + sy / s)
        return max(0, min(px, W - 1)), max(0, min(py, H - 1))

    def apply_op(mode, p0, p1):
        x0, y0 = p0; x1, y1 = p1
        lo_x, hi_x = sorted((x0, x1)); lo_y, hi_y = sorted((y0, y1))
        # LINE은 굵기만큼 여유를 둔 bbox 저장
        t = st["thick"]
        m = t + 1
        by0, by1 = max(0, lo_y - m), min(H, hi_y + m + 1)
        bx0, bx1 = max(0, lo_x - m), min(W, hi_x + m + 1)
        undo_stack.append((by0, by1, bx0, bx1, work[by0:by1, bx0:bx1].copy()))
        val = MODE_VALUE[mode]
        if mode == "LINE":
            cv2.line(work, (x0, y0), (x1, y1), val, t)
            applied.append((mode, x0, y0, x1, y1, t))
        else:
            if hi_x - lo_x < 1 and hi_y - lo_y < 1:
                undo_stack.pop()
                return
            work[lo_y:hi_y + 1, lo_x:hi_x + 1] = val
            applied.append((mode, lo_x, lo_y, hi_x, hi_y, 0))
        mx0 = ox + lo_x * res; mx1 = ox + hi_x * res
        my1 = oy + (H - 1 - lo_y) * res; my0 = oy + (H - 1 - hi_y) * res
        print(f"{mode}: px({lo_x},{lo_y})-({hi_x},{hi_y})  "
              f"map x {mx0:.2f}~{mx1:.2f}, y {my0:.2f}~{my1:.2f}")

    def render():
        s = st["scale"]
        iw, ih = int(VW / s) + 2, int(VH / s) + 2
        x0, y0 = int(st["vx"]), int(st["vy"])
        x1, y1 = min(W, x0 + iw), min(H, y0 + ih)
        patch = work[y0:y1, x0:x1]
        vis = cv2.cvtColor(patch, cv2.COLOR_GRAY2BGR)
        if overlay is not None and st["show_ov"]:
            vis[overlay[y0:y1, x0:x1]] = (0, 0, 255)
        vis = cv2.resize(vis, (int(vis.shape[1] * s), int(vis.shape[0] * s)),
                         interpolation=cv2.INTER_NEAREST)
        canvas = np.full((VH + 30, VW, 3), 40, np.uint8)
        ch, cw = min(VH, vis.shape[0]), min(VW, vis.shape[1])
        canvas[:ch, :cw] = vis[:ch, :cw]
        if st["show_grid"] and s >= 2:
            first = (x0 // 10 + 1) * 10
            for gx in range(first, x1, 10):
                cx = int((gx - st["vx"]) * s)
                if 0 <= cx < cw:
                    col = (0, 180, 0) if gx % 50 == 0 else (60, 110, 60)
                    canvas[:ch, cx] = col
            first = (y0 // 10 + 1) * 10
            for gy in range(first, y1, 10):
                cy = int((gy - st["vy"]) * s)
                if 0 <= cy < ch:
                    col = (0, 180, 0) if gy % 50 == 0 else (60, 110, 60)
                    canvas[cy, :cw] = col
        if drag["on"] and drag["p0"] and drag["p1"]:
            c = MODE_COLOR[st["mode"]]
            q0 = (int((drag["p0"][0] - st["vx"]) * s), int((drag["p0"][1] - st["vy"]) * s))
            q1 = (int((drag["p1"][0] - st["vx"]) * s), int((drag["p1"][1] - st["vy"]) * s))
            if st["mode"] == "LINE":
                cv2.line(canvas, q0, q1, c, max(1, int(st["thick"] * s)))
            else:
                cv2.rectangle(canvas, q0, q1, c, 1)
        px, py = st["cursor"]
        mx = ox + px * res
        my = oy + (H - 1 - py) * res
        info = (f"{st['mode']}"
                + (f" t={st['thick']}" if st["mode"] == "LINE" else "")
                + f"  x{st['scale']:.1f}  px({px},{py})  map({mx:.2f},{my:.2f})"
                + f"  [1]RECT [2]LINE [3]ERASE [4]UNKNOWN [g]grid [u]undo [s]save")
        cv2.putText(canvas, info, (8, VH + 21), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        return canvas

    win = "map_editor"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    def on_mouse(ev, sx, sy, flags, _):
        if sy > VH:
            return
        px, py = scr2img(sx, sy)
        st["cursor"] = (px, py)
        if ev == cv2.EVENT_RBUTTONDOWN:
            pan.update(on=True, start=(sx, sy), v0=(st["vx"], st["vy"]))
        elif ev == cv2.EVENT_RBUTTONUP:
            pan["on"] = False
        elif ev == cv2.EVENT_MOUSEMOVE and pan["on"]:
            s = st["scale"]
            st["vx"] = pan["v0"][0] - (sx - pan["start"][0]) / s
            st["vy"] = pan["v0"][1] - (sy - pan["start"][1]) / s
            clamp_view()
        elif ev == cv2.EVENT_LBUTTONDOWN:
            drag.update(on=True, p0=(px, py), p1=(px, py))
        elif ev == cv2.EVENT_MOUSEMOVE and drag["on"]:
            drag["p1"] = (px, py)
        elif ev == cv2.EVENT_LBUTTONUP and drag["on"]:
            drag["on"] = False
            apply_op(st["mode"], drag["p0"], (px, py))
        elif ev == cv2.EVENT_MOUSEWHEEL:
            s0 = st["scale"]
            s1 = min(12.0, s0 + 0.5) if flags > 0 else max(1.0, s0 - 0.5)
            # 커서 위치 고정 확대
            st["vx"] += sx / s0 - sx / s1
            st["vy"] += sy / s0 - sy / s1
            st["scale"] = s1
            clamp_view()

    cv2.setMouseCallback(win, on_mouse)
    print("편집 시작. 조작법은 파일 상단 주석 또는 창 하단 안내 참고.")

    while True:
        cv2.imshow(win, render())
        k = cv2.waitKey(30) & 0xFF
        if k == ord("1"):
            st["mode"] = "RECT"
        elif k == ord("2"):
            st["mode"] = "LINE"
        elif k == ord("3"):
            st["mode"] = "ERASE"
        elif k == ord("4"):
            st["mode"] = "UNKNOWN"
        elif k == ord("["):
            st["thick"] = max(1, st["thick"] - 1)
        elif k == ord("]"):
            st["thick"] = min(9, st["thick"] + 1)
        elif k == ord("o"):
            st["show_ov"] = not st["show_ov"]
        elif k == ord("g"):
            st["show_grid"] = not st["show_grid"]
        elif k in (ord("+"), ord("=")):
            st["scale"] = min(12.0, st["scale"] + 0.5); clamp_view()
        elif k == ord("-"):
            st["scale"] = max(1.0, st["scale"] - 0.5); clamp_view()
        elif k == ord("u") and undo_stack:
            y0, y1, x0, x1, patch = undo_stack.pop()
            work[y0:y1, x0:x1] = patch
            applied.pop()
            print("undo")
        elif k == ord("r"):
            work[:] = img
            undo_stack.clear()
            applied.clear()
            print("전부 취소")
        elif k == ord("s"):
            out_dir = os.path.dirname(os.path.abspath(args.map_yaml))
            out_pgm = os.path.join(out_dir, args.out + ".pgm")
            out_yaml = os.path.join(out_dir, args.out + ".yaml")
            cv2.imwrite(out_pgm, work)
            cv2.imwrite(os.path.join(out_dir, args.out + ".png"), work)
            new_meta = dict(meta)
            new_meta["image"] = args.out + ".pgm"
            with open(out_yaml, "w") as f:
                yaml.dump(new_meta, f, default_flow_style=None, sort_keys=False)
            print(f"\n저장: {out_pgm}")
            print(f"저장: {out_yaml}")
            print("\n적용 내역 (기록용):")
            for m, x0, y0, x1, y1, t in applied:
                extra = f" thick={t}" if m == "LINE" else ""
                print(f"  {m} ({x0},{y0},{x1},{y1}){extra}")
            break
        elif k == 27:
            print("저장하지 않고 종료")
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
