#!/usr/bin/env python3
"""
merge_maps2.py — 두 개의 ROS 2 occupancy grid map(.pgm + .yaml)을 하나로 병합
(macOS 창 겹침 문제 회피: 창을 한 번에 하나씩만 띄우는 순차 클릭 방식)

사용 예)
  python3 merge_maps2.py room_v2.yaml hallway.yaml -o merged --pick

  # 변환값을 이미 안다면 (--pick 결과를 재사용)
  python3 merge_maps2.py room_v2.yaml hallway.yaml -o merged \
      --dx 4.2 --dy -1.3 --dtheta 91.5

의존성: numpy, opencv-python, pyyaml
"""

import argparse
import os
import numpy as np
import cv2
import yaml

UNK, FREE, OCC = 0, 1, 2
FREE_PX, OCC_PX = 250, 50


# ----------------------------------------------------------------------
def load_map(yaml_path):
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    base = os.path.dirname(os.path.abspath(yaml_path))
    img_rel = meta["image"]
    img_path = img_rel if os.path.isabs(img_rel) else os.path.join(base, img_rel)

    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"이미지를 못 읽음: {img_path}")

    if int(meta.get("negate", 0)) == 1:
        img = 255 - img

    origin = [float(v) for v in meta["origin"]]
    if abs(origin[2]) > 1e-6:
        raise ValueError("origin yaw != 0 인 맵은 지원하지 않음")

    lab = np.full(img.shape, UNK, dtype=np.uint8)
    lab[img >= FREE_PX] = FREE
    lab[img <= OCC_PX] = OCC

    return {"lab": lab, "res": float(meta["resolution"]),
            "ox": origin[0], "oy": origin[1],
            "h": img.shape[0], "w": img.shape[1], "img": img,
            "name": os.path.basename(yaml_path)}


def save_map(lab, res, ox, oy, out_prefix):
    img = np.full(lab.shape, 205, dtype=np.uint8)
    img[lab == FREE] = 254
    img[lab == OCC] = 0

    pgm = out_prefix + ".pgm"
    cv2.imwrite(pgm, img)

    meta = {"image": os.path.basename(pgm), "mode": "trinary",
            "resolution": float(res), "origin": [float(ox), float(oy), 0.0],
            "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}
    with open(out_prefix + ".yaml", "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=False, sort_keys=False)

    return pgm, out_prefix + ".yaml"


# ----------------------------------------------------------------------
# ROS 맵 규약: 이미지 왼쪽 '아래'가 origin. 행은 위→아래로 증가하므로 y축이 뒤집힘.
def px_to_world(m, cols, rows):
    wx = m["ox"] + (cols + 0.5) * m["res"]
    wy = m["oy"] + (m["h"] - 1 - rows + 0.5) * m["res"]
    return wx, wy


def world_to_px(m, wx, wy):
    cols = (wx - m["ox"]) / m["res"] - 0.5
    rows = (m["h"] - 1) - ((wy - m["oy"]) / m["res"] - 0.5)
    return cols, rows


# ----------------------------------------------------------------------
def estimate_rigid_2d(src_pts, dst_pts):
    """src(B 월드) -> dst(A 월드) SE(2) 추정 (2D Kabsch)"""
    P = np.asarray(src_pts, dtype=float)
    Q = np.asarray(dst_pts, dtype=float)
    if len(P) < 2:
        raise ValueError("대응점이 최소 2개 필요")

    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    Pc, Qc = P - pc, Q - qc

    num = np.sum(Pc[:, 0] * Qc[:, 1] - Pc[:, 1] * Qc[:, 0])
    den = np.sum(Pc[:, 0] * Qc[:, 0] + Pc[:, 1] * Qc[:, 1])
    theta = np.arctan2(num, den)

    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    t = qc - R @ pc
    rms = float(np.sqrt(np.mean(np.sum(((R @ P.T).T + t - Q) ** 2, axis=1))))
    return R, t, theta, rms


def pick_one(m, title, want_n=None):
    """창을 하나만 띄워서 클릭 좌표를 모은다. ESC 또는 want_n개 채우면 종료."""
    vis = cv2.cvtColor(m["img"], cv2.COLOR_GRAY2BGR)
    scale = min(1.0, 850.0 / max(m["w"], m["h"]))
    pts = []

    def cb(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        col, row = x / scale, y / scale
        wx, wy = px_to_world(m, col, row)
        pts.append((wx, wy))
        cv2.circle(vis, (int(col), int(row)), 5, (0, 0, 255), -1)
        cv2.putText(vis, str(len(pts)), (int(col) + 8, int(row) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        print(f"    #{len(pts)}  world=({wx:.3f}, {wy:.3f})")

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, int(m["w"] * scale) + 40, int(m["h"] * scale) + 40)
    cv2.moveWindow(title, 60, 60)
    cv2.setMouseCallback(title, cb)

    while True:
        disp = cv2.resize(vis, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_NEAREST)
        cv2.imshow(title, disp)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:                      # ESC
            break
        if want_n is not None and len(pts) >= want_n:
            print(f"    {want_n}개 완료 — 창을 닫습니다.")
            cv2.waitKey(400)
            break

    cv2.destroyWindow(title)
    for _ in range(5):                   # macOS: 창이 실제로 닫히도록
        cv2.waitKey(1)
    return pts


def pick_correspondences(map_a, map_b):
    print("\n" + "=" * 62)
    print(" 1단계: MAP A (기준 맵) 에서 특징점을 클릭하세요.")
    print("        서로 멀리 떨어진 3~4개를 권장합니다.")
    print("        다 찍었으면 창에서 ESC 를 누르세요.")
    print("=" * 62)
    pts_a = pick_one(map_a, "STEP 1 - MAP A (base)")
    if len(pts_a) < 2:
        raise ValueError(f"A에서 {len(pts_a)}개만 찍혔습니다. 최소 2개 필요")

    print("\n" + "=" * 62)
    print(f" 2단계: MAP B 에서 '같은 지점'을 '같은 순서'로 {len(pts_a)}개 클릭하세요.")
    print(f"        {len(pts_a)}개를 채우면 자동으로 닫힙니다.")
    print("=" * 62)
    pts_b = pick_one(map_b, "STEP 2 - MAP B (to align)", want_n=len(pts_a))
    if len(pts_b) < len(pts_a):
        raise ValueError(f"B에서 {len(pts_b)}개만 찍혔습니다. A와 개수가 같아야 합니다")

    return pts_b[:len(pts_a)], pts_a


# ----------------------------------------------------------------------
def merge(map_a, map_b, R, t, occ_priority=True):
    if abs(map_a["res"] - map_b["res"]) > 1e-9:
        raise ValueError(f"resolution 불일치: {map_a['res']} vs {map_b['res']}")
    res = map_a["res"]

    def corners_world(m):
        cs = np.array([[0, 0], [m["w"] - 1, 0],
                       [0, m["h"] - 1], [m["w"] - 1, m["h"] - 1]], dtype=float)
        wx, wy = px_to_world(m, cs[:, 0], cs[:, 1])
        return np.stack([wx, wy], axis=1)

    ca = corners_world(map_a)
    cb = (R @ corners_world(map_b).T).T + t
    allc = np.vstack([ca, cb])

    min_x, min_y = allc[:, 0].min() - res, allc[:, 1].min() - res
    max_x, max_y = allc[:, 0].max() + res, allc[:, 1].max() + res

    W = int(np.ceil((max_x - min_x) / res))
    H = int(np.ceil((max_y - min_y) / res))
    out = {"res": res, "ox": min_x, "oy": min_y, "h": H, "w": W}
    print(f"  병합 캔버스: {W} x {H} px  ({W*res:.1f} x {H*res:.1f} m)")

    rows, cols = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
    WX, WY = px_to_world(out, cols.astype(float), rows.astype(float))

    def sample(m, wx, wy):
        c, r = world_to_px(m, wx, wy)
        c, r = np.rint(c).astype(int), np.rint(r).astype(int)
        valid = (c >= 0) & (c < m["w"]) & (r >= 0) & (r < m["h"])
        lab = np.full(wx.shape, UNK, dtype=np.uint8)
        lab[valid] = m["lab"][r[valid], c[valid]]
        return lab

    lab_a = sample(map_a, WX, WY)

    Rt = R.T
    BX = Rt[0, 0] * (WX - t[0]) + Rt[0, 1] * (WY - t[1])
    BY = Rt[1, 0] * (WX - t[0]) + Rt[1, 1] * (WY - t[1])
    lab_b = sample(map_b, BX, BY)

    if occ_priority:
        merged = np.maximum(lab_a, lab_b)
    else:
        merged = lab_a.copy()
        fill = merged == UNK
        merged[fill] = lab_b[fill]

    both = (lab_a != UNK) & (lab_b != UNK)
    if both.sum() > 0:
        agree = (lab_a[both] == lab_b[both]).sum() / both.sum()
        print(f"  겹침 픽셀 {both.sum():,}개, 라벨 일치율 {agree*100:.1f}%")
        if agree < 0.90:
            print("  ⚠ 90% 미만 — 정합을 다시 잡는 것이 좋습니다.")
    else:
        print("  ⚠ 겹치는 영역이 없습니다. 대응점이 잘못됐을 가능성이 큽니다.")

    return merged, out


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_a", help="기준 맵 .yaml")
    ap.add_argument("map_b", help="붙일 맵 .yaml")
    ap.add_argument("-o", "--out", default="merged")
    ap.add_argument("--pick", action="store_true", help="클릭으로 대응점 지정")
    ap.add_argument("--dx", type=float, default=0.0)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dtheta", type=float, default=0.0, help="deg")
    ap.add_argument("--a-priority", action="store_true",
                    help="점유 우선 대신 A 우선(A의 unknown만 B로 채움)")
    args = ap.parse_args()

    A, B = load_map(args.map_a), load_map(args.map_b)
    print(f"A: {A['name']}  {A['w']}x{A['h']}px  res={A['res']}  "
          f"origin=({A['ox']}, {A['oy']})")
    print(f"B: {B['name']}  {B['w']}x{B['h']}px  res={B['res']}  "
          f"origin=({B['ox']}, {B['oy']})")

    if args.pick:
        src, dst = pick_correspondences(A, B)
        R, t, theta, rms = estimate_rigid_2d(src, dst)
        print(f"\n추정 변환: dx={t[0]:.3f} m, dy={t[1]:.3f} m, "
              f"dtheta={np.degrees(theta):.2f} deg, RMS={rms*100:.1f} cm")
        print(f"재사용:  --dx {t[0]:.4f} --dy {t[1]:.4f} "
              f"--dtheta {np.degrees(theta):.4f}")
        if rms > 0.15:
            print("  ⚠ RMS 15cm 초과 — 클릭 지점이 서로 다른 곳일 수 있습니다.")
    else:
        th = np.radians(args.dtheta)
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t = np.array([args.dx, args.dy])

    merged, out = merge(A, B, R, t, occ_priority=not args.a_priority)
    pgm, ymlp = save_map(merged, out["res"], out["ox"], out["oy"], args.out)
    print(f"\n저장 완료: {pgm}, {ymlp}")


if __name__ == "__main__":
    main()
