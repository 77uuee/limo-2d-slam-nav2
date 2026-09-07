#!/usr/bin/env python3
"""
merge_maps.py — 두 개의 ROS 2 occupancy grid map(.pgm + .yaml)을 하나로 병합

사용 예)
  # 1) 대응점 클릭으로 정합 (권장, 겹침 구간에서 3점 이상 찍기)
  python3 merge_maps.py room.yaml hallway.yaml -o merged --pick

  # 2) 변환값을 이미 안다면 직접 지정 (단위: m, deg)
  python3 merge_maps.py room.yaml hallway.yaml -o merged --dx 4.2 --dy -1.3 --dtheta 91.5

출력: merged.pgm, merged.yaml  (map_server / AMCL 에 그대로 사용 가능)

의존성: numpy, opencv-python, pyyaml
"""

import argparse
import os
import numpy as np
import cv2
import yaml

# 라벨 우선순위: 값이 클수록 병합 시 우선 (점유 > 자유 > 미탐색)
UNK, FREE, OCC = 0, 1, 2

# map_saver_cli 기본 출력 규약: 흰색(254)=free, 검정(0)=occupied, 회색(205)=unknown.
# yaml 의 occupied_thresh/free_thresh 로 판정하면 205(=occ 0.196)가 free 로
# 새는 경계 케이스가 있어, 여기서는 픽셀값으로 직접 3분류한다. 더 안전하다.
FREE_PX, OCC_PX = 250, 50


# ----------------------------------------------------------------------
# 입출력
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

    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"]]
    if abs(origin[2]) > 1e-6:
        raise ValueError("origin yaw != 0 인 맵은 이 스크립트가 지원하지 않음 "
                         "(map_saver_cli 출력은 항상 0)")

    lab = np.full(img.shape, UNK, dtype=np.uint8)
    lab[img >= FREE_PX] = FREE
    lab[img <= OCC_PX] = OCC

    return {"lab": lab, "res": res, "ox": origin[0], "oy": origin[1],
            "h": img.shape[0], "w": img.shape[1], "img": img, "meta": meta,
            "name": os.path.basename(yaml_path)}


def save_map(lab, res, ox, oy, out_prefix):
    """라벨 배열 -> pgm + yaml"""
    img = np.full(lab.shape, 205, dtype=np.uint8)   # unknown
    img[lab == FREE] = 254
    img[lab == OCC] = 0

    pgm = out_prefix + ".pgm"
    cv2.imwrite(pgm, img)

    meta = {
        "image": os.path.basename(pgm),
        "mode": "trinary",
        "resolution": float(res),
        "origin": [float(ox), float(oy), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(out_prefix + ".yaml", "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=False, sort_keys=False)

    return pgm, out_prefix + ".yaml"


# ----------------------------------------------------------------------
# 픽셀 <-> 월드 좌표
# ----------------------------------------------------------------------
# ROS 맵 규약: 이미지의 '왼쪽 아래' 픽셀이 origin. 행(row)은 위에서 아래로 증가하므로
# y 축은 뒤집힌다. 이걸 틀리면 맵이 상하로 뒤집힌 채 붙는다 — 가장 흔한 실수.
def px_to_world(m, cols, rows):
    wx = m["ox"] + (cols + 0.5) * m["res"]
    wy = m["oy"] + (m["h"] - 1 - rows + 0.5) * m["res"]
    return wx, wy


def world_to_px(m, wx, wy):
    cols = (wx - m["ox"]) / m["res"] - 0.5
    rows = (m["h"] - 1) - ((wy - m["oy"]) / m["res"] - 0.5)
    return cols, rows


# ----------------------------------------------------------------------
# 정합 (rigid transform 추정)
# ----------------------------------------------------------------------
def estimate_rigid_2d(src_pts, dst_pts):
    """src(맵B 월드좌표) -> dst(맵A 월드좌표) 로 보내는 SE(2) 추정.
    Kabsch 알고리즘의 2D 버전. 스케일은 고정(둘 다 미터 단위이므로)."""
    P = np.asarray(src_pts, dtype=float)
    Q = np.asarray(dst_pts, dtype=float)
    if len(P) < 2:
        raise ValueError("대응점이 최소 2개 필요")

    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    Pc, Qc = P - pc, Q - qc

    num = np.sum(Pc[:, 0] * Qc[:, 1] - Pc[:, 1] * Qc[:, 0])   # cross
    den = np.sum(Pc[:, 0] * Qc[:, 0] + Pc[:, 1] * Qc[:, 1])   # dot
    theta = np.arctan2(num, den)

    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    t = qc - R @ pc

    rms = float(np.sqrt(np.mean(np.sum(((R @ P.T).T + t - Q) ** 2, axis=1))))
    return R, t, theta, rms


def pick_correspondences(map_a, map_b):
    """두 맵을 나란히 띄우고 A -> B -> A -> B ... 순서로 대응점을 클릭.
    ESC 로 종료. 겹침 구간(문 주변)의 뚜렷한 모서리를 3점 이상 찍는 게 좋다."""
    views, clicks = {}, {"a": [], "b": []}

    for key, m in (("a", map_a), ("b", map_b)):
        vis = cv2.cvtColor(m["img"], cv2.COLOR_GRAY2BGR)
        scale = min(1.0, 900.0 / max(m["w"], m["h"]))
        views[key] = {"vis": vis, "scale": scale, "map": m}

    def make_cb(key):
        def cb(event, x, y, flags, param):
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            v = views[key]
            col, row = x / v["scale"], y / v["scale"]
            wx, wy = px_to_world(v["map"], col, row)
            clicks[key].append((wx, wy))
            cv2.circle(v["vis"], (int(col), int(row)), 4, (0, 0, 255), -1)
            cv2.putText(v["vis"], str(len(clicks[key])), (int(col) + 6, int(row) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            print(f"  [{key.upper()}] #{len(clicks[key])}  world=({wx:.3f}, {wy:.3f})")
        return cb

    print("\n대응점을 A와 B에서 '같은 순서'로 찍으세요. 끝나면 아무 창에서 ESC.")
    for key, title in (("a", "MAP A (기준)"), ("b", "MAP B (붙일 맵)")):
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(title, make_cb(key))
        cv2.moveWindow(title, 50 if key == "a" else 950, 50)
        cv2.resizeWindow(title, 800, 800)
        views[key]["title"] = title

    while True:
        for key in ("a", "b"):
            v = views[key]
            disp = cv2.resize(v["vis"], None, fx=v["scale"], fy=v["scale"],
                              interpolation=cv2.INTER_NEAREST)
            cv2.imshow(v["title"], disp)
        if cv2.waitKey(20) == 27:
            break
    cv2.destroyAllWindows()

    n = min(len(clicks["a"]), len(clicks["b"]))
    if n < 2:
        raise ValueError("대응점이 부족합니다 (최소 2쌍)")
    return clicks["b"][:n], clicks["a"][:n]


# ----------------------------------------------------------------------
# 병합
# ----------------------------------------------------------------------
def merge(map_a, map_b, R, t, occ_priority=True):
    if abs(map_a["res"] - map_b["res"]) > 1e-9:
        raise ValueError(f"resolution 불일치: {map_a['res']} vs {map_b['res']}. "
                         "두 맵을 같은 해상도로 다시 저장하세요.")
    res = map_a["res"]

    # 1) 두 맵의 코너를 A 월드좌표계로 모아 전체 bbox 계산
    def corners_world(m):
        cs = np.array([[0, 0], [m["w"] - 1, 0], [0, m["h"] - 1], [m["w"] - 1, m["h"] - 1]],
                      dtype=float)
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

    # 2) 출력 픽셀 -> A월드 -> 각 원본 맵 픽셀 (역변환 샘플링, nearest)
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

    # 3) 결합 규칙
    if occ_priority:
        # 한쪽이라도 벽이라고 하면 벽으로 본다 — 주행 안전 쪽으로 보수적
        merged = np.maximum(lab_a, lab_b)
    else:
        # A를 신뢰하고, A가 unknown인 곳만 B로 채운다 — 정합 오차가 클 때 유리
        merged = lab_a.copy()
        fill = merged == UNK
        merged[fill] = lab_b[fill]

    # 겹침 구간 정합 품질 리포트
    both = (lab_a != UNK) & (lab_b != UNK)
    if both.sum() > 0:
        agree = (lab_a[both] == lab_b[both]).sum() / both.sum()
        print(f"  겹침 픽셀 {both.sum():,}개, 라벨 일치율 {agree*100:.1f}% "
              f"(90% 미만이면 정합을 다시 잡으세요)")
    else:
        print("  ⚠ 겹치는 영역이 없습니다. 두 맵에 공통 구간(문 주변)이 있는지 확인하세요.")

    return merged, out


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_a", help="기준이 될 맵의 .yaml (예: 강의실)")
    ap.add_argument("map_b", help="붙일 맵의 .yaml (예: 복도)")
    ap.add_argument("-o", "--out", default="merged", help="출력 prefix")
    ap.add_argument("--pick", action="store_true", help="클릭으로 대응점 지정")
    ap.add_argument("--dx", type=float, default=0.0)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dtheta", type=float, default=0.0, help="deg, B를 회전")
    ap.add_argument("--a-priority", action="store_true",
                    help="점유 우선 대신 A 우선(=A의 unknown만 B로 채움)")
    args = ap.parse_args()

    A, B = load_map(args.map_a), load_map(args.map_b)
    print(f"A: {A['name']}  {A['w']}x{A['h']}px  res={A['res']}  origin=({A['ox']}, {A['oy']})")
    print(f"B: {B['name']}  {B['w']}x{B['h']}px  res={B['res']}  origin=({B['ox']}, {B['oy']})")

    if args.pick:
        src, dst = pick_correspondences(A, B)
        R, t, theta, rms = estimate_rigid_2d(src, dst)
        print(f"\n추정 변환: dx={t[0]:.3f} m, dy={t[1]:.3f} m, "
              f"dtheta={np.degrees(theta):.2f} deg, RMS={rms*100:.1f} cm")
        print(f"(재사용: --dx {t[0]:.4f} --dy {t[1]:.4f} --dtheta {np.degrees(theta):.4f})")
        if rms > 0.15:
            print("  ⚠ RMS가 15cm를 넘습니다. 클릭 점이 서로 다른 지점이거나 맵 왜곡이 큽니다.")
    else:
        th = np.radians(args.dtheta)
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t = np.array([args.dx, args.dy])

    merged, out = merge(A, B, R, t, occ_priority=not args.a_priority)
    pgm, ymlp = save_map(merged, out["res"], out["ox"], out["oy"], args.out)
    print(f"\n저장 완료: {pgm}, {ymlp}")
    print("확인:  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=" + ymlp)


if __name__ == "__main__":
    main()
