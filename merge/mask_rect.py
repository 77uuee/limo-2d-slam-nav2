#!/usr/bin/env python3
"""
mask_rect.py — 픽셀 좌표를 직접 지정해서 맵에 벽(마스킹)을 추가한다

mask_map.py는 마우스 드래그 방식이라 정확한 좌표를 찍기 어렵다.
이미 계산해둔 좌표가 있을 때는 이 스크립트를 쓴다. 재현도 되고,
어떤 좌표를 왜 막았는지 명령어에 그대로 남는다.

사용법)
  python3 mask_rect.py merged_final.yaml -o merged_ramp \
      -r 300,181,314,190 -r 339,181,353,190

  -r x0,y0,x1,y1   막을 사각형 (원본 픽셀 좌표, 양 끝 포함)
  --erase          벽으로 칠하는 대신 자유공간으로 지움
  --restore-from   그 구역을 다른 맵(마스킹 전 원본)의 값으로 되돌림.
                   마스킹을 취소할 때 쓴다. 흰색으로 미는 것과 달리
                   실제로 관측된 벽 정보가 그대로 살아난다.
  --preview        적용 결과를 문자 그림으로 출력 (GUI 불필요)

의존성: numpy, opencv-python, pyyaml
"""

import argparse
import os
import cv2
import yaml

OCCUPIED = 0      # 검정 = 벽
FREE = 254        # 흰색 = 자유공간


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


def parse_rect(s):
    try:
        x0, y0, x1, y1 = (int(v) for v in s.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"형식이 틀림: {s!r} (x0,y0,x1,y1 이어야 함)")
    return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)


def to_map_xy(px, py, meta, H):
    """픽셀 -> map 프레임 좌표. pgm은 위가 y가 큰 쪽이라 뒤집어야 한다."""
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    return ox + px * res, oy + (H - 1 - py) * res


def preview(img, rects, meta, pad=8):
    """마스킹 구역 주변을 문자로 출력. #=벽  .=자유  ?=미지  *=이번에 칠한 곳"""
    H, W = img.shape
    res = float(meta["resolution"])
    x0 = max(0, min(r[0] for r in rects) - pad)
    x1 = min(W - 1, max(r[2] for r in rects) + pad)
    y0 = max(0, min(r[1] for r in rects) - pad)
    y1 = min(H - 1, max(r[3] for r in rects) + pad)

    def in_rects(x, y):
        return any(a <= x <= c and b <= y <= d for a, b, c, d in rects)

    print(f"\n적용 결과  (x={x0}~{x1}, y={y0}~{y1})   * = 이번에 바뀐 셀")
    for y in range(y0, y1 + 1):
        row = ""
        for x in range(x0, x1 + 1):
            v = img[y, x]
            c = "#" if v < 100 else ("." if v > 200 else "?")
            if in_rects(x, y):
                c = "*"
            row += c
        _, my = to_map_xy(x0, y, meta, H)
        print(f"y={y:3d} ({my:5.2f}m) {row}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml", help="원본 맵의 .yaml")
    ap.add_argument("-o", "--out", default=None, help="출력 prefix")
    ap.add_argument("-r", "--rect", type=parse_rect, action="append", required=True,
                    metavar="x0,y0,x1,y1", help="사각형 (여러 번 지정 가능)")
    ap.add_argument("--erase", action="store_true", help="벽 대신 자유공간으로 지움")
    ap.add_argument("--restore-from", default=None, metavar="ORIG.yaml",
                    help="그 구역을 원본 맵의 값으로 되돌림 (마스킹 취소)")
    ap.add_argument("--preview", action="store_true", help="결과를 문자로 출력")
    args = ap.parse_args()

    img, meta = load_map(args.map_yaml)
    H, W = img.shape
    res = float(meta["resolution"])
    out_prefix = args.out or (os.path.splitext(os.path.basename(args.map_yaml))[0]
                              + "_rect")

    orig = None
    if args.restore_from:
        orig, orig_meta = load_map(args.restore_from)
        if orig.shape != img.shape:
            raise SystemExit(f"크기가 다릅니다: {img.shape} vs {orig.shape}")
        if float(orig_meta["resolution"]) != res or orig_meta["origin"][:2] != meta["origin"][:2]:
            raise SystemExit("resolution/origin이 달라 픽셀이 대응하지 않습니다")
        what = "원본값으로 복원"
    else:
        fill = FREE if args.erase else OCCUPIED
        what = "자유공간으로 지움" if args.erase else "벽으로 칠함"

    print(f"맵: {W} x {H} px @ {res} m/pix  ({W*res:.1f} x {H*res:.1f} m)")

    out = img.copy()
    total = 0
    for i, (x0, y0, x1, y1) in enumerate(args.rect, 1):
        if not (0 <= x0 and x1 < W and 0 <= y0 and y1 < H):
            raise SystemExit(f"#{i} 좌표가 맵 밖입니다: ({x0},{y0})-({x1},{y1})")
        if orig is not None:
            out[y0:y1 + 1, x0:x1 + 1] = orig[y0:y1 + 1, x0:x1 + 1]
        else:
            out[y0:y1 + 1, x0:x1 + 1] = fill
        n = (x1 - x0 + 1) * (y1 - y0 + 1)
        total += n
        mx0, my0 = to_map_xy(x0, y1, meta, H)     # 좌하단
        mx1, my1 = to_map_xy(x1, y0, meta, H)     # 우상단
        print(f"  #{i} ({x0},{y0})-({x1},{y1})  "
              f"{(x1-x0+1)*res:.2f} x {(y1-y0+1)*res:.2f} m  "
              f"map ({mx0:.2f},{my0:.2f})-({mx1:.2f},{my1:.2f})  {what}")

    pgm = out_prefix + ".pgm"
    cv2.imwrite(pgm, out)

    new_meta = dict(meta)
    new_meta["image"] = os.path.basename(pgm)
    with open(out_prefix + ".yaml", "w") as f:
        yaml.safe_dump(new_meta, f, default_flow_style=False, sort_keys=False)

    cv2.imwrite(out_prefix + ".png", out)

    print(f"\n{len(args.rect)}개 영역 / {total * res * res:.2f} m² 처리")
    print(f"저장: {pgm}, {out_prefix}.yaml, {out_prefix}.png")

    if args.preview:
        preview(out, args.rect, new_meta)


if __name__ == "__main__":
    main()
