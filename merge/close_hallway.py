#!/usr/bin/env python3
"""
복도 벽 보간 스크립트 (full_v5 -> full_v6)

원리:
  복도의 각 행(y)마다 흰색(free)이 끝나는 지점을 찾고, 그 바로 바깥에
  검은색(occupied)이 있으면 "관측된 벽" = anchor 로 기록한다.
  벽이 없는 행은 위/아래 가장 가까운 anchor 사이를 선형 보간해서 채운다.

제약:
  보간한 벽 위치가 free 경계보다 안쪽(통로 쪽)이면 free 경계로 밀어낸다.
  → 실제로 관측된 통행 가능 공간은 절대 줄이지 않는다.
"""
import argparse
import numpy as np
import cv2

FREE, UNKNOWN, OCC = 254, 205, 0


def load(yaml_path):
    import yaml
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    import os
    pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    return img, meta


def scan_side(img, meta, y0, y1, xlo, xhi, side, probe_m=0.40):
    """행별로 (row_px, free_edge_px, wall_px or None) 반환"""
    h, w = img.shape
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    probe = max(1, int(round(probe_m / res)))
    xs = int(round((xlo - ox) / res))
    xe = min(w, int(round((xhi - ox) / res)))

    rows = []
    py_lo = int(round(h - 1 - (y1 - oy) / res))
    py_hi = int(round(h - 1 - (y0 - oy) / res))
    for py in range(max(0, py_lo), min(h, py_hi + 1)):
        seg = img[py, xs:xe]
        free = np.where(seg == FREE)[0]
        if len(free) == 0:
            continue
        if side == 'east':
            edge = xs + free[-1]                      # 마지막 free 칸
            probe_seg = img[py, edge + 1:edge + 1 + probe]
            hit = np.where(probe_seg == OCC)[0]
            wall = edge + 1 + hit[0] if len(hit) else None
        else:
            edge = xs + free[0]
            lo = max(0, edge - probe)
            probe_seg = img[py, lo:edge]
            hit = np.where(probe_seg == OCC)[0]
            wall = lo + hit[-1] if len(hit) else None
        rows.append([py, edge, wall])
    return rows


def interpolate(rows, side, max_span_px):
    """anchor 사이를 선형 보간. max_span_px 보다 긴 공백은 채우지 않음(개구부일 수 있음)"""
    known = [(i, r[2]) for i, r in enumerate(rows) if r[2] is not None]
    if len(known) < 2:
        return []
    filled = []
    for k in range(len(known) - 1):
        i0, x0 = known[k]
        i1, x1 = known[k + 1]
        if i1 - i0 <= 1:
            continue
        if i1 - i0 > max_span_px:
            filled.append(('SKIP', rows[i0][0], rows[i1][0], i1 - i0))
            continue
        for i in range(i0 + 1, i1):
            t = (i - i0) / (i1 - i0)
            xw = int(round(x0 + t * (x1 - x0)))
            edge = rows[i][1]
            # free 영역 침범 방지: 통로 안쪽으로 들어오면 free 경계 바로 바깥으로 밀어냄
            if side == 'east':
                xw = max(xw, edge + 1)
            else:
                xw = min(xw, edge - 1)
            filled.append((rows[i][0], xw))
    return filled


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src_yaml')
    ap.add_argument('-o', '--out', required=True, help='출력 이름 (확장자 없이)')
    ap.add_argument('--y0', type=float, default=0.0)
    ap.add_argument('--y1', type=float, default=20.0)
    ap.add_argument('--xlo', type=float, default=5.4, help='복도 검색 밴드 서쪽 끝(map x)')
    ap.add_argument('--xhi', type=float, default=10.0, help='복도 검색 밴드 동쪽 끝(map x)')
    ap.add_argument('--thickness', type=int, default=2, help='그릴 벽 두께(px)')
    ap.add_argument('--max-gap', type=float, default=3.0, help='이 길이(m)를 넘는 공백은 개구부로 보고 안 채움')
    args = ap.parse_args()

    img, meta = load(args.src_yaml)
    res = meta['resolution']
    out = img.copy()
    max_span = int(round(args.max_gap / res))

    report = {}
    for side in ('west', 'east'):
        rows = scan_side(img, meta, args.y0, args.y1, args.xlo, args.xhi, side)
        filled = interpolate(rows, side, max_span)
        skips = [f for f in filled if f[0] == 'SKIP']
        pts = [f for f in filled if f[0] != 'SKIP']
        for py, xw in pts:
            for t in range(args.thickness):
                x = xw + t if side == 'east' else xw - t
                if 0 <= x < out.shape[1] and out[py, x] != FREE:
                    out[py, x] = OCC
        report[side] = (len(rows), sum(1 for r in rows if r[2] is not None), len(pts), skips)

    cv2.imwrite(args.out + '.pgm', out)
    with open(args.src_yaml) as f:
        y = f.read()
    import os
    y = y.replace(os.path.basename(meta['image']), os.path.basename(args.out) + '.pgm')
    with open(args.out + '.yaml', 'w') as f:
        f.write(y)

    ox, oy = meta['origin'][0], meta['origin'][1]
    h = img.shape[0]
    print('=== %s -> %s ===' % (args.src_yaml, args.out))
    for side, (nrow, nanchor, nfill, skips) in report.items():
        print('%-5s: 검사 행 %d / 관측 벽(anchor) %d / 보간해서 채운 행 %d'
              % (side, nrow, nanchor, nfill))
        for s in skips:
            y_a = (h - 1 - s[1]) * res + oy
            y_b = (h - 1 - s[2]) * res + oy
            print('        건너뜀(개구부 추정): y %.2f ~ %.2f (%.1f m)'
                  % (min(y_a, y_b), max(y_a, y_b), abs(y_a - y_b)))
    print('변경된 셀 수: %d' % int((out != img).sum()))


if __name__ == '__main__':
    main()
