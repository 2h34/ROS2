#!/usr/bin/env python3
"""M2-B1 offline segmentation comparison. No ROS, TF, or truth input.

Usage: python3 m2_segment_compare.py /path/to/lidar_m2a/run02
"""
import csv
import json
import math
import sys
from pathlib import Path

# Task-defined search region (laser frame); diagnostic, not a verified detector.
ANGLE_LO_DEG, ANGLE_HI_DEG = -20.0, 38.0
RANGE_LO, RANGE_HI = 2.3, 10.5
# Parameter sensitivity experiments: these are NOT accepted production thresholds.
METHODS = (("old_20mm", None), ("adaptive_k2", 2.0), ("adaptive_k3", 3.0))
DELTA_M = 0.005
INSPECT_SCANS = {0, 29, 43, 51, 97}


def select_points(row, meta):
    """Preserve original scan index; never use Supervisor truth to select points."""
    kept = []
    count = int(meta["horizontal_resolution"])
    for i in range(count):
        r = float(row[f"r{i}"])
        if not math.isfinite(r) or not (meta["range_min"] <= r <= meta["range_max"]):
            continue
        if not (RANGE_LO <= r <= RANGE_HI):
            continue
        theta = meta["angle_min"] + i * meta["angle_increment"]
        if not (ANGLE_LO_DEG <= math.degrees(theta) <= ANGLE_HI_DEG):
            continue
        kept.append((i, r, r * math.cos(theta), r * math.sin(theta)))
    return kept


def split_segments(points, angle_increment, k):
    if not points:
        return []
    segments = [[points[0]]]
    for prev, curr in zip(points, points[1:]):
        dist = math.hypot(curr[2] - prev[2], curr[3] - prev[3])
        if k is None:  # Exact original segmentation rule (no index test).
            new_segment = dist > 0.020
        else:
            nominal = 2.0 * math.sqrt(prev[1] * curr[1]) * math.sin(abs(angle_increment) / 2.0)
            threshold = k * nominal + DELTA_M
            new_segment = (curr[0] - prev[0] != 1) or dist > threshold
        if new_segment:
            segments.append([])
        segments[-1].append(curr)
    return segments


def describe(seg, scan_id, method, index):
    first, last = seg[0], seg[-1]
    cx = sum(p[2] for p in seg) / len(seg)
    cy = sum(p[3] for p in seg) / len(seg)
    gaps = [math.hypot(b[2] - a[2], b[3] - a[3]) for a, b in zip(seg, seg[1:])]
    return {
        "scan_id": scan_id, "method": method, "segment_id": index,
        "point_count": len(seg), "start_index": first[0], "end_index": last[0],
        "chord_m": round(math.hypot(last[2] - first[2], last[3] - first[3]), 4),
        "mean_range_m": round(sum(p[1] for p in seg) / len(seg), 4),
        "mean_angle_deg": round(math.degrees(math.atan2(cy, cx)), 2),
        "max_adjacent_gap_m": round(max(gaps, default=0.0), 4),
    }


def save_csv(path, rows):
    if not rows:
        raise RuntimeError(f"No data to write: {path}")
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python3 m2_segment_compare.py /path/to/lidar_m2a/run02")
    directory = Path(sys.argv[1])
    with (directory / "metadata.json").open() as f:
        meta = json.load(f)

    summaries, details = [], []
    with (directory / "scans.csv").open(newline="") as f:
        for row in csv.DictReader(f):
            scan_id = int(row["scan_id"])
            points = select_points(row, meta)
            for method, k in METHODS:
                segments = split_segments(points, float(meta["angle_increment"]), k)
                eligible = [s for s in segments if len(s) >= 3]
                summaries.append({
                    "scan_id": scan_id, "time_s": row["time_s"],
                    "method": method, "sector_points": len(points),
                    "segment_count": len(segments), "segments_ge3": len(eligible),
                    "largest_segment_n": max((len(s) for s in segments), default=0),
                })
                for idx, segment in enumerate(segments):
                    if len(segment) >= 3:
                        details.append(describe(segment, scan_id, method, idx))

    output_dir = Path.cwd() / "m2_analysis"
    output_dir.mkdir(exist_ok=True)
    summary_path = output_dir / "m2_segment_summary.csv"
    detail_path = output_dir / "m2_segment_details.csv"
    save_csv(summary_path, summaries)
    if details:
        save_csv(detail_path, details)

    print("\nM2 segmentation comparison (not an object detector)")
    print(f"Search: {ANGLE_LO_DEG}..{ANGLE_HI_DEG} deg, {RANGE_LO}..{RANGE_HI} m")
    print(f"Methods: old=0.020m; adaptive=(k*nominal+{DELTA_M}m) AND consecutive indices")
    for scan_id in sorted(INSPECT_SCANS):
        subset = [r for r in summaries if r["scan_id"] == scan_id]
        if not subset:
            continue
        print(f"\nScan {scan_id} (candidate points={subset[0]['sector_points']}):")
        for r in subset:
            candidates = [d for d in details if d["scan_id"] == scan_id and d["method"] == r["method"]]
            main_candidates = sorted(candidates, key=lambda x: x["point_count"], reverse=True)[:3]
            parts = [f"n={d['point_count']} i={d['start_index']}..{d['end_index']} "
                     f"r={d['mean_range_m']:.2f}m chord={d['chord_m']:.3f}m"
                     for d in main_candidates]
            print(f"  {r['method']:>11}: segments={r['segment_count']:3}, "
                  f"n>=3={r['segments_ge3']:2}, max_n={r['largest_segment_n']:2}; "
                  + ("; ".join(parts) if parts else "no segment with >=3 points"))

    print(f"\nSaved {len(summaries)} per-scan method rows -> {summary_path}")
    if details:
        print(f"Saved {len(details)} segments (n>=3) -> {detail_path}")
    else:
        print("No segments with >=3 points; no segment-details file written")
    print("NOTE: Segments are UNLABELED. Do not infer 'box detected' from a large segment.")


if __name__ == "__main__":
    main()
