#!/usr/bin/env python3
"""End-to-end sorting test: random puck layouts, full R-G-B run, check the bins.

Needs the simulation and the geometric detector running:

  ros2 launch panda_bringup pick_and_place.launch.py
  python3 evaluate_sorting.py --layouts 3 --out results/sorting_eval.json

For each layout the pucks are placed with Gazebo's set_pose service, the picker
is run with target_colors:=RGB, and the final puck positions are read back from
Gazebo. A puck counts as sorted when it rests inside the bin for its own colour.
"""
import argparse
import json
import re
import subprocess
import time

import numpy as np

WORLD = "lab_world"
MODELS = {"R": "red_puck", "G": "green_puck", "B": "blue_puck"}
PUCK_Z = 0.38
BINS = {"R": (1.035, -0.116), "G": (0.918, -0.318), "B": (0.716, -0.435)}  # world
BIN_HALF = 0.075              # inner half-width is 0.078 m
BIN_FLOOR_Z = 0.362
BASE_WORLD = (0.6, 0.0)       # base frame -> world: x_w = 0.6 - y_b, y_w = x_b


def sh(cmd, **kw):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, **kw)


def set_pose(model, x, y, z):
    req = (f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{w: 1}}')
    sh(f"ign service -s /world/{WORLD}/set_pose --reqtype ignition.msgs.Pose "
       f"--reptype ignition.msgs.Boolean --timeout 3000 --req '{req}'")


def puck_positions():
    out = sh(f"timeout 6 ign topic -e -t /world/{WORLD}/pose/info -n 1").stdout
    found = {}
    for blk in out.split('pose {')[1:]:
        m = re.search(r'name: "(\w+)"', blk)
        if not m or m.group(1) not in MODELS.values():
            continue
        pos = re.search(r'position \{(.*?)\}', blk, re.S)
        d = dict(re.findall(r'(\w):\s*([-\d.e]+)', pos.group(1))) if pos else {}
        found[m.group(1)] = [float(d.get(k, 0.0)) for k in "xyz"]
    return found


def in_bin(color, p):
    cx, cy = BINS[color]
    return (abs(p[0] - cx) < BIN_HALF and abs(p[1] - cy) < BIN_HALF
            and p[2] > BIN_FLOOR_Z)


def base_to_world(bx, by):
    return BASE_WORLD[0] - by, BASE_WORLD[1] + bx


def random_layout(rng):
    while True:
        pts = rng.uniform([0.45, -0.30], [0.75, 0.30], size=(3, 2))
        d = [np.linalg.norm(pts[i] - pts[j])
             for i in range(3) for j in range(i + 1, 3)]
        if min(d) > 0.12:
            return {c: tuple(p) for c, p in zip("RGB", pts)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layouts", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="sorting_eval.json")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)

    results = []
    for i in range(args.layouts):
        layout = random_layout(rng)
        for c, (bx, by) in layout.items():
            wx, wy = base_to_world(bx, by)
            set_pose(MODELS[c], wx, wy, PUCK_Z)
        time.sleep(4)
        t0 = time.time()
        sh("ros2 run pymoveit2 pick_and_place.py --ros-args "
           "-p target_colors:=RGB -r /color_coordinates:=/geom_coords")
        elapsed = time.time() - t0
        final = puck_positions()
        ok = {c: in_bin(c, final[MODELS[c]]) for c in "RGB"}
        results.append({"layout_base_xy": {c: list(v) for c, v in layout.items()},
                        "final_world": final, "in_own_bin": ok,
                        "wall_seconds": round(elapsed, 1)})
        print(f"layout {i}: {ok} ({elapsed:.0f} s)", flush=True)
        json.dump(results, open(args.out, "w"), indent=1)

    n = sum(len(r["in_own_bin"]) for r in results)
    k = sum(sum(r["in_own_bin"].values()) for r in results)
    print(f"{k} of {n} pucks ended in the bin for their colour")


if __name__ == "__main__":
    main()
