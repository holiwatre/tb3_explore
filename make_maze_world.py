#!/usr/bin/env python3
from __future__ import annotations
import argparse, textwrap
from pathlib import Path

SDF_HEADER = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{world_name}">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
"""

SDF_FOOTER = """  </world>
</sdf>
"""

def box_model(name, x, y, z, sx, sy, sz, color=(0.1,0.1,0.1,1.0), static=True):
    amb = f"{color[0]} {color[1]} {color[2]} {color[3]}"
    dif = amb
    return f"""    <model name="{name}">
      <static>{"true" if static else "false"}</static>
      <link name="link">
        <pose>{x:.6f} {y:.6f} {z:.6f} 0 0 0</pose>
        <collision name="col"><geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          <material><ambient>{amb}</ambient><diffuse>{dif}</diffuse></material>
        </visual>
      </link>
    </model>
"""

def parse_grid(path: Path):
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line: 
            continue
        rows.append(line)
    if not rows:
        raise SystemExit("grid file is empty")
    w = len(rows[0])
    if any(len(r)!=w for r in rows):
        raise SystemExit("all rows in grid must have equal length")
    return rows  # top row first

def main():
    p = argparse.ArgumentParser(
        description="Generate Gazebo world: 12x12 cells (0.25m) + outer walls. "
                    "Any '#' in --grid becomes a 0.25x0.25xH cube centered in that cell.",
        formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--grid", type=Path, required=True,
                   help="text grid ('.' or '0' empty, '#' or '1' filled). First line is the TOP row.")
    p.add_argument("--out", type=Path, required=True,
                   help="output .world path")
    p.add_argument("--world-name", default="maze_3x3")
    p.add_argument("--cell", type=float, default=0.25, help="cell size (m), default 0.25")
    p.add_argument("--cells", type=int, default=12, help="grid cells per side, default 12")
    p.add_argument("--cube-height", type=float, default=0.25, help="cube height (m), default 0.25")
    p.add_argument("--cube-scale", type=float, default=1.0,
                   help="scale inside cell for x,y (e.g., 0.9 -> 10%% margin). default 1.0")
    p.add_argument("--wall-height", type=float, default=0.25, help="outer wall height (m)")
    p.add_argument("--wall_thick", type=float, default=0.05, help="outer wall thickness (m)")
    p.add_argument("--floor-color", default="0.95 0.95 0.95 1", help="RGBA")
    args = p.parse_args()

    N = args.cells
    CELL = args.cell
    ARENA = N * CELL  # 3.0m

    # build SDF
    out = []
    out.append(SDF_HEADER.format(world_name=args.world_name))

    # Floor (exact ARENA size), centered at (ARENA/2, ARENA/2)
    out.append(f"""    <model name="floor">
      <static>true</static>
      <link name="link">
        <pose>{ARENA/2:.6f} {ARENA/2:.6f} 0.005 0 0 0</pose>
        <visual name="vis">
          <geometry><box><size>{ARENA:.6f} {ARENA:.6f} 0.01</size></box></geometry>
          <material><ambient>{args.floor_color}</ambient><diffuse>{args.floor_color}</diffuse></material>
        </visual>
      </link>
    </model>
""")

    # Outer walls placed OUTSIDE the 3x3m interior so cells stay exact 0.25m
    t = args.wall_thick
    h = args.wall_height
    color_wall = (0.05,0.05,0.05,1)
    # West/East
    out.append(box_model("wall_west",  -t/2,        ARENA/2, h/2, t, ARENA+2*t, h, color_wall))
    out.append(box_model("wall_east",  ARENA+t/2,   ARENA/2, h/2, t, ARENA+2*t, h, color_wall))
    # South/North
    out.append(box_model("wall_south", ARENA/2,     -t/2,    h/2, ARENA+2*t, t, h, color_wall))
    out.append(box_model("wall_north", ARENA/2, ARENA+t/2,   h/2, ARENA+2*t, t, h, color_wall))

    # Parse grid and place cubes per '#'
    rows = parse_grid(args.grid)
    H = len(rows)
    W = len(rows[0])
    if H != N or W != N:
        raise SystemExit(f"grid must be {N}x{N}, got {W}x{H}")

    sx = CELL * max(0.0, min(1.0, args.cube_scale))
    sy = sx
    sz = args.cube_height
    zc = sz/2.0
    color_block = (0.05,0.05,0.05,1)  # same as 벽(검은색)

    # NOTE: text first line is TOP row -> convert to Gazebo Y increasing upward
    for r_top in range(N):
        r = N-1-r_top  # invert to bottom-based row index
        for c in range(N):
            ch = rows[r_top][c]
            filled = (ch in "#1")
            if not filled: 
                continue
            cx = (c + 0.5) * CELL
            cy = (r + 0.5) * CELL
            out.append(box_model(f"cell_{r}_{c}", cx, cy, zc, sx, sy, sz, color_block))

    out.append(SDF_FOOTER)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("".join(out), encoding="utf-8")
    print(f"Wrote {args.out} (cells={N}, cell={CELL} m, cubes from '{args.grid.name}')")

if __name__ == "__main__":
    main()
