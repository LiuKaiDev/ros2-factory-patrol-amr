#!/usr/bin/env python3
"""Generate lightweight OBJ text-label meshes for the AMR showcase worlds.

The meshes are intentionally simple: each label is built from tiny rectangular
stroke cells in a 5x7 ASCII bitmap font. This keeps Gazebo assets small,
texture-free, and easy to regenerate without external Python dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "src" / "robot_simulation" / "media" / "text_meshes"
MATERIAL_FILE = "label_text_black.mtl"


@dataclass(frozen=True)
class LabelSpec:
    mesh_name: str
    text: str
    width_m: float = 1.15
    max_height_m: float = 0.34


LABELS = [
    LabelSpec("label_robot_1.obj", "ROBOT 1"),
    LabelSpec("label_robot_2.obj", "ROBOT 2"),
    LabelSpec("label_dock.obj", "DOCK\nCHARGE"),
    LabelSpec("label_receiving.obj", "RECEIVING", width_m=1.35),
    LabelSpec("label_door.obj", "DOOR"),
    LabelSpec("label_elevator.obj", "ELEVATOR", width_m=1.30),
    LabelSpec("label_floor_2.obj", "FLOOR 2"),
    LabelSpec("label_storage_a.obj", "STORAGE A", width_m=1.40),
    LabelSpec("label_storage_b.obj", "STORAGE B", width_m=1.40),
    LabelSpec("label_packing.obj", "PACKING"),
    LabelSpec("label_traffic.obj", "TRAFFIC"),
    LabelSpec("label_slow_zone.obj", "SLOW\nZONE"),
    LabelSpec("label_confirm.obj", "CONFIRM"),
    LabelSpec("label_payload.obj", "PAYLOAD"),
    LabelSpec("label_charging_status.obj", "CHARGING", width_m=1.30),
    LabelSpec("label_docking_retry_status.obj", "DOCK\nRETRY"),
    LabelSpec("label_opportunity_charge_status.obj", "OPPORT\nCHARGE", width_m=1.35),
    LabelSpec("label_top_module_status.obj", "TOP\nMODULE"),
    LabelSpec("label_localization_status.obj", "LOCALIZE", width_m=1.30),
    LabelSpec("label_manual_takeover_status.obj", "MANUAL\nTAKEOVER", width_m=1.45),
    LabelSpec("label_operator_confirmation_status.obj", "OPERATOR\nCONFIRM", width_m=1.50),
    LabelSpec("label_traffic_block_status.obj", "TRAFFIC\nBLOCK", width_m=1.35),
    LabelSpec("label_traffic_deadlock_status.obj", "TRAFFIC\nDEADLOCK", width_m=1.55),
    LabelSpec("label_traffic_reopen_status.obj", "TRAFFIC\nREOPEN", width_m=1.45),
    LabelSpec("label_speed_limit_status.obj", "SPEED\nLIMIT"),
    LabelSpec("label_speed_restored_status.obj", "SPEED\nRESTORED", width_m=1.45),
    LabelSpec("label_obstacle_status.obj", "OBSTACLE", width_m=1.30),
    LabelSpec("label_obstacle_clear_status.obj", "OBSTACLE\nCLEAR", width_m=1.45),
]


FONT = {
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01111", "10000", "10000", "10111", "10001", "10001", "01111"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "J": ["00111", "00010", "00010", "00010", "00010", "10010", "01100"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "W": ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
}


def text_grid(text: str) -> list[str]:
    rows: list[str] = []
    lines = text.splitlines()
    for line_index, line in enumerate(lines):
        line_rows = ["" for _ in range(7)]
        for char_index, char in enumerate(line.upper()):
            glyph = FONT.get(char, FONT[" "])
            if char_index:
                for row in range(7):
                    line_rows[row] += "0"
            for row in range(7):
                line_rows[row] += glyph[row]
        rows.extend(line_rows)
        if line_index != len(lines) - 1:
            rows.append("0" * max(1, len(line_rows[0])))
    width = max(len(row) for row in rows)
    return [row.ljust(width, "0") for row in rows]


def add_quad(vertices: list[tuple[float, float, float]], faces: list[tuple[int, int, int]], x0: float, y0: float, x1: float, y1: float) -> None:
    base = len(vertices) + 1
    vertices.extend([(x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0), (x0, y1, 0.0)])
    faces.append((base, base + 1, base + 2))
    faces.append((base, base + 2, base + 3))


def write_obj(spec: LabelSpec) -> None:
    grid = text_grid(spec.text)
    rows = len(grid)
    cols = len(grid[0])
    cell = min(spec.width_m / cols, spec.max_height_m / rows)
    gap = cell * 0.12
    width = cols * cell
    height = rows * cell
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    for row, pixels in enumerate(grid):
        col = 0
        while col < cols:
            if pixels[col] != "1":
                col += 1
                continue
            start = col
            while col < cols and pixels[col] == "1":
                col += 1
            end = col
            left = start * cell - width / 2 + gap
            right = end * cell - width / 2 - gap
            top = height / 2 - row * cell - gap
            bottom = height / 2 - (row + 1) * cell + gap
            add_quad(vertices, faces, left, bottom, right, top)

    lines = [
        "# Generated by scripts/generate_amr_sim_text_geometry_labels.py",
        f"mtllib {MATERIAL_FILE}",
        f"o {Path(spec.mesh_name).stem}",
        "usemtl label_text_black",
    ]
    lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices)
    lines.extend(f"f {a} {b} {c}" for a, b, c in faces)
    (MESH_DIR / spec.mesh_name).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    (MESH_DIR / MATERIAL_FILE).write_text(
        "\n".join(
            [
                "newmtl label_text_black",
                "Ka 0.000 0.000 0.000",
                "Kd 0.000 0.000 0.000",
                "Ks 0.000 0.000 0.000",
                "Ke 0.000 0.000 0.000",
                "d 1.0",
                "illum 1",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    for spec in LABELS:
        write_obj(spec)


if __name__ == "__main__":
    main()
