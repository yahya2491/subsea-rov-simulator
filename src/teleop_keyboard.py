#!/usr/bin/env python3
"""
Keyboard teleop for dummy_rov in Gazebo.

Run while gz sim is open:
    python3 src/teleop_keyboard.py

Controls:
    W / S    — surge   forward / backward
    A / D    — sway    left / right
    Q / E    — yaw     CCW / CW
    R / F    — heave   up / down
    SPACE    — stop all thrusters
    Ctrl+C   — quit (also stops all thrusters)

Each keypress sends a thrust command that STAYS until you change it.
Press SPACE to stop.
"""

import curses
import subprocess

# ── Config ────────────────────────────────────────────────────────────────────
MODEL   = "dummy_rov"
SCALE   = 10.0  # Newtons applied per key (max 15)

# ── Thruster joints (indices 0–7) ─────────────────────────────────────────────
#   0:fl  1:fr  2:bl  3:br  |  4:fl_top  5:fr_top  6:bl_top  7:br_top
JOINTS = [
    "thruster_fl_joint",
    "thruster_fr_joint",
    "thruster_bl_joint",
    "thruster_br_joint",
    "thruster_fl_top_joint",
    "thruster_fr_top_joint",
    "thruster_bl_top_joint",
    "thruster_br_top_joint",
]

# ── Mixing table ──────────────────────────────────────────────────────────────
#              fl    fr    bl    br  |  flt  frt  blt  brt
COMMANDS = {
    ord('w'): [ 1,    1,    1,    1,     0,   0,   0,   0],   # surge  fwd
    ord('s'): [-1,   -1,   -1,   -1,     0,   0,   0,   0],   # surge  back
    ord('a'): [-1,    1,    1,   -1,     0,   0,   0,   0],   # sway   left
    ord('d'): [ 1,   -1,   -1,    1,     0,   0,   0,   0],   # sway   right
    ord('q'): [-1,    1,   -1,    1,     0,   0,   0,   0],   # yaw    CCW
    ord('e'): [ 1,   -1,    1,   -1,     0,   0,   0,   0],   # yaw    CW
    ord('r'): [ 0,    0,    0,    0,     1,   1,   1,   1],   # heave  up
    ord('f'): [ 0,    0,    0,    0,    -1,  -1,  -1,  -1],   # heave  down
    ord(' '): [ 0,    0,    0,    0,     0,   0,   0,   0],   # STOP
}

STOP = [0] * 8

# ── Gazebo publisher ──────────────────────────────────────────────────────────
def gz_pub(joint, force_n):
    subprocess.Popen(
        ["gz", "topic",
         "-t", f"/model/{MODEL}/joint/{joint}/cmd_thrust",
         "-m", "gz.msgs.Double",
         "-p", f"data: {force_n:.2f}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

def send(multipliers, scale=SCALE):
    for joint, m in zip(JOINTS, multipliers):
        gz_pub(joint, m * scale)

# ── Curses UI ─────────────────────────────────────────────────────────────────
KEY_LABELS = {
    ord('w'): "W  → surge  FORWARD",
    ord('s'): "S  → surge  BACK",
    ord('a'): "A  → sway   LEFT",
    ord('d'): "D  → sway   RIGHT",
    ord('q'): "Q  → yaw    CCW",
    ord('e'): "E  → yaw    CW",
    ord('r'): "R  → heave  UP",
    ord('f'): "F  → heave  DOWN",
    ord(' '): "SPACE → STOP",
}

def main(stdscr):
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)

    current_mux = list(STOP)
    last_action = "none"

    def draw():
        stdscr.clear()
        stdscr.addstr(0, 0, "╔══════════════════════════════════════╗")
        stdscr.addstr(1, 0, "║     dummy_rov  Keyboard  Teleop      ║")
        stdscr.addstr(2, 0, "╚══════════════════════════════════════╝")

        stdscr.addstr(4,  0, "  W / S   surge   (fwd / back)")
        stdscr.addstr(5,  0, "  A / D   sway    (left / right)")
        stdscr.addstr(6,  0, "  Q / E   yaw     (CCW / CW)")
        stdscr.addstr(7,  0, "  R / F   heave   (up / down)")
        stdscr.addstr(8,  0, "  SPACE   STOP all thrusters")
        stdscr.addstr(9,  0, "  Ctrl+C  quit")

        stdscr.addstr(11, 0, "─────────────────────────────────────────")
        stdscr.addstr(12, 0, f"  Last key : {last_action}")
        stdscr.addstr(13, 0, "─────────────────────────────────────────")

        stdscr.addstr(14, 0, "  Thruster      Newton")
        for i, (joint, m) in enumerate(zip(JOINTS, current_mux)):
            force = m * SCALE
            bar = "█" * int(abs(force) / 1.5)
            sign = "+" if force >= 0 else "-"
            stdscr.addstr(15 + i, 0, f"  {joint:<28s} {sign}{abs(force):4.1f} N  {bar}")

        stdscr.refresh()

    draw()

    try:
        while True:
            key = stdscr.getch()

            if key == 3:            # Ctrl+C
                break
            elif key in COMMANDS:
                current_mux = COMMANDS[key]
                send(current_mux)
                label = KEY_LABELS.get(key, str(key))
                last_action = label
                draw()

            curses.napms(50)        # 20 Hz polling

    finally:
        send(STOP, scale=1.0)       # safety: zero all thrusters on exit
        stdscr.addstr(24, 0, "All thrusters stopped. Bye.")
        stdscr.refresh()
        curses.napms(800)

if __name__ == "__main__":
    curses.wrapper(main)
