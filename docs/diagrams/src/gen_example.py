"""Generate docs/diagrams/example.svg: one command decomposed into poses around the real G1.

Run render_g1.py first; this script embeds g1_render.png at the placement stored in g1_render.json.
"""
import base64
import json
import math
import sys
from pathlib import Path

from common import AXIS_X, AXIS_Y, BG, INK, LINE, MONO, MUTED, P100, P300, P500, P700, P900, PANEL, header, title
from scene_camera import project as P

HERE = Path(__file__).resolve().parent
out = [header()]
add = out.append

POSE_COLORS = (P900, P500, '#B25FD6')


def pts(points):
    return ' '.join(f'{x:.1f},{y:.1f}' for x, y in points)


add(title('One command, three poses',
          'The agent splits the sentence into motions, the tools turn them into ROS poses, Nav2 walks the robot through them.'))

# ---------------- Floor ----------------
GX, GY = (-2.5, 6.0), (-3.0, 4.0)
add('<defs><radialGradient id="fadeMask" cx="0.55" cy="0.55" r="0.5"><stop offset="0.5" stop-color="#fff"/>'
    '<stop offset="1" stop-color="#000"/></radialGradient>'
    '<mask id="sceneMask"><rect x="600" y="140" width="1000" height="760" fill="url(#fadeMask)"/></mask></defs>')
add('<g mask="url(#sceneMask)">')
add(f'<polygon points="{pts([P(GX[0], GY[0]), P(GX[1], GY[0]), P(GX[1], GY[1]), P(GX[0], GY[1])])}" fill="{PANEL}"/>')
for i in range(int((GX[1] - GX[0]) / 0.5) + 1):
    x = GX[0] + i * 0.5
    a, b = P(x, GY[0]), P(x, GY[1])
    major = abs(x - round(x)) < 1e-6
    add(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{P300 if major else LINE}" stroke-opacity="{0.6 if major else 0.8}" stroke-width="1"/>')
for i in range(int((GY[1] - GY[0]) / 0.5) + 1):
    y = GY[0] + i * 0.5
    a, b = P(GX[0], y), P(GX[1], y)
    major = abs(y - round(y)) < 1e-6
    add(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{P300 if major else LINE}" stroke-opacity="{0.6 if major else 0.8}" stroke-width="1"/>')
add('</g>')

for m in (1, 2, 3):
    tx, ty = P(m, -0.42)
    add(f'<text x="{tx:.1f}" y="{ty:.1f}" font-size="13" font-family="{MONO}" fill="{MUTED}" text-anchor="middle">{m} m</text>')


def disk(cx, cy, r, n=48):
    return [P(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def floor_arrow(x0, y0, theta, length, width=0.09, head=0.24, head_w=0.22):
    c, s = math.cos(theta), math.sin(theta)

    def w(u, v):
        return P(x0 + u * c - v * s, y0 + u * s + v * c)

    body = length - head
    return [w(0, -width / 2), w(body, -width / 2), w(body, -head_w / 2), w(length, 0),
            w(body, head_w / 2), w(body, width / 2), w(0, width / 2)]


# Path
a, b = P(0.7, 0), P(2.72, 0)
add(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{POSE_COLORS[0]}" stroke-width="3" stroke-dasharray="9 7" stroke-linecap="round"/>')
a, b = P(3, 0.32), P(3, 0.72)
add(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{POSE_COLORS[2]}" stroke-width="3" stroke-dasharray="9 7" stroke-linecap="round"/>')

# base_link axes gizmo
GZ = (0.0, 0.0)
for theta, color, label in ((0, AXIS_X, 'x'), (math.pi / 2, AXIS_Y, 'y')):
    add(f'<polygon points="{pts(floor_arrow(GZ[0], GZ[1], theta, 0.5, width=0.035, head=0.12, head_w=0.11))}" fill="{color}"/>')
    lx, ly = P(GZ[0] + 0.62 * math.cos(theta), GZ[1] + 0.62 * math.sin(theta))
    add(f'<text x="{lx:.1f}" y="{ly + 5:.1f}" font-size="15" font-weight="700" font-family="{MONO}" fill="{color}" text-anchor="middle">{label}</text>')

# Pose 1 and 2 share the position (3, 0)
add(f'<polygon points="{pts(disk(3, 0, 0.32))}" fill="{POSE_COLORS[0]}" fill-opacity="0.10" stroke="{POSE_COLORS[0]}" stroke-width="1.5"/>')
add(f'<polygon points="{pts(floor_arrow(3, 0, 0, 0.75))}" fill="{POSE_COLORS[0]}"/>')
arc_t = [i * (math.pi / 2) / 30 for i in range(4, 27)]
arc = [P(3 + 0.52 * math.cos(t), 0.52 * math.sin(t)) for t in arc_t]
add(f'<polyline points="{pts(arc)}" fill="none" stroke="{POSE_COLORS[1]}" stroke-width="4.5" stroke-linecap="round"/>')
tip = arc[-1]
ang = math.atan2(arc[-1][1] - arc[-3][1], arc[-1][0] - arc[-3][0])
head = [(tip[0] + 9 * math.cos(ang + math.pi / 2), tip[1] + 9 * math.sin(ang + math.pi / 2)),
        (tip[0] + 16 * math.cos(ang), tip[1] + 16 * math.sin(ang)),
        (tip[0] + 9 * math.cos(ang - math.pi / 2), tip[1] + 9 * math.sin(ang - math.pi / 2))]
add(f'<polygon points="{pts(head)}" fill="{POSE_COLORS[1]}"/>')

# Pose 3 at (3, 1) facing +y
add(f'<polygon points="{pts(disk(3, 1, 0.32))}" fill="{POSE_COLORS[2]}" fill-opacity="0.10" stroke="{POSE_COLORS[2]}" stroke-width="1.5"/>')
add(f'<polygon points="{pts(floor_arrow(3, 1, math.pi / 2, 0.75))}" fill="{POSE_COLORS[2]}"/>')

# ---------------- The real G1 (mesh render) ----------------
fx, fy = P(0, 0)
add(f'<ellipse cx="{fx + 4:.1f}" cy="{fy:.1f}" rx="46" ry="9" fill="{P900}" opacity="0.14"/>')
placement = json.loads((HERE / 'g1_render.json').read_text())
png = base64.b64encode((HERE / 'g1_render.png').read_bytes()).decode('ascii')
add(f'<image x="{placement["x"]}" y="{placement["y"]}" width="{placement["width"]}" height="{placement["height"]}" '
    f'href="data:image/png;base64,{png}"/>')


# ---------------- Callouts ----------------
def callout(anchor, dx, dy, color, num, text):
    ax, ay = anchor
    w = 40 + len(text) * 8.3
    bx, by = min(ax + dx, 1570 - w / 2), ay + dy
    add(f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{bx:.1f}" y2="{by:.1f}" stroke="{color}" stroke-width="1.2"/>')
    add(f'<circle cx="{ax:.1f}" cy="{ay:.1f}" r="3.5" fill="{color}"/>')
    add(f'<g filter="url(#shadow)"><rect x="{bx - w / 2:.1f}" y="{by - 36:.1f}" width="{w:.1f}" height="36" rx="18" fill="{BG}" stroke="{color}" stroke-width="1.4"/></g>')
    add(f'<circle cx="{bx - w / 2 + 19:.1f}" cy="{by - 18:.1f}" r="11" fill="{color}"/>')
    add(f'<text x="{bx - w / 2 + 19:.1f}" y="{by - 13.5:.1f}" font-size="13" font-weight="700" fill="{BG}" text-anchor="middle">{num}</text>')
    add(f'<text x="{bx - w / 2 + 38:.1f}" y="{by - 13:.1f}" font-size="14" font-family="{MONO}" fill="{INK}">{text}</text>')


callout(P(3.55, 0, 0), -60, 130, POSE_COLORS[0], '1', 'x 3.0  y 0.0  yaw 0°')
callout(P(3.3, 0.42, 0), 110, -110, POSE_COLORS[1], '2', 'x 3.0  y 0.0  yaw 90°')
callout(P(3, 1.55, 0), -60, -60, POSE_COLORS[2], '3', 'x 3.0  y 1.0  yaw 90°')

# ---------------- Left panel ----------------
add(f'<rect x="56" y="168" width="492" height="150" rx="14" fill="{P100}"/>')
add(f'<text x="84" y="204" font-size="12.5" font-weight="700" letter-spacing="2" fill="{P700}">COMMAND</text>')
add(f'<text x="84" y="244" font-size="23" font-weight="600" fill="{INK}">“<tspan fill="{POSE_COLORS[0]}">Move 3 meters forward</tspan>,</text>')
add(f'<text x="84" y="274" font-size="23" font-weight="600" fill="{INK}"><tspan fill="{POSE_COLORS[1]}">turn left 90°</tspan> and <tspan fill="{POSE_COLORS[2]}">advance</tspan></text>')
add(f'<text x="84" y="304" font-size="23" font-weight="600" fill="{INK}"><tspan fill="{POSE_COLORS[2]}">1 more meter</tspan>”</text>')

rows = [
    ('make_relative_translation', 'Forward · 3.0 m'),
    ('make_relative_turn', 'Left · π/2'),
    ('make_relative_translation', 'Left · 1.0 m · yaw π/2'),
]
add(f'<text x="60" y="370" font-size="12.5" font-weight="700" letter-spacing="2" fill="{MUTED}">TOOL CALLS</text>')
for i, ((tool, args), color) in enumerate(zip(rows, POSE_COLORS)):
    y = 388 + i * 72
    add(f'<rect x="56" y="{y}" width="492" height="58" rx="12" fill="{BG}" stroke="{LINE}" stroke-width="1.2"/>')
    add(f'<rect x="56" y="{y}" width="5" height="58" rx="2.5" fill="{color}"/>')
    add(f'<circle cx="92" cy="{y + 29}" r="13" fill="{color}"/>')
    add(f'<text x="92" y="{y + 34}" font-size="14" font-weight="700" fill="{BG}" text-anchor="middle">{i + 1}</text>')
    add(f'<text x="118" y="{y + 25}" font-size="15" font-family="{MONO}" fill="{INK}">{tool}</text>')
    add(f'<text x="118" y="{y + 45}" font-size="13.5" fill="{MUTED}">{args}</text>')

y = 388 + 3 * 72 + 14
add(f'<rect x="56" y="{y}" width="492" height="88" rx="14" fill="{P900}"/>')
add(f'<text x="84" y="{y + 32}" font-size="12.5" font-weight="700" letter-spacing="2" fill="{P300}">NAV2 GOAL</text>')
add(f'<text x="84" y="{y + 64}" font-size="21" font-weight="700" fill="{BG}">NavigateThroughPoses</text>')
add(f'<text x="330" y="{y + 64}" font-size="15" fill="{P300}">3 poses + generated BT</text>')

add('</svg>')
with open(sys.argv[1], 'w', encoding='utf-8') as f:
    f.write('\n'.join(out) + '\n')
