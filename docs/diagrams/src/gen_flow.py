"""Generate docs/diagrams/architecture.svg: the system flow, icon-driven, minimal text."""
import sys

from common import BG, REPAIR, REPAIR_BG, INK, LINE, MONO, MUTED, P100, P300, P500, P700, P900, PANEL, header, title

OPERATOR, LLM, VERIFY, NAV2 = P300, P500, P700, P900

out = [header()]
add = out.append

add(title('nav2_agent', 'The LLM designs the plan. Deterministic code verifies it. Nav2 drives the robot.'))

# Zones
zones = [
    (270, 360, LLM, 'LLM · PROPOSES'),
    (650, 340, VERIFY, 'DETERMINISTIC · VERIFIES'),
    (1010, 550, NAV2, 'NAV2 · EXECUTES'),
]
for x, w, color, label in zones:
    add(f'<rect x="{x}" y="160" width="{w}" height="690" rx="20" fill="{PANEL}"/>')
    add(f'<rect x="{x}" y="160" width="{w}" height="5" rx="2.5" fill="{color}"/>')
    add(f'<text x="{x + w / 2}" y="196" font-size="13" font-weight="700" letter-spacing="2.5" fill="{color}" '
        f'text-anchor="middle">{label}</text>')

CY = 430
R = 70


def node(cx, color, fill, icon, title, subtitle, cy=CY, r=R):
    add(f'<g filter="url(#shadow)"><circle cx="{cx}" cy="{cy}" r="{r}" fill="{BG}" stroke="{color}" stroke-width="2.5"/></g>')
    add(f'<g transform="translate({cx},{cy})" stroke="{color}" fill="none" stroke-width="3.2" stroke-linecap="round" stroke-linejoin="round">{icon}</g>')
    add(f'<text x="{cx}" y="{cy + r + 42}" font-size="22" font-weight="700" fill="{INK}" text-anchor="middle">{title}</text>')
    add(f'<text x="{cx}" y="{cy + r + 68}" font-size="15" fill="{MUTED}" text-anchor="middle">{subtitle}</text>')


ICON_SPEECH = ('<path d="M-30,-24 H30 a8,8 0 0 1 8,8 V14 a8,8 0 0 1 -8,8 H-6 L-20,34 V22 H-30 a8,8 0 0 1 -8,-8 V-16 a8,8 0 0 1 8,-8 z"/>'
               '<path d="M-22,-6 H22 M-22,6 H10"/>')
ICON_SPARKLE = ('<path d="M-6,-34 C-3,-12 2,-7 24,-4 C2,-1 -3,4 -6,26 C-9,4 -14,-1 -36,-4 C-14,-7 -9,-12 -6,-34 z"/>'
                '<path d="M24,12 C25,20 27,22 35,23 C27,24 25,26 24,34 C23,26 21,24 13,23 C21,22 23,20 24,12 z"/>')
ICON_SHIELD = ('<path d="M0,-34 L28,-24 V0 C28,18 14,30 0,36 C-14,30 -28,18 -28,0 V-24 z"/>'
               '<path d="M-12,1 L-3,10 L14,-8"/>')
ICON_TREE = ('<rect x="-10" y="-34" width="20" height="16" rx="4"/>'
             '<rect x="-36" y="14" width="20" height="16" rx="4"/><rect x="-10" y="14" width="20" height="16" rx="4"/>'
             '<rect x="16" y="14" width="20" height="16" rx="4"/>'
             '<path d="M0,-18 V-2 M-26,14 V-2 H26 V14 M0,-2 V14"/>')
ICON_ROBOT = ('<rect x="-26" y="-22" width="52" height="42" rx="12"/>'
              '<path d="M0,-22 V-34"/><circle cx="0" cy="-37" r="3.5"/>'
              '<rect x="-16" y="-8" width="32" height="12" rx="6" fill="currentColor" stroke="none" opacity="0.0"/>'
              '<circle cx="-10" cy="-2" r="3.5" fill="#3B1464"/><circle cx="10" cy="-2" r="3.5" fill="#3B1464"/>'
              '<path d="M-10,10 H10"/><path d="M-26,-2 H-34 M26,-2 H34"/>')
ICON_DOC = ('<path d="M-18,-24 H8 L20,-12 V24 H-18 z"/><path d="M8,-24 V-12 H20"/>'
            '<path d="M-10,-2 H12 M-10,8 H12 M-10,16 H4"/>')
ICON_CHAT = ('<path d="M-22,-16 H22 a6,6 0 0 1 6,6 V10 a6,6 0 0 1 -6,6 H-4 L-14,26 V16 H-22 a6,6 0 0 1 -6,-6 V-10 a6,6 0 0 1 6,-6 z"/>'
             '<path d="M-14,-2 H14"/>')

X_OP, X_AGENT, X_VAL, X_NAV, X_ROBOT = 145, 450, 820, 1170, 1420

node(X_OP, P500, None, ICON_SPEECH, 'Command', 'Natural language')
node(X_AGENT, LLM, None, ICON_SPARKLE, 'Planning agent', 'LLM + planning tools')
node(X_VAL, VERIFY, None, ICON_SHIELD, 'Validator', 'Catalog rules')
node(X_NAV, NAV2, None, ICON_TREE, 'Nav2', 'Runs the generated BT')
node(X_ROBOT, NAV2, None, ICON_ROBOT, 'Robot', '')


def arrow(x1, x2, label, color=MUTED, marker='Muted', y=CY):
    add(f'<path d="M{x1},{y} H{x2}" stroke="{color}" stroke-width="2.5" marker-end="url(#arrow{marker})"/>')
    if label:
        add(f'<text x="{(x1 + x2) / 2}" y="{y - 14}" font-size="13.5" font-weight="600" fill="{color}" text-anchor="middle">{label}</text>')


arrow(X_OP + R + 6, X_AGENT - R - 10, '')
arrow(X_AGENT + R + 6, X_VAL - R - 10, 'Plan + BT')
arrow(X_VAL + R + 6, X_NAV - R - 10, 'Goal')
arrow(X_NAV + R + 6, X_ROBOT - R - 10, 'Control', NAV2, 'P900')

# Repair loop arc above
ARC_Y, CTRL_Y = CY - R - 6, CY - 200
add(f'<path d="M{X_VAL - 30},{ARC_Y} C{X_VAL - 60},{CTRL_Y} {X_AGENT + 60},{CTRL_Y} {X_AGENT + 30},{ARC_Y}" '
    f'stroke="{REPAIR}" stroke-width="2.2" stroke-dasharray="7 6" fill="none" marker-end="url(#arrowRepair)"/>')
# Center the label on the arc apex (cubic Bezier at t=0.5).
apex_x, apex_y = (X_AGENT + X_VAL) / 2, (2 * ARC_Y + 6 * CTRL_Y) / 8
add(f'<rect x="{apex_x - 70}" y="{apex_y - 17}" width="140" height="34" rx="17" fill="{REPAIR_BG}" stroke="{REPAIR}" stroke-width="1.5"/>')
add(f'<text x="{apex_x}" y="{apex_y + 5.5}" font-size="15" font-weight="600" fill="{REPAIR}" text-anchor="middle">Repairing</text>')

# Catalog node
CAT_Y = 690
node_r = 44
add(f'<g filter="url(#shadow)"><rect x="{X_VAL - 150}" y="{CAT_Y - node_r}" width="300" height="{node_r * 2}" rx="{node_r}" fill="{BG}" stroke="{VERIFY}" stroke-opacity="0.8" stroke-width="2"/></g>')
add(f'<g transform="translate({X_VAL - 104},{CAT_Y}) scale(0.8)" stroke="{VERIFY}" fill="none" stroke-width="3.2" stroke-linecap="round" stroke-linejoin="round">{ICON_DOC}</g>')
add(f'<text x="{X_VAL - 70}" y="{CAT_Y - 4}" font-size="19" font-weight="700" fill="{INK}">BT catalog</text>')
add(f'<text x="{X_VAL - 70}" y="{CAT_Y + 20}" font-size="13.5" font-family="{MONO}" fill="{VERIFY}">bt_catalog.yaml</text>')
add(f'<path d="M{X_VAL},{CAT_Y - node_r - 2} V{CY + R + 90}" stroke="{VERIFY}" stroke-width="2.2" marker-end="url(#arrowP700)"/>')
add(f'<path d="M{X_VAL - 150},{CAT_Y} H{X_AGENT} V{CY + R + 90}" stroke="{VERIFY}" stroke-opacity="0.75" stroke-width="2" stroke-dasharray="5 5" fill="none" marker-end="url(#arrowP700)"/>')
add(f'<text x="{(X_AGENT + X_VAL - 150) / 2 + 20}" y="{CAT_Y - 12}" font-size="13.5" fill="{VERIFY}" text-anchor="middle">Toolbox in prompt</text>')

# Feedback loop along the bottom
FB_Y = 800
add(f'<path d="M{X_ROBOT},{CY + R + 80} V{FB_Y} H{X_AGENT + 126}" stroke="{LLM}" stroke-width="2.5" fill="none" marker-end="url(#arrowP500)"/>')
add(f'<text x="{(X_ROBOT + X_VAL + 150) / 2}" y="{FB_Y - 14}" font-size="15" font-weight="600" fill="{LLM}" text-anchor="middle">Real outcome</text>')
add(f'<text x="{(X_ROBOT + X_VAL + 150) / 2}" y="{FB_Y + 24}" font-size="13.5" fill="{MUTED}" text-anchor="middle">Status · Error · Recoveries · Distance</text>')
# Report pill
add(f'<g filter="url(#shadow)"><rect x="{X_AGENT - 116}" y="{FB_Y - 30}" width="236" height="60" rx="30" fill="{BG}" stroke="{LLM}" stroke-width="2"/></g>')
add(f'<g transform="translate({X_AGENT - 82},{FB_Y + 2}) scale(0.7)" stroke="{LLM}" fill="none" stroke-width="3.4" stroke-linecap="round" stroke-linejoin="round">{ICON_CHAT}</g>')
add(f'<text x="{X_AGENT - 52}" y="{FB_Y + 7}" font-size="18" font-weight="700" fill="{INK}">Outcome report</text>')
add(f'<path d="M{X_AGENT - 116},{FB_Y} H{X_OP} V{CY + R + 90}" stroke="{P500}" stroke-width="2.5" fill="none" marker-end="url(#arrowP500)"/>')
add(f'<text x="{X_OP + 12}" y="{FB_Y - 14}" font-size="13.5" font-family="{MONO}" fill="{P700}">/nav2_agent/status</text>')

add('</svg>')
with open(sys.argv[1], 'w', encoding='utf-8') as f:
    f.write('\n'.join(out) + '\n')
