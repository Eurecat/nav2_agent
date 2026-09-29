"""Shared theme and SVG building blocks for the nav2_agent diagrams (Eurecat-style: white + purples)."""

MONO = "'JetBrains Mono', 'DejaVu Sans Mono', monospace"
SANS = "Inter, 'Helvetica Neue', 'Segoe UI', Ubuntu, Roboto, Arial, sans-serif"

# Neutrals
BG = '#FFFFFF'
PANEL = '#F7F4FB'
LINE = '#E3DAEE'
INK = '#23153A'
MUTED = '#6E6582'

# Purple family, dark to light
P900 = '#3B1464'
P700 = '#5C2A96'
P500 = '#8750C8'
P300 = '#B795E3'
P100 = '#EEE6F9'

# Functional accents, used sparingly
ERROR = '#C8416A'
ERROR_BG = '#FBEDF2'
OK = '#2E8B6A'
REPAIR = '#A21CCB'
REPAIR_BG = '#F7EAFD'
AXIS_X, AXIS_Y = '#D9534F', '#3E9D5C'


def header(width=1600, height=900):
    markers = ''.join(
        f'<marker id="arrow{name}" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" '
        f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{color}"/></marker>'
        for name, color in [('Ink', INK), ('Muted', MUTED), ('P900', P900), ('P700', P700), ('P500', P500),
                            ('P300', P300), ('Error', ERROR), ('Ok', OK), ('Repair', REPAIR)]
    )
    return f'''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 {width} {height}" width="{width}" height="{height}" font-family="{SANS}">
<defs>
  <filter id="shadow" x="-10%" y="-10%" width="120%" height="140%"><feDropShadow dx="0" dy="4" stdDeviation="8" flood-color="{P900}" flood-opacity="0.10"/></filter>
  {markers}
</defs>
<rect width="{width}" height="{height}" fill="{BG}"/>
'''


def title(text, subtitle):
    return (f'<rect x="56" y="48" width="6" height="40" rx="3" fill="{P700}"/>'
            f'<text x="78" y="80" font-size="34" font-weight="700" fill="{INK}">{text}</text>'
            f'<text x="78" y="112" font-size="17" fill="{MUTED}">{subtitle}</text>')
