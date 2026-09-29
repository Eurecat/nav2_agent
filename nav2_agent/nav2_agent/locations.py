"""Named locations loaded from YAML."""

from pathlib import Path
from typing import Dict

import yaml

from nav2_agent.models import Location


def normalize_location_name(name: str) -> str:
    return '_'.join(name.strip().lower().replace('-', ' ').split())


def load_locations(path: Path) -> Dict[str, Location]:
    """Load locations from a YAML file with a top-level `locations` map."""
    if not path.is_file():
        raise FileNotFoundError(f'Locations file not found: {path}')

    with path.open('r', encoding='utf-8') as yaml_file:
        data = yaml.safe_load(yaml_file) or {}

    entries = data.get('locations')
    if not isinstance(entries, dict) or not entries:
        raise ValueError(f'Locations file must define a non-empty `locations` map: {path}')

    locations = {}
    for name, fields in entries.items():
        key = normalize_location_name(str(name))
        locations[key] = Location(name=key, **(fields or {}))
    return locations


def format_locations_reference(locations: Dict[str, Location]) -> str:
    lines = []
    for location in locations.values():
        line = f'- {location.name}'
        if location.description:
            line += f': {location.description}'
        lines.append(line)
    return 'Known locations:\n' + '\n'.join(lines)
