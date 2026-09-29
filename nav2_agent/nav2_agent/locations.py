"""Named locations loaded from YAML."""

import os
import tempfile
from pathlib import Path
from typing import Dict, Optional

import yaml

from nav2_agent.models import Location


def normalize_location_name(name: str) -> str:
    return '_'.join(name.strip().lower().replace('-', ' ').split())


SAVED_LOCATIONS_PATH = Path('~/.ros/nav2_agent/locations.yaml').expanduser()


def load_locations(path: Path, allow_empty: bool = False) -> Dict[str, Location]:
    """Load locations from a YAML file with a top-level `locations` map."""
    if not path.is_file():
        raise FileNotFoundError(f'Locations file not found: {path}')

    with path.open('r', encoding='utf-8') as yaml_file:
        data = yaml.safe_load(yaml_file) or {}

    entries = data.get('locations') or {}
    if not isinstance(entries, dict) or (not entries and not allow_empty):
        raise ValueError(f'Locations file must define a non-empty `locations` map: {path}')

    locations = {}
    for name, fields in entries.items():
        key = normalize_location_name(str(name))
        locations[key] = Location(name=key, **(fields or {}))
    return locations


class LocationStore:
    """Configured locations plus locations saved at runtime.

    Saved locations are written to a separate file and take precedence over configured ones with the same name.
    """

    def __init__(self, configured: Dict[str, Location], saved_path: Path = SAVED_LOCATIONS_PATH) -> None:
        self._configured = dict(configured)
        self._saved_path = saved_path
        self._saved = load_locations(saved_path, allow_empty=True) if saved_path.is_file() else {}

    @property
    def saved_path(self) -> Path:
        return self._saved_path

    def all(self) -> Dict[str, Location]:
        return {**self._configured, **self._saved}

    def get(self, name: str) -> Optional[Location]:
        return self.all().get(normalize_location_name(name))

    def save(self, name: str, x: float, y: float, theta: float, description: str = '') -> Location:
        key = normalize_location_name(name)
        if not key:
            raise ValueError('Location name must not be empty.')
        location = Location(name=key, x=x, y=y, theta=theta, description=description)
        self._saved[key] = location
        self._write()
        return location

    def forget(self, name: str) -> Location:
        key = normalize_location_name(name)
        if key in self._saved:
            location = self._saved.pop(key)
            self._write()
            return location
        if key in self._configured:
            raise ValueError(f'Location {key!r} is defined in the locations file and cannot be removed at runtime.')
        raise KeyError(key)

    def _write(self) -> None:
        self._saved_path.parent.mkdir(parents=True, exist_ok=True)
        data = {'locations': {
            name: location.model_dump(exclude={'name'}, exclude_defaults=True) for name, location in self._saved.items()
        }}
        with tempfile.NamedTemporaryFile('w', dir=self._saved_path.parent, delete=False, encoding='utf-8') as handle:
            yaml.safe_dump(data, handle, sort_keys=False)
        os.replace(handle.name, self._saved_path)


def format_locations_reference(locations: Dict[str, Location]) -> str:
    if not locations:
        return 'Known locations: none.'
    lines = []
    for location in locations.values():
        line = f'- {location.name}'
        if location.description:
            line += f': {location.description}'
        lines.append(line)
    return 'Known locations:\n' + '\n'.join(lines)
