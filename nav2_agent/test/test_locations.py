import tempfile
import unittest
from pathlib import Path

try:
    from nav2_agent.locations import format_locations_reference, load_locations, normalize_location_name
except ImportError:  # pydantic is not installed
    load_locations = None

LOCATIONS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'locations.yaml'


@unittest.skipIf(load_locations is None, 'pydantic is not installed')
class TestLocations(unittest.TestCase):
    def test_load_locations(self):
        locations = load_locations(LOCATIONS_PATH)

        self.assertIn('loading_area', locations)
        self.assertEqual(20.3, locations['loading_area'].x)

    def test_names_are_normalized(self):
        self.assertEqual('loading_area', normalize_location_name(' Loading  Area '))
        self.assertEqual('loading_area', normalize_location_name('loading-area'))

    def test_reference_lists_names_and_descriptions(self):
        reference = format_locations_reference(load_locations(LOCATIONS_PATH))

        self.assertIn('- shelves: In front of the storage racks.', reference)

    def test_file_without_locations_is_rejected(self):
        with tempfile.NamedTemporaryFile('w', suffix='.yaml') as handle:
            handle.write('other: {}\n')
            handle.flush()
            with self.assertRaisesRegex(ValueError, 'non-empty `locations` map'):
                load_locations(Path(handle.name))


if __name__ == '__main__':
    unittest.main()
