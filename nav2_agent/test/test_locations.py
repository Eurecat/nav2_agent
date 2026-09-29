import tempfile
import unittest
from pathlib import Path

try:
    from nav2_agent.locations import LocationStore, format_locations_reference, load_locations, normalize_location_name
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


@unittest.skipIf(load_locations is None, 'pydantic is not installed')
class TestLocationStore(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.saved_path = Path(self.directory.name) / 'nav2_agent' / 'locations.yaml'
        self.store = LocationStore(load_locations(LOCATIONS_PATH), saved_path=self.saved_path)

    def tearDown(self):
        self.directory.cleanup()

    def test_saved_location_is_persisted(self):
        self.store.save('Corner', 1.0, 2.0, 0.5, 'Next to the door')

        reloaded = LocationStore({}, saved_path=self.saved_path)
        self.assertEqual((1.0, 2.0, 0.5), (reloaded.get('corner').x, reloaded.get('corner').y, reloaded.get('corner').theta))

    def test_saved_location_overrides_configured_one(self):
        self.store.save('shelves', 1.0, 1.0, 0.0)

        self.assertEqual(1.0, self.store.get('shelves').x)

    def test_forget_saved_location(self):
        self.store.save('corner', 1.0, 2.0, 0.0)
        self.store.forget('corner')

        self.assertIsNone(self.store.get('corner'))

    def test_configured_location_cannot_be_forgotten(self):
        with self.assertRaisesRegex(ValueError, 'cannot be removed at runtime'):
            self.store.forget('shelves')

    def test_unknown_location_cannot_be_forgotten(self):
        with self.assertRaises(KeyError):
            self.store.forget('kitchen')


if __name__ == '__main__':
    unittest.main()
