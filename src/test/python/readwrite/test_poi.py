import unittest
import tempfile

from readwrite.poi import *


class TestLoadPOI(unittest.TestCase):
    """Tests readwrite.poi."""

    def test_load_poi_position(self):
        crisp_poi = load_poi_position('data/poi/crisp_poi.txt')

        self.assertEqual(len(crisp_poi), 49506)

        self.assertEqual(crisp_poi[0], (330, 249, 260))
        self.assertEqual(crisp_poi[1], (330, 249, 261))
        self.assertEqual(crisp_poi[2], (330, 249, 262))
        self.assertEqual(crisp_poi[3], (330, 248, 263))
        self.assertEqual(crisp_poi[4], (330, 247, 264))
        self.assertEqual(crisp_poi[5], (330, 246, 265))
        self.assertEqual(crisp_poi[6], (330, 246, 266))
        self.assertEqual(crisp_poi[7], (329, 245, 267))
        self.assertEqual(crisp_poi[8], (330, 246, 267))
        self.assertEqual(crisp_poi[9], (330, 245, 268))

        drone_poi = load_poi_position('data/poi/drone_poi.txt')

        self.assertEqual(len(drone_poi), 3817)

        self.assertAlmostEqual(drone_poi[0], (95.6122970581, -5, -17.7385005951))
        self.assertAlmostEqual(drone_poi[1], (99.2548980713, 0, -24.6259002686))
        self.assertAlmostEqual(drone_poi[2], (99.7789993286, -0.5586460233, -25.6168994904))

    def test_load_poi_list(self):
        with tempfile.NamedTemporaryFile() as t:
            with open(t.name, 'w') as f:
                f.write('1\n3\n5\n')

            xs = load_poi_list(t.name)
            self.assertEqual(xs, [1, 3, 5])
