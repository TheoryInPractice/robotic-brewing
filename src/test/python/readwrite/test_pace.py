import unittest
import networkx as nx
from io import StringIO
from readwrite.pace import *


class TestPACE(unittest.TestCase):
    """Tests pace module."""

    def test_load_pace(self):
        """Tests load_pace()."""

        G, s, t, k = load_pace('src/test/resources/instances/001_tiny.gr')
        self.assertEqual(len(G), 5)
        self.assertEqual(G.number_of_edges(), 4)
        self.assertEqual(s, 0)
        self.assertEqual(t, 0)
        self.assertEqual(k, 4)
        self.assertDictEqual(nx.get_node_attributes(G, 'color'), {
            0: [], 1: [0, 1, 2], 2: [1, 3], 3: [1], 4: [3]
        })
        self.assertEqual(nx.get_edge_attributes(G, 'weight'), {
            (0, 1): 3,
            (0, 2): 6,
            (0, 3): 2,
            (3, 4): 3,
        })

        G, s, t, k = load_pace('src/test/resources/instances/005_real_weight.gr')
        self.assertEqual(len(G), 5)
        self.assertEqual(G.number_of_edges(), 4)
        self.assertEqual(s, 0)
        self.assertEqual(t, 0)
        self.assertEqual(k, 4)
        self.assertDictEqual(nx.get_node_attributes(G, 'color'), {
            0: [], 1: [0, 1, 2], 2: [1, 3], 3: [1], 4: [3]
        })
        self.assertEqual(nx.get_edge_attributes(G, 'weight'), {
            (0, 1): 0.03,
            (0, 2): 0.06,
            (0, 3): 0.02,
            (3, 4): 0.03,
        })

    def test_write_pace(self):
        """Tests write_pace()."""

        G, s, t, k = load_pace('src/test/resources/instances/001_tiny.gr')
        fp = StringIO()
        write_pace(fp, G, s, t, k)
        self.assertEqual(fp.getvalue(), '\n'.join([
            'p ip 5 4 4 0 0 4',
            'v 0',
            'v 1 0 1 2',
            'v 2 1 3',
            'v 3 1',
            'v 4 3',
            'e 0 1 3',
            'e 0 2 6',
            'e 0 3 2',
            'e 3 4 3',
            ''
        ]))
        fp.close()

        fp = StringIO()
        write_pace(fp, G, comments=['a', 'b'])
        self.assertEqual(fp.getvalue(), '\n'.join([
            'c a',
            'c b',
            'p ip 5 4 4',
            'v 0',
            'v 1 0 1 2',
            'v 2 1 3',
            'v 3 1',
            'v 4 3',
            'e 0 1 3',
            'e 0 2 6',
            'e 0 3 2',
            'e 3 4 3',
            ''
        ]))
        fp.close()
