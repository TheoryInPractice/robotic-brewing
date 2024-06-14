import unittest

from readwrite.iris import load_iris


class TestLoadIris(unittest.TestCase):
    """Tests the load_iris function."""

    # Path to the test graph files graph_vertex and graph_edge. With 40 vertices and 350 edges.
    path_prefix = 'src/test/resources/test_files/graph'

    def test_graph_structure(self):
        graph = load_iris(self.path_prefix)

        self.assertGreater(len(graph.nodes), 0, 'Graph should have at least one node')
        self.assertGreater(len(graph.edges), 0, 'Graph should have at least one edge')

    def test_node_properties(self):
        graph = load_iris(self.path_prefix)

        self.assertIn(1, graph.nodes, 'Graph should contain node with ID 1')

        node_color = graph.nodes[1].get('color')
        self.assertIsNotNone(node_color, 'Node should have a "color" property')
        self.assertIsInstance(node_color, list, '"color" property should be a list')
        self.assertTrue(all(isinstance(color, int) for color in node_color), 'All colors should be integers')

    def test_edge_properties(self):
        graph = load_iris(self.path_prefix)

        edge_data = graph[1][2]
        self.assertIn('weight', edge_data, 'Edge should have a "weight" property')
        self.assertIsInstance(edge_data['weight'], (int, float), '"weight" should be an integer or float')

        self.assertIn('checked', edge_data, 'Edge should have a "checked" property')
        self.assertIsInstance(edge_data['checked'], bool, '"checked" should be a boolean')
        self.assertIn('valid', edge_data, 'Edge should have a "valid" property')
        self.assertIsInstance(edge_data['valid'], bool, '"valid" should be a boolean')

    def test_graph_specifics(self):
        graph = load_iris(self.path_prefix)

        # Test the total number of vertices and edges
        self.assertEqual(len(graph.nodes), 40, 'Graph should have exactly 40 nodes')
        self.assertEqual(len(graph.edges), 350, 'Graph should have exactly 350 edges')

        # Test the global visibility set for node 2
        node_2_colors = set(graph.nodes[2]['color'])

        expected_colors = {6897, 6910, 6911, 6915, 6917, 16790, 16791, 16800, 17001, 17003, 17009, 17036, 17045, 17046, 17047}
        self.assertSetEqual(node_2_colors, expected_colors, 'Global visibility set does not match expected')

        # Empty colors node
        node_3_colors = set(graph.nodes[5]['color'])
        self.assertSetEqual(node_3_colors, set(), 'Node with empty color set should have an empty set')
