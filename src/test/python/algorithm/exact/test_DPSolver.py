import unittest
import networkx as nx

from readwrite import *
from algorithm.exact.DPSolver import DPSolver


class TestDPSolver(unittest.TestCase):
    """Tests DPSolver class."""

    def test_solve(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = DPSolver(G)
        self.assertTrue(solver.solve(4, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 0, 3, 4, 3, 0],
            [0, 3, 4, 3, 0, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 16)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/002_tiny.gr')[0]
        solver = DPSolver(G)
        self.assertTrue(solver.solve(4, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 2, 0],
            [0, 2, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 15)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/003_tiny.gr')[0]
        solver = DPSolver(G)
        self.assertTrue(solver.solve(7, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 2, 3, 4, 1, 7, 6, 7, 1, 0],
            [0, 1, 7, 6, 7, 1, 4, 3, 2, 1, 0],
            [0, 1, 7, 6, 7, 1, 2, 3, 4, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3, 4, 5, 6])
        self.assertEqual(solver.solution_weight, 55)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/004_tiny.gr')[0]
        solver = DPSolver(G)
        self.assertTrue(solver.solve(6, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 2, 6, 7, 6, 2, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3, 4, 5])
        self.assertEqual(solver.solution_weight, 108)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/006_non_metric.gr')[0]
        solver = DPSolver(G)
        self.assertTrue(solver.solve(2, 0, closed_walk=True))
        self.assertListEqual(solver.solution, [0, 1, 2, 3, 2, 1, 0])
        self.assertListEqual(solver.solution_colors, [0, 1])
        self.assertEqual(solver.solution_weight, 12)

    def test_non_consecutive_colors(self):
        G = nx.empty_graph(5)
        nx.set_node_attributes(G, {
            0: [],
            1: [0, 2],
            2: [1, 4],
            3: [1],
            4: [4],
        }, 'color')

        G.add_edge(0, 1, weight=3)
        G.add_edge(0, 2, weight=6)
        G.add_edge(0, 3, weight=2)
        G.add_edge(3, 4, weight=3)

        solver = DPSolver(G)
        self.assertTrue(solver.solve(4, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 0, 3, 4, 3, 0],
            [0, 3, 4, 3, 0, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 4])

        # -----------------------------------------------------------------------

        G = nx.empty_graph(5)

        nx.set_node_attributes(G, {
            0: set(),
            1: {10, 20},
            2: {20},
            3: {1},
            4: {3}
        }, 'color')

        G.add_edge(0, 1, weight=3)
        G.add_edge(0, 2, weight=6)
        G.add_edge(0, 3, weight=2)
        G.add_edge(3, 4, weight=3)

        solver = DPSolver(G)

        self.assertTrue(solver.solve(4, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 0, 3, 4, 3, 0],
            [0, 3, 4, 3, 0, 1, 0],
        ])

        self.assertListEqual(solver.solution_colors, [1, 10, 3, 20])

    # -----------------------------------------------------------------------

    def collecting_all_colors_is_impossible(self):
        G = nx.empty_graph(5)

        nx.set_node_attributes(G, {
            0: set(),
            1: {10, 20},
            2: {20},
            3: {1},
            4: {3}
        }, 'color')

        G.add_edge(0, 1, weight=3)
        G.add_edge(0, 2, weight=6)
        G.add_edge(0, 3, weight=2)
        G.add_edge(3, 2, weight=3)

        solver = DPSolver(G)

        self.assertFalse(solver.solve(4, 0, closed_walk=True))

        # -----------------------------------------------------------------------

        G = nx.empty_graph(5)

        nx.set_node_attributes(G, {
            0: set(),
            1: {10, 20},
            2: {20},
            3: {1},
            4: {3}
        }, 'color')

        G.add_edge(0, 1, weight=3)
        G.add_edge(0, 2, weight=6)
        G.add_edge(0, 3, weight=2)
        G.add_edge(3, 2, weight=3)

        solver = DPSolver(G)

        self.assertFalse(solver.solve(4, 4, closed_walk=True))


# -----------------------------------------------------------------------


    def test_solve_random_start_vertex(self):

        G = nx.empty_graph()

        G.add_nodes_from([1, 2, 3, 4, 5])

        nx.set_node_attributes(G, {
            1: set(),
            2: {10, 20},
            3: {20},
            4: {1},
            5: {3}
        }, 'color')

        G.add_edge(1, 2, weight=3)
        G.add_edge(1, 3, weight=6)
        G.add_edge(1, 4, weight=2)
        G.add_edge(4, 5, weight=3)

        solver = DPSolver(G)

        self.assertTrue(solver.solve(4, 4, closed_walk=True))

        self.assertListEqual(solver.solution_colors, [1, 10, 3, 20])

    # -----------------------------------------------------------------------

        G = nx.empty_graph(5)

        nx.set_node_attributes(G, {
            0: set(),
            1: {10, 20},
            2: {20},
            3: {1},
            4: {3}
        }, 'color')

        G.add_edge(0, 1, weight=3)
        G.add_edge(0, 2, weight=6)
        G.add_edge(0, 3, weight=2)
        G.add_edge(3, 4, weight=3)

        solver = DPSolver(G)

        self.assertTrue(solver.solve(4, 3, closed_walk=True))
        self.assertIn(solver.solution, [
            [3, 4, 3, 0, 1, 0, 3],
            [3, 0, 3, 4, 3, 0, 1, 0],
        ])

        self.assertListEqual(solver.solution_colors, [1, 10, 3, 20])


# -----------------------------------------------------------------------


    def test_random_graph_structure(self):

        G = nx.empty_graph()

        G.add_nodes_from([10, 20, 30, 40, 50])

        nx.set_node_attributes(G, {
            10: set(),
            20: {10, 20},
            30: {20},
            40: {1},
            50: {3}
        }, 'color')

        G.add_edge(10, 20, weight=3)
        G.add_edge(10, 30, weight=6)
        G.add_edge(10, 40, weight=2)
        G.add_edge(40, 50, weight=3)

        solver = DPSolver(G)

        self.assertTrue(solver.solve(4, 10, closed_walk=True))

# -----------------------------------------------------------------------

    def test_non_consecutive_vertices(self):

        G = nx.empty_graph()

        G.add_nodes_from([1, 2, 4, 8, 20])

        nx.set_node_attributes(G, {
            1: [],
            2: [0, 2],
            4: [1, 3],
            8: [1],
            20: [3]
        }, 'color')

        G.add_edge(1, 2, weight=3)
        G.add_edge(1, 4, weight=6)
        G.add_edge(1, 8, weight=2)
        G.add_edge(8, 20, weight=3)

        self.assertTrue(DPSolver(G))

    def test_solve_minimal(self):
        G = load_pace(f'src/test/resources/instances/110_empty.gr')[0]
        solver = DPSolver(G)
        self.assertTrue(solver.solve(0, 0, closed_walk=True))
        self.assertListEqual(solver.solution, [0])
        self.assertListEqual(solver.solution_colors, [])
        self.assertEqual(solver.solution_weight, 0)

        G = load_pace(f'src/test/resources/instances/111_minimal.gr')[0]
        solver = DPSolver(G)
        self.assertFalse(solver.solve(1, 0, closed_walk=True))

        solver = DPSolver(G)
        self.assertTrue(solver.solve(1, 1, closed_walk=True))
        self.assertListEqual(solver.solution, [1])
        self.assertListEqual(solver.solution_colors, [0])
        self.assertEqual(solver.solution_weight, 0)

        G = load_pace(f'src/test/resources/instances/112_minimal.gr')[0]
        solver = DPSolver(G)
        self.assertFalse(solver.solve(1, 100, closed_walk=True))

        solver = DPSolver(G)
        self.assertTrue(solver.solve(1, 103, closed_walk=True))
        self.assertListEqual(solver.solution, [103])
        self.assertListEqual(solver.solution_colors, [97])
        self.assertEqual(solver.solution_weight, 0)
