import unittest
import networkx as nx

from readwrite import *
from algorithm.base.BaseSolver import BaseSolver


class Solver(BaseSolver):
    def solve(self, k: int, start_vertex: int, closed_walk: bool) -> bool:
        pass


class TestBaseSolver(unittest.TestCase):
    """Tests BaseSolver class."""

    def test_scale_weights_to_integers(self):
        G = load_pace('src/test/resources/instances/005_real_weight.gr')[0]
        solver = Solver(G)
        solver.scale_weight_to_integers(100.0)
        self.assertEqual(nx.get_edge_attributes(solver.G, 'weight'), {
            (0, 1): 3,
            (0, 2): 6,
            (0, 3): 2,
            (3, 4): 3,
        })

    def test_split_colors(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = Solver(G)
        solver.split_colors()
        self.assertEqual(nx.get_node_attributes(solver.G, 'color'), {
            0: [],
            1: [0],
            2: [1],
            3: [1],
            4: [3],
            5: [1],
            6: [2],
            7: [3],
        })
        self.assertEqual(nx.get_edge_attributes(solver.G, 'weight'), {
            (0, 1): 3,
            (0, 2): 6,
            (0, 3): 2,
            (1, 5): 0,
            (1, 6): 0,
            (2, 7): 0,
            (3, 4): 3,
        })

    def test_remove_unreachable_vertices(self):
        G = load_pace('src/test/resources/instances/108_unreachable.gr')[0]
        solver = Solver(G)
        solver.remove_unreachable_vertices(0)
        self.assertSetEqual(set(solver.G), {0, 3, 4})

    def test_colorless_vertices(self):
        G = load_pace('src/test/resources/instances/109_colorless.gr')[0]
        solver = Solver(G)
        solver.remove_colorless_vertices(0)
        self.assertSetEqual(set(solver.G), {0, 1, 3})

    def test_create_transitive_closure(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = Solver(G)
        solver.create_transitive_closure()
        self.assertEqual(nx.get_edge_attributes(solver.G, 'weight'), {
            (0, 1): 3,
            (0, 2): 6,
            (0, 3): 2,
            (0, 4): 5,
            (1, 2): 9,
            (1, 3): 5,
            (1, 4): 8,
            (2, 3): 8,
            (2, 4): 11,
            (3, 4): 3,
        })

        solver.create_transitive_closure()  # should change nothing
        self.assertEqual(nx.get_edge_attributes(solver.G, 'weight'), {
            (0, 1): 3,
            (0, 2): 6,
            (0, 3): 2,
            (0, 4): 5,
            (1, 2): 9,
            (1, 3): 5,
            (1, 4): 8,
            (2, 3): 8,
            (2, 4): 11,
            (3, 4): 3,
        })

        G = load_pace('src/test/resources/instances/006_non_metric.gr')[0]
        solver = Solver(G)
        solver.create_transitive_closure()
        self.assertEqual(nx.get_edge_attributes(solver.G, 'weight'), {
            (0, 1): 3,
            (0, 2): 4,
            (0, 3): 200,
            (1, 2): 1,
            (1, 3): 100,
            (2, 3): 2,
        })

        G = load_pace('src/test/resources/instances/006_non_metric.gr')[0]
        solver = Solver(G)
        solver.create_transitive_closure(create_metric=True)
        self.assertEqual(nx.get_edge_attributes(solver.G, 'weight'), {
            (0, 1): 3,
            (0, 2): 4,
            (0, 3): 6,
            (1, 2): 1,
            (1, 3): 3,
            (2, 3): 2,
        })

        # disconnected graph
        G = load_pace('src/test/resources/instances/111_minimal.gr')[0]
        solver = Solver(G)
        solver.create_transitive_closure()
        self.assertEqual(solver.G.number_of_edges(), 0)

    def test_set_critical_path(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = Solver(G)
        solver.split_colors()
        solver.create_transitive_closure()
        # print(nx.get_node_attributes(solver.G, 'label'))
        solver.set_critical_path([0, 5, 1, 4, 6, 7, 2, 0])
        self.assertListEqual(solver.solution, [0, 1, 0, 3, 4, 3, 0, 1, 0, 2, 0])

    def test_set_solution(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = Solver(G)
        solver.set_critical_path([0, 1, 0, 3, 4, 3, 0])
        self.assertEqual(solver.solution_weight, 16)
        self.assertEqual(solver.solution_colors, [0, 1, 2, 3])
