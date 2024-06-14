import unittest
from readwrite import *

try:
    import gurobipy
except ImportError:
    raise unittest.SkipTest('Gurobi is not installed')

from algorithm.exact.ILPSolver import ILPSolver


class TestILPSolver(unittest.TestCase):
    """Tests DPSolver class."""

    def testTinyGraph(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = ILPSolver(G)
        solver.solve(4, 0)

        self.assertIn(solver.solution, [
            [0, 1, 0, 3, 4, 3, 0],
            [0, 3, 4, 3, 0, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 16)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/002_tiny.gr')[0]
        solver = ILPSolver(G)
        solver.solve(4, 0)

        self.assertIn(solver.solution, [
            [0, 1, 2, 0],
            [0, 2, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 15)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/003_tiny.gr')[0]
        solver = ILPSolver(G)
        solver.solve(7, 0)

        self.assertIn(solver.solution, [
            [0, 1, 2, 3, 4, 1, 7, 6, 7, 1, 0],
            [0, 1, 4, 3, 2, 1, 7, 6, 7, 1, 0],
            [0, 1, 7, 6, 7, 1, 4, 3, 2, 1, 0],
            [0, 1, 7, 6, 7, 1, 2, 3, 4, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3, 4, 5, 6])
        self.assertEqual(solver.solution_weight, 55)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/004_tiny.gr')[0]
        solver = ILPSolver(G)
        solver.solve(6, 0)

        self.assertIn(solver.solution, [
            [0, 1, 2, 6, 7, 6, 2, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3, 4, 5])
        self.assertEqual(solver.solution_weight, 108)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/006_non_metric.gr')[0]
        solver = ILPSolver(G)
        solver.solve(2, 0)

        self.assertListEqual(solver.solution, [0, 1, 2, 3, 2, 1, 0])
        self.assertListEqual(solver.solution_colors, [0, 1])
        self.assertEqual(solver.solution_weight, 12)

# -----------------------------------------------------------------------
    def test_non_consecutive_vertices(self):
        G = load_pace('src/test/resources/instances/201_nonconsecutive_vertices.gt')[0]

        solver = ILPSolver(G)
        solver.solve(4, 0)

        self.assertIn(solver.solution, [
            [0, 1, 0, 4, 5, 4, 0],
            [0, 4, 5, 4, 0, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 16)

# -----------------------------------------------------------------------
    def test_non_consecutive_colors(self):
        G = load_pace('src/test/resources/instances/301_nonconsecutive_colors.gt')[0]

        solver = ILPSolver(G)
        solver.solve(4, 0)

        self.assertIn(solver.solution, [
            [0, 1, 0, 3, 4, 3, 0],
            [0, 3, 4, 3, 0, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 4])
        self.assertEqual(solver.solution_weight, 16)

# -----------------------------------------------------------------------
    def test_not_all_colors(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = ILPSolver(G)
        solver.solve(3, 0)

        self.assertIn(solver.solution, [
            [0, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2])
        self.assertEqual(solver.solution_weight, 6)

        solver = ILPSolver(G)
        solver.solve(1, 0)

        self.assertIn(solver.solution, [
            [0, 3, 0],
        ])
        self.assertListEqual(solver.solution_colors, [1])
        self.assertEqual(solver.solution_weight, 4)

# -----------------------------------------------------------------------
    def test_non_zero_start(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = ILPSolver(G)
        solver.solve(4, 1, 1)

        self.assertIn(solver.solution, [
            [1, 0, 3, 4, 3, 0, 1],
            [1, 0, 3, 4, 3, 0, 1],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 16)

# -----------------------------------------------------------------------
    def test_non_zero(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = ILPSolver(G)
        solver.solve(1, 3)

        self.assertIn(solver.solution, [
            [3, 0],
        ])
        self.assertListEqual(solver.solution_colors, [1])
        self.assertEqual(solver.solution_weight, 2)

# -----------------------------------------------------------------------
    def test_minimal(self):
        G = load_pace(f'src/test/resources/instances/111_minimal.gr')[0]
        solver = ILPSolver(G)
        solver.solve(1, 1, 1)
        self.assertListEqual(solver.solution, [1])
        self.assertListEqual(solver.solution_colors, [0])
        self.assertEqual(solver.solution_weight, 0)

# -----------------------------------------------------------------------
    def test_start_not_ending(self):
        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = ILPSolver(G)
        solver.solve(4, 0, 1)

        self.assertIn(solver.solution, [
            [0, 3, 4, 3, 0, 1],
        ])

        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 13)

# -----------------------------------------------------------------------
    def test_k_is_zero(self):
        G = load_pace(f'src/test/resources/instances/001_tiny.gr')[0]
        solver = ILPSolver(G)

        solver.solve(0, 0)

        self.assertListEqual(solver.solution, [0])
        self.assertListEqual(solver.solution_colors, [])
        self.assertEqual(solver.solution_weight, 0)

    # -----------------------------------------------------------------------

        G = load_pace(f'src/test/resources/instances/110_empty.gr')[0]
        solver = ILPSolver(G)

        solver.solve(0, 0)

        self.assertListEqual(solver.solution, [0])
        self.assertListEqual(solver.solution_colors, [])
        self.assertEqual(solver.solution_weight, 0)

    # -----------------------------------------------------------------------

        G = load_pace(f'src/test/resources/instances/007_revisit.gr')[0]
        solver = ILPSolver(G)

        solver.solve(4, 0)

        self.assertIn(solver.solution, [
            [0, 2, 0, 3, 0, 4, 0, 1, 0],
        ])

        self.assertListEqual(solver.solution_colors, [1, 2, 3, 4])
        self.assertEqual(solver.solution_weight, 20)

# -----------------------------------------------------------------------
    def test_zero_weight(self):
        G = load_pace(f'src/test/resources/instances/009_zero_weight.gr')[0]
        solver = ILPSolver(G)

        solver.solve(2, 0)

        self.assertIn(solver.solution, [
            [0, 2, 1, 2, 0],
        ])

        self.assertListEqual(solver.solution_colors, [1, 2])
        self.assertEqual(solver.solution_weight, 0)

# -----------------------------------------------------------------------

    def test_minimal_112(self):
        G = load_pace(f'src/test/resources/instances/112_minimal.gr')[0]
        solver = ILPSolver(G)
        solver.solve(1, 103, 103)
        self.assertListEqual(solver.solution, [103])
        self.assertListEqual(solver.solution_colors, [97])
        self.assertEqual(solver.solution_weight, 0)


if __name__ == '__main__':
    unittest.main()
