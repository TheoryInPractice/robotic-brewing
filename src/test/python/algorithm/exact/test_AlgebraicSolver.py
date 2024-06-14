import unittest
import pytest
from random import Random

from readwrite import *
from algorithm.exact.AlgebraicSolver import AlgebraicSolver


class TestAlgebraicSolver(unittest.TestCase):
    """Tests AlgebraicSolver class."""

    def test_solve(self):
        num_iterations = 4  # speed up the test

        G = load_pace('src/test/resources/instances/001_tiny.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(4, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 0, 3, 4, 3, 0],
            [0, 3, 4, 3, 0, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 16)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/002_tiny.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(4, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 2, 0],
            [0, 2, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3])
        self.assertEqual(solver.solution_weight, 15)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/006_non_metric.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(2, 0, closed_walk=True))
        self.assertListEqual(solver.solution, [0, 1, 2, 3, 2, 1, 0])
        self.assertListEqual(solver.solution_colors, [0, 1])
        self.assertEqual(solver.solution_weight, 12)

    def test_solve_corner(self):
        num_iterations = 5  # speed up the test

        for name in ['101', '102', '103']:
            G = load_pace(f'src/test/resources/instances/{name}_corner.gr')[0]
            solver = AlgebraicSolver(G, Random(12345), num_iterations)
            self.assertFalse(solver.solve(3, 6, closed_walk=True))

        for w, name in enumerate(['104', '105', '106']):
            G = load_pace(f'src/test/resources/instances/{name}_corner.gr')[0]
            solver = AlgebraicSolver(G, Random(12345), num_iterations)
            self.assertTrue(solver.solve(3, 6, closed_walk=True))
            self.assertListEqual(solver.solution, [6, 4, 6])
            self.assertListEqual(solver.solution_colors, [1, 3, 5])
            self.assertEqual(solver.solution_weight, w * 2)

        G = load_pace(f'src/test/resources/instances/107_corner.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(2, 0, closed_walk=True))
        self.assertListEqual(solver.solution, [0, 1, 0])
        self.assertListEqual(solver.solution_colors, [0, 1])
        self.assertEqual(solver.solution_weight, 10)

    def test_solve_minimal(self):
        num_iterations = 3

        G = load_pace(f'src/test/resources/instances/110_empty.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(0, 0, closed_walk=True))
        self.assertListEqual(solver.solution, [0])
        self.assertListEqual(solver.solution_colors, [])
        self.assertEqual(solver.solution_weight, 0)

        G = load_pace(f'src/test/resources/instances/111_minimal.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertFalse(solver.solve(1, 0, closed_walk=True))

        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(1, 1, closed_walk=True))
        self.assertListEqual(solver.solution, [1])
        self.assertListEqual(solver.solution_colors, [0])
        self.assertEqual(solver.solution_weight, 0)

        G = load_pace(f'src/test/resources/instances/112_minimal.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertFalse(solver.solve(1, 100, closed_walk=True))

        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(1, 103, closed_walk=True))
        self.assertListEqual(solver.solution, [103])
        self.assertListEqual(solver.solution_colors, [97])
        self.assertEqual(solver.solution_weight, 0)

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/114_colorless.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(2, 0, closed_walk=True))
        self.assertListEqual(solver.solution, [0, 1, 2, 1, 0])
        self.assertListEqual(solver.solution_colors, [0, 1])
        self.assertEqual(solver.solution_weight, 6)

    @pytest.mark.skip(reason="very slow")
    def test_solve_large(self):
        num_iterations = 6  # speed up the test

        # -----------------------------------------------------------------------
        G = load_pace('src/test/resources/instances/003_tiny.gr')[0]
        solver = AlgebraicSolver(G, Random(12345), num_iterations)
        self.assertTrue(solver.solve(7, 0, closed_walk=True))
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
        solver = AlgebraicSolver(G, Random(123456), num_iterations)
        self.assertTrue(solver.solve(6, 0, closed_walk=True))
        self.assertIn(solver.solution, [
            [0, 1, 2, 6, 7, 6, 2, 1, 0],
        ])
        self.assertListEqual(solver.solution_colors, [0, 1, 2, 3, 4, 5])
        self.assertEqual(solver.solution_weight, 108)
