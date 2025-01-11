import unittest
from random import Random
import math

from algebra import FinitePolynomialRing


class TestFinitePolynomialRing(unittest.TestCase):
    """Tests FinitePolynomialRing class."""

    def test_add(self):
        R = FinitePolynomialRing(5, 4)
        x = R.one()
        y = R.zero()

        self.assertEqual(y + y, y)
        self.assertEqual(x + y, x)
        self.assertEqual(x + x, R([2, 0, 0, 0, 0]))
        self.assertEqual(x + 0, x)
        self.assertEqual(0 + x, x)
        self.assertRaises(NotImplementedError, lambda: x + 1)
        self.assertRaises(NotImplementedError, lambda: 1 + x)

    def test_sub(self):
        R = FinitePolynomialRing(5, 4)
        x = R.one()
        y = R.zero()

        self.assertEqual(y - x, R([3, 0, 0, 0, 0]))
        self.assertRaises(NotImplementedError, lambda: x - 1)

    def test_mul(self):
        R = FinitePolynomialRing(5, 4)
        x = R.one()
        y = R.zero()

        self.assertEqual(x * y, y)
        self.assertEqual(x * 2, R([2, 0, 0, 0, 0]))
        self.assertEqual(x * x, x)
        self.assertEqual(R([1, 1, 1, 1, 1]) * R([1, 2, 3, 2, 1]), R([2, 1, 0, 1, 2]))
        self.assertRaises(NotImplementedError, lambda: x * 1.5)

        R = FinitePolynomialRing(5, 256)
        self.assertEqual(R([1, 1, 1, 1, 1]) * R([1, 2, 3, 2, 1]), R([250, 253, 252, 1, 6]))

    def test_repr(self):
        R = FinitePolynomialRing(5, 4)
        self.assertEqual(repr(R.one()), '[1, 0, 0, 0, 0]')

    def test_repr_latex(self):
        R = FinitePolynomialRing(5, 4)
        self.assertEqual(R.one()._repr_latex_(), '$\\begin{bmatrix}1\\\\0\\\\0\\\\0\\\\0\\\\\\end{bmatrix}$')
