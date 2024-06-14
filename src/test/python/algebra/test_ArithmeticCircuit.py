import unittest

from algebra.ArithmeticCircuit import ArithmeticCircuit


class TestArithmeticCircuit(unittest.TestCase):
    """Tests ArithmeticCircuit class."""

    def test_properties(self):
        c = ArithmeticCircuit(3)
        self.assertEqual(c.number_of_variables(), 3)
        self.assertEqual(c.number_of_nodes(), 3)

        root = c.add_addition_gate(0, 1, 2)
        self.assertEqual(c.evaluate([1, 10, 100])[root], 111)
