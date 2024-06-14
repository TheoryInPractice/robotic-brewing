import unittest
from random import Random
import math

from algebra import ArithmeticCircuit, MultilinearDetector


class TestMultilinearDetector(unittest.TestCase):
    """Tests MultilinearDetector class."""

    def test_find_certificate(self):
        C = ArithmeticCircuit(5)
        a1 = C.add_addition_gate(0, 1, 2)
        a2 = C.add_addition_gate(2, 3, 4)
        a3 = C.add_addition_gate(2, 3, 4)
        a4 = C.add_addition_gate(3, 4)
        root = C.add_multiplication_gate(a1, a2, a3, a4)
        rand = Random(12345)

        for _ in range(3):
            detector = MultilinearDetector(C, 3)
            self.assertFalse(detector.run(rand, 10))

            detector = MultilinearDetector(C, 4)
            self.assertTrue(detector.run(rand, 10))

            cert = detector.find_certificate(Random(12345))
            expect = [math.prod([2, 5, 7, 11]), math.prod([3, 5, 7, 11])]
            self.assertIn(cert.evaluate([2, 3, 5, 7, 11])[root], expect)

        # -----------------------------------------------------------------------
        C = ArithmeticCircuit(6)
        a11 = C.add_addition_gate(0, 1, 4)
        a12 = C.add_addition_gate(0, 1, 4)
        a13 = C.add_addition_gate(0, 1, 4)
        a14 = C.add_addition_gate(0, 1, 4)
        a15 = C.add_addition_gate(0, 1, 4)
        m21 = C.add_multiplication_gate(2, a15)
        m22 = C.add_multiplication_gate(3, a14)
        m23 = C.add_multiplication_gate(4, a13)
        m24 = C.add_multiplication_gate(5, a12)
        a31 = C.add_addition_gate(m22, 5)
        a32 = C.add_addition_gate(m23, m24)
        a33 = C.add_addition_gate(m21, 5)
        a34 = C.add_addition_gate(m23, m24)
        m41 = C.add_multiplication_gate(a33, a34)
        m42 = C.add_multiplication_gate(a31, a32, a11)
        root = C.add_addition_gate(m41, m42)
        rand = Random(12345)

        for _ in range(3):
            detector = MultilinearDetector(C, 2)
            self.assertFalse(detector.run(rand, 10))

            detector = MultilinearDetector(C, 3)
            self.assertTrue(detector.run(rand, 10))

            cert = detector.find_certificate(rand)
            expect = [math.prod([2, 11, 13]), math.prod([3, 11, 13])]
            self.assertIn(cert.evaluate([2, 3, 5, 7, 11, 13])[root], expect)

        # -----------------------------------------------------------------------
        # x_0^4 + x_0 x_2 + x_1 x_2
        C = ArithmeticCircuit(3)
        a1 = C.add_addition_gate(0, 1)
        m1 = C.add_multiplication_gate(a1, 2)
        a2 = C.add_addition_gate(0)
        m2 = C.add_multiplication_gate(a2, 0)
        m3 = C.add_multiplication_gate(m2, 0)
        m4 = C.add_multiplication_gate(m3, 0)
        root = C.add_addition_gate(m1, m4)
        rand = Random(12345)

        for _ in range(3):
            detector = MultilinearDetector(C, 1)
            self.assertFalse(detector.run(rand, 10))

            detector = MultilinearDetector(C, 2)
            self.assertTrue(detector.run(rand, 10))

            cert = detector.find_certificate(rand)
            expect = [math.prod([2, 5]), math.prod([3, 5])]
            self.assertIn(cert.evaluate([2, 3, 5])[root], expect)

            detector = MultilinearDetector(C, 3)
            self.assertTrue(detector.run(rand, 10))

            detector = MultilinearDetector(C, 4)
            self.assertTrue(detector.run(rand, 10))

        # -----------------------------------------------------------------------
        C = ArithmeticCircuit(5)
        a11 = C.add_addition_gate(1, 2, 3)
        a12 = C.add_addition_gate(3, 4)
        a13 = C.add_addition_gate(3, 4)
        a14 = C.add_addition_gate(1, 2, 3)
        a15 = C.add_addition_gate(1, 2, 3)
        a16 = C.add_addition_gate(0, 1)
        a17 = C.add_addition_gate(0, 1)
        m21 = C.add_multiplication_gate(a16, a17)
        m22 = C.add_multiplication_gate(0, a15)
        m23 = C.add_multiplication_gate(a14, 4)
        m24 = C.add_multiplication_gate(a12, a13)
        a31 = C.add_addition_gate(m21, m22)
        a32 = C.add_addition_gate(m23, m24)
        root = C.add_multiplication_gate(a11, a31, a32)

        rand = Random(12345)

        for _ in range(3):
            detector = MultilinearDetector(C, 4)
            self.assertFalse(detector.run(rand, 10))

            detector = MultilinearDetector(C, 5)
            self.assertTrue(detector.run(rand, 10))

            cert = detector.find_certificate(rand)
            self.assertEqual(cert.evaluate([2, 3, 5, 7, 11])[root], math.prod([2, 3, 5, 7, 11]))

        # -----------------------------------------------------------------------
        C = ArithmeticCircuit(2)
        m1 = C.add_multiplication_gate(0, 1)
        a1 = C.add_addition_gate(m1)
        root = C.add_addition_gate(0, 1)

        for _ in range(3):
            detector = MultilinearDetector(C, 1)
            self.assertTrue(detector.run(rand, 10))

            cert = detector.find_certificate(rand)
            self.assertIn(cert.evaluate([2, 3])[root], [2, 3])
