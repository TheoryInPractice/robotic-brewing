from random import Random
from typing import Optional, Any
import math
from collections import deque
from .ArithmeticCircuit import ArithmeticCircuit
from .FinitePolynomialRing import FinitePolynomialRing

__all__ = ['MultilinearDetector']


class MultilinearDetector:
    def __init__(
        self,
        C: ArithmeticCircuit,
        k: int,
        ell: Optional[int] = None,
    ):
        assert k > 0, 'k must be a positive integer'

        self.C = C
        self.n = C.number_of_variables()
        self.k = k
        self.ell = int(math.ceil(3 + math.log2(self.C.max_degree()))) if ell is None else ell
        self.R = FinitePolynomialRing(self.ell, 2 ** (self.k + 1))
        self.last_x = None  # type: Optional[list[Any]]
        self.last_fingerprint = None  # type: Optional[dict[tuple[int, int], Any]]
        self.last_result = None  # type: Optional[dict[int, Any]]

    def run(
        self,
        rand: Random,
        num_iterations: int = 30,
        early_return: bool = True
    ) -> bool:
        true_count = 0
        total_count = 0

        for _ in range(num_iterations):
            ret = self._run_iteration(rand)
            true_count += 1 if ret else 0
            total_count += 1
            # sys.stdout.write('T' if ret else 'F')

            if early_return and ret:
                break
        # print(f'\ntrue rate: {true_count}/{total_count} = {true_count / total_count:.3f}')
        return true_count > 0

    def _run_iteration(self, rand: Random) -> bool:

        # Create random values for variables.
        # x_i <- Z_2^k
        x = [rand.randint(1, 2 ** self.k - 1) for _ in range(self.n)]

        # Create random fingerprints for all in-edges to addition gates.
        # y_j <- R

        y = {j: self.R([rand.randint(0, 1) for _ in range(self.ell)]) for j in self.C.fingerprint_edges()}
        # print('y:', y)

        # Main loop.
        self._run_evaluation(x, y)
        assert self.last_result is not None

        return bool(self.last_result[self.C.output_node])

    def _run_evaluation(self, x: list[Any], y: dict[tuple[int, int], Any]) -> None:
        ret = {i: self.R.zero() for i in range(self.C.number_of_nodes())}

        for t in range(2 ** self.k):
            z = [self.R([1 - (-1) ** ((x[i] & t).bit_count() % 2)] + [0] * (self.ell - 1)) for i in range(self.n)]
            result = self.C.evaluate(z, y)

            for r, val in result.items():
                ret[r] = ret[r] + val

        self.last_x = x.copy()
        self.last_fingerprint = y.copy()
        self.last_result = ret.copy()

    # Deprecated: Monte Carlo algorithm
    # def find_certificate(self, rand: Random, num_iterations: Optional[int]=None) -> ArithmeticCircuit:
    #     C = self.C.copy()
    #     n = C.number_of_nodes()
    #     k = self.k

    #     if num_iterations is None:
    #         num_iterations = 1 + int(math.ceil(math.log2(2 * k * math.log2(C.number_of_nodes()))))

    #     # Preprocessing
    #     C.remove_unreachable()

    #     # Traversal.
    #     p = {i: len(C.out_edges[i]) for i in range(n)}
    #     q: deque[int] = deque()
    #     q.append(C.output_node)

    #     while q:
    #         v = q.popleft()
    #         if C.node_types[v] == 0:
    #             continue

    #         # Select a path at an addition gate.
    #         next_target = C.in_edges[v].copy()
    #         clean_in_edges = False

    #         if v != C.output_node and not C.out_edges[v]:
    #             clean_in_edges = True
    #         else:
    #             assert v == C.output_node or len(C.out_edges[v]) == 1
    #             if C.node_types[v] == 1:
    #                 us = C.in_edges[v].copy()
    #                 assert us, f'empty in-neighbors: {us}'

    #                 # binary search
    #                 left, right = 0, len(us)

    #                 while right - left > 1:
    #                     m = (left + right) // 2

    #                     det = MultilinearDetector(C, k)

    #                     # remove edges
    #                     for i in range(left, m):
    #                         C.remove_edge(us[i], v)
    #                         p[us[i]] -= 1

    #                     # test
    #                     ret = det.run(rand, num_iterations=num_iterations)

    #                     if ret:
    #                         left = m  # safe to remove
    #                     else:
    #                         # restore edges
    #                         for i in range(left, m):
    #                             C.add_edge(us[i], v)
    #                             p[us[i]] += 1
    #                         for i in range(m, right):
    #                             C.remove_edge(us[i], v)
    #                             p[us[i]] -= 1
    #                         right = m

    #         # Continue traversal.
    #         for u in next_target:
    #             if u in C.in_edges[v]:
    #                 p[u] -= 1
    #             if p[u] == 0:
    #                 q.append(u)
    #         if clean_in_edges:
    #             C.remove_node(v)

    #     return C

    # New: Las Vegas algorithm
    def find_certificate(self, rand: Random) -> ArithmeticCircuit:
        C = self.C.copy()
        n = C.number_of_nodes()
        k = self.k

        # Preprocessing
        C.remove_unreachable()

        # Traversal.
        p = {i: len(C.out_edges[i]) for i in range(n)}
        q: deque[int] = deque()
        q.append(C.output_node)

        while q:
            v = q.popleft()
            if C.node_types[v] == 0:
                continue

            # Select a path at an addition gate.
            next_target = C.in_edges[v].copy()
            clean_in_edges = False

            if v != C.output_node and not C.out_edges[v]:
                clean_in_edges = True
            else:
                assert v == C.output_node or len(C.out_edges[v]) == 1
                if C.node_types[v] == 1:
                    us = C.in_edges[v].copy()
                    assert us, f'empty in-neighbors: {us}'

                    # binary search
                    left, right = 0, len(us)

                    while right - left > 1:
                        m = (left + right) // 2
                        a, b = left, m
                        while True:
                            # remove edges:
                            for i in range(a, b):
                                C.remove_edge(us[i], v)
                                p[us[i]] -= 1

                            # test
                            det = MultilinearDetector(C, k)
                            ret = det.run(rand, num_iterations=1)

                            if ret:  # safe to remove
                                if a == left:
                                    left = b
                                else:
                                    right = a
                                break

                            # restore edges
                            for i in range(a, b):
                                C.add_edge(us[i], v)
                                p[us[i]] += 1

                            # swap intervals
                            if a == left:
                                a, b = m, right
                            else:
                                a, b = left, m

            # Continue traversal.
            for u in next_target:
                if u in C.in_edges[v]:
                    p[u] -= 1
                if p[u] == 0:
                    q.append(u)
            if clean_in_edges:
                C.remove_node(v)

        return C
