import galois

__all__ = ['FinitePolynomialRing']


def create_mul_table(ell):
    vec: list[list[tuple[int, int, int]]] = [[] for _ in range(2 * ell - 1)]
    GF = galois.GF(2, ell)
    degs = GF.irreducible_poly.nonzero_degrees[1:]

    for i in range(ell):
        for j in range(ell):
            vec[i + j] += [(1, i, j)]

    for d in range(2 * ell - 2, ell - 1, -1):
        for _, i, j in vec[d]:
            for e in degs:
                vec[d - ell + e] += [(-1, i, j)]
        vec.pop()

    return vec


def FinitePolynomialRing(ell, modulo):
    # create multiplication table
    mul_table = create_mul_table(ell)

    class R:
        def __init__(self, values):
            self.ell = ell
            self.modulo = modulo
            self.values = values
            assert len(values) == ell

        def __eq__(self, other) -> bool:
            if not isinstance(other, R):
                return False
            return self.ell == other.ell and self.modulo == other.modulo and self.values == other.values

        def __add__(self, other) -> 'R':
            if other == 0:
                return self
            if not isinstance(other, R):
                raise NotImplementedError
            return R([(x + y) % self.modulo for x, y in zip(self.values, other.values)])
        __radd__ = __add__

        def __mul__(self, other) -> 'R':
            if isinstance(other, int):
                # scalar multiplication
                return R([x * other % self.modulo for x in self.values])
            elif not isinstance(other, R):
                raise NotImplementedError
            ret = [
                sum(s * self.values[i] * other.values[j] for s, i, j in d) % self.modulo
                for d in mul_table
            ]
            return R(ret)
        __rmul__ = __mul__

        def __neg__(self) -> 'R':
            return R([(-x) % self.modulo for x in self.values])

        def __sub__(self, other) -> 'R':
            if not isinstance(other, R):
                raise NotImplementedError
            return self + (-other)

        def __bool__(self) -> bool:
            return any(self.values)

        @staticmethod
        def zero() -> 'R':
            return R([0 for _ in range(ell)])

        @staticmethod
        def one() -> 'R':
            return R([1] + [0 for _ in range(ell - 1)])

        def __repr__(self):
            return repr(self.values)

        def _repr_latex_(self):
            terms = []

            buff = [r'\begin{bmatrix}']
            buff += [str(x) + r'\\' for x in self.values]
            buff += [r'\end{bmatrix}']
            terms += [''.join(buff)]
            return '$' + ' + '.join(terms) + '$'
    return R
