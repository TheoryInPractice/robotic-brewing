import math
from functools import cache
import networkx as nx
from algorithm.base import BaseSolver


class DPSolver(BaseSolver):
    def __init__(self, G: nx.Graph) -> None:
        super().__init__(G)

    @cache
    def _solve(self, G: nx.Graph, s: int, v: int, colors: frozenset[int]):
        """
        @param G connected, edge-weighted, vertex-single-colored graph
        @param s starting vertex
        @param v current (ending) vertex
        @param colors set of colors to collect
        """
        vcol = set(G.nodes[v]['color'])
        if s == v and not colors:
            return 0, []
        elif not (vcol & colors):
            return math.inf, None
        else:
            best = math.inf
            cert = None
            for u in G[v]:
                r, c = self._solve(G, s, u, colors - vcol)
                r += G.edges[u, v]['weight']
                if r < best:
                    best = r
                    cert = c + [v]
            return best, cert

    def solve(self, k: int, start_vertex: int, closed_walk: bool) -> bool:
        assert start_vertex in self.G

        c = self.number_of_colors
        if k != c:
            raise NotImplementedError  # TODO
        if not closed_walk:
            raise NotImplementedError  # TODO

        # corner case
        if k <= len(self.G.nodes[start_vertex]['color']):
            self.set_solution([start_vertex])
            return True

        self.remove_unreachable_vertices(start_vertex)

        # assign a new color to start_vertex
        self.G.nodes[start_vertex]['color'] += [c]
        self.create_transitive_closure(create_metric=True)

        # remove vertices with no color (this must be after creating the transitive closure)
        self.remove_colorless_vertices(start_vertex)

        cert = self._solve(self.G, start_vertex, start_vertex, frozenset(range(c + 1)))[1]
        if cert is not None:
            path = [v for v in reversed(cert)] + [start_vertex]
            self.set_critical_path(path)
        return cert is not None
