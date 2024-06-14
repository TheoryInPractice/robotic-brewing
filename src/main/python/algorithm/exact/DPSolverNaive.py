import math
from functools import cache
from typing import Optional
import networkx as nx
import sys
from algorithm.base import BaseSolver


class DPSolverNaive(BaseSolver):
    def __init__(self, G: nx.Graph) -> None:
        super().__init__(G)

    @cache
    def _solve(self, G: nx.Graph, s: int, v: int, colors: frozenset[int], steps: int) -> tuple[float, Optional[list[int]]]:
        """
        @param G connected, edge-weighted, vertex-single-colored graph
        @param s starting vertex
        @param v current (ending) vertex
        @param colors set of colors to collect
        @param steps number of hops of the walk
        """
        if steps == 0:
            if s == v and not colors:
                return 0, []
            else:
                return math.inf, None

        best = math.inf
        cert = None
        vcol = set(G.nodes[v]['color'])
        for u in G[v]:
            r, c = self._solve(G, s, u, colors - vcol, steps - 1)
            r += G.edges[u, v]['weight']
            if r < best:
                assert c is not None
                best = r
                cert = c + [v]
        return best, cert

    def solve(self, k: int, start_vertex: int, closed_walk: bool) -> bool:
        assert start_vertex in self.G

        n = len(self.G)
        if k != self.number_of_colors:
            raise NotImplementedError  # TODO
        if not closed_walk:
            raise NotImplementedError  # TODO

        # corner case
        if k <= len(self.G.nodes[start_vertex]['color']):
            self.set_solution([start_vertex])
            return True

        sys.setrecursionlimit(10000000)
        self.remove_unreachable_vertices(start_vertex)

        best = math.inf
        cert: Optional[list[int]] = None
        for t in range(2 * n):
            r, c = self._solve(self.G, start_vertex, start_vertex, frozenset(range(self.number_of_colors)), t)
            if r < best:
                best, cert = r, c

        if cert is None:
            return False
        else:
            path = list(reversed(cert)) + [start_vertex]
            self.set_critical_path(path)  # use critical_path as some vertices may be smoothed

            return True
