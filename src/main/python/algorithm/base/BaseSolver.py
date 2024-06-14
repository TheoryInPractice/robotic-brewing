from typing import Optional, Union
import networkx as nx
from abc import ABC, abstractmethod


__all__ = ['BaseSolver']


class BaseSolver(ABC):
    def __init__(self, G: nx.Graph) -> None:
        n = len(G)
        m = G.number_of_edges()

        assert all(isinstance(v, int) for v in G), 'vertex labels must be integers'

        # Compact the color set.
        colors = nx.get_node_attributes(G, 'color')
        assert len(colors) == n, 'every node must have a color value'

        color_map: dict[int, int] = {}  # map from original color to relabeled color
        color_map_inv: list[int] = []  # map from relabeled color to original color

        for color in colors.values():
            for c in color:
                if c not in color_map:
                    x = len(color_map)
                    color_map[c] = x
                    color_map_inv += [c]

        # Checks edge weights.
        weights = nx.get_edge_attributes(G, 'weight')
        assert len(weights) == m, 'every edge must have a weight value'

        self.original_graph = G
        self.G = G.copy()
        nx.set_node_attributes(self.G, {v: [color_map[c] for c in color] for v, color in colors.items()}, 'color')
        self.number_of_colors = len(color_map)
        self.color_map = color_map
        self.color_map_inv = color_map_inv

        # fields for solution
        self.solution: Optional[list[int]] = None
        self.solution_colors: Optional[list[int]] = None
        self.solution_weight: Optional[Union[int, float]] = None

    @abstractmethod
    def solve(self, k: int, start_vertex: int, closed_walk: bool) -> bool:
        pass  # pragma: no cover

    def scale_weight_to_integers(self, scale: float) -> None:
        weights = nx.get_edge_attributes(self.G, 'weight')
        weights = {e: int(round(w * scale)) for e, w in weights.items()}
        nx.set_edge_attributes(self.G, weights, 'weight')

    def split_colors(self) -> None:
        """Split every multi-colored vertex into distinct vertices with a single color."""

        top_label = max(self.G) + 1

        for v in list(self.G):
            colors = list(self.G.nodes[v]['color'])
            if len(colors) > 1:
                self.G.nodes[v]['color'] = [colors[0]]

                for c in colors[1:]:
                    u = top_label
                    top_label += 1
                    self.G.add_node(u, color=[c], label=v)  # set original vertex as label
                    self.G.add_edge(v, u, weight=0)

    def remove_unreachable_vertices(self, source: int) -> None:
        """Removes all vertices unreachable from the source vertex."""

        reachable = nx.node_connected_component(self.G, source)
        to_remove = [v for v in self.G if v not in reachable]
        self.G.remove_nodes_from(to_remove)

    def remove_colorless_vertices(self, exception: int) -> None:
        """Removes all vertices without colors except for one vertex."""

        to_remove = [v for v in self.G if v != exception and not self.G.nodes[v]['color']]
        self.G.remove_nodes_from(to_remove)

    def create_transitive_closure(self, create_metric: bool = False) -> None:
        """Creates the transitive closure of the current graph.

        @param create_metric if True, updates edge weights to
                             the shortest distance between the endpoints,
                             thus making a metric complete graph.
        """

        n, m = self.G.number_of_nodes(), self.G.number_of_edges()
        if not create_metric and m * 2 == n * (n - 1):
            return  # already a complete graph

        length = dict(nx.all_pairs_dijkstra_path_length(self.G))
        vs = list(self.G)
        for i in range(n):
            for j in range(i + 1, n):
                u, v = vs[i], vs[j]
                if u not in length or v not in length[u]:
                    continue
                if not self.G.has_edge(u, v):
                    self.G.add_edge(u, v, weight=length[u][v])
                elif create_metric:
                    self.G.edges[u, v]['weight'] = length[u][v]

    def _get_label(self, v: int) -> int:
        return self.G.nodes[v].get('label', v)

    def set_critical_path(self, path: list[int]) -> None:
        assert path, 'path cannot be empty'

        xs = []
        for i in range(len(path) - 1):
            u = self._get_label(path[i])
            v = self._get_label(path[i + 1])
            if u == v:
                xs += [u]
            else:
                xs += nx.shortest_path(self.original_graph, u, v, weight='weight')[:-1]
        xs += [self._get_label(path[-1])]

        # remove consecutive duplicates
        solution = [xs[i] for i in range(len(xs)) if i == 0 or xs[i - 1] != xs[i]]
        self.set_solution(solution)

    def set_solution(self, path: list[int]) -> None:
        assert path, 'path cannot be empty'

        self.solution = path
        self.solution_colors = list({c for v in path for c in self.original_graph.nodes[v]['color']})

        edges = [(self.solution[i], self.solution[i + 1]) for i in range(len(self.solution) - 1)]
        self.solution_weight = sum(self.original_graph.edges[e]['weight'] for e in edges)
