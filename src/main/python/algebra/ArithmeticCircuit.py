import copy
from collections import deque
import copy
from typing import Any
import networkx as nx
import matplotlib.pyplot as plt
import sympy as sp

__all__ = ['ArithmeticCircuit']


class ArithmeticCircuit:
    """
    Models an arithmetic circuit consisting of source nodes, addition gates,
    and multiplication gates with no scalar nodes.
    """

    def __init__(self, num_variables: int) -> None:
        assert num_variables >= 1

        self.in_edges: list[list[int]] = [[] for _ in range(num_variables)]  # source nodes
        self.out_edges: list[list[int]] = [[] for _ in range(num_variables)]  # source nodes
        self.node_types = [0 for _ in range(num_variables)]  # 0: source, 1: addition, 2: multiplication
        self.output_node = 0

    def copy(self) -> 'ArithmeticCircuit':
        n = self.number_of_variables()
        ret = ArithmeticCircuit(n)
        ret.in_edges = copy.deepcopy(self.in_edges)
        ret.out_edges = copy.deepcopy(self.out_edges)
        ret.node_types = self.node_types.copy()
        ret.output_node = self.output_node
        return ret

    def number_of_variables(self) -> int:
        return self.node_types.count(0)

    def number_of_nodes(self) -> int:
        return len(self.in_edges)

    # def addition_gates(self) -> list[int]:
    #     return [i for i, t in enumerate(self.node_types) if t == 1]

    # def multiplication_gates(self) -> list[int]:
    #     return [i for i, t in enumerate(self.node_types) if t == 2]

    def fingerprint_edges(self) -> list[tuple[int, int]]:
        return [(i, j) for i, t in enumerate(self.node_types) if t == 1 for j in self.in_edges[i]]

    def max_degree(self) -> int:
        degrees = {}

        for i in self._topological_ordering():
            if self.node_types[i] == 0:
                degrees[i] = 1
            elif not self.in_edges[i]:
                degrees[i] = 0
            elif self.node_types[i] == 1:
                degrees[i] = max(degrees[j] for j in self.in_edges[i])
            else:
                degrees[i] = sum(degrees[j] for j in self.in_edges[i])

        return max(degrees.values())

    # ---------------------------------------------------------------------------
    #    Modification
    # ---------------------------------------------------------------------------

    def set_output_node(self, output_node: int) -> None:
        self.output_node = output_node

    def add_edge(self, u, v) -> None:
        assert v not in self.out_edges[u]
        assert u not in self.in_edges[v]

        self.out_edges[u] += [v]
        self.in_edges[v] += [u]

    def remove_edge(self, u, v) -> None:
        assert v in self.out_edges[u]
        assert u in self.in_edges[v]

        self.out_edges[u].remove(v)
        self.in_edges[v].remove(u)

    def remove_node(self, v) -> None:
        for u in list(self.in_edges[v]):
            self.remove_edge(u, v)
        for u in list(self.out_edges[v]):
            self.remove_edge(v, u)

    def add_addition_gate(self, *in_neighbors: int, set_as_output_node=True) -> int:
        return self._add_gate(1, in_neighbors, set_as_output_node)

    def add_multiplication_gate(self, *in_neighbors: int, set_as_output_node=True) -> int:
        return self._add_gate(2, in_neighbors, set_as_output_node)

    def _add_gate(self, node_type: int, in_neighbors: tuple[int, ...], set_as_output_node: bool = True) -> int:
        assert len(in_neighbors) >= 1

        n = self.number_of_nodes()
        assert all(0 <= u < n for u in in_neighbors)

        self.in_edges += [list(in_neighbors)]
        self.out_edges += [[]]
        self.node_types += [node_type]

        for u in in_neighbors:
            self.out_edges[u] += [n]
        if set_as_output_node:
            self.set_output_node(n)
        return n

    def _topological_ordering(self):
        n = self.number_of_nodes()
        num_parents = [len(self.in_edges[i]) for i in range(n)]

        q: deque[int] = deque()
        for i in range(n):
            if num_parents[i] == 0:
                q.append(i)

        ret = []
        while q:
            u = q.popleft()
            ret += [u]
            for v in self.out_edges[u]:
                num_parents[v] -= 1
                if num_parents[v] == 0:
                    q.append(v)
        return ret

    # def find_boundary_non_additions(self, node: int) -> list[int]:
    #     assert self.node_types[node] == 1

    #     boundary = []
    #     inside = []
    #     visited = set()
    #     q: deque[int] = deque()
    #     q.append(node)

    #     while q:
    #         v = q.popleft()

    #         for u in self.in_edges[v]:
    #             if u not in visited:
    #                 visited.add(u)
    #                 if self.node_types[u] == 1:  # addition
    #                     inside += [u]
    #                     q.append(u)
    #                 else:
    #                     boundary += [u]
    #     return boundary

    # def find_boundary_non_multiplications(self, node: int, k: int) -> list[tuple[int, int]]:
    #     assert self.node_types[node] == 2

    #     boundary = []
    #     inside = []
    #     visited = {node: 1}  # stores the number of distinct paths
    #     q: deque[int] = deque()
    #     q.append(node)

    #     while q:
    #         v = q.popleft()

    #         for u in self.in_edges[v]:
    #             if u not in visited:
    #                 if self.node_types[u] == 2:  # multiplication
    #                     inside += [u]
    #                     q.append(u)
    #                 else:
    #                     boundary += [u]
    #             # count up the number of paths
    #             visited[u] = min(k + 1, visited.get(u, 0) + visited[v])

    #     return [(v, visited[v]) for v in boundary]

    def _bfs(self, start: int, reverse: bool) -> list[int]:
        ret = []
        q: deque[int] = deque()
        q.append(start)
        visited = set()

        while q:
            v = q.popleft()
            ret += [v]

            for u in self.in_edges[v] if reverse else self.out_edges[v]:
                if u not in visited:
                    visited.add(u)
                    q.append(u)
        return ret

    def remove_unreachable(self) -> None:
        n = self.number_of_nodes()
        reachable = set(self._bfs(self.output_node, reverse=True))
        for i in range(n):
            if i not in reachable:
                self.remove_node(i)

    def symbolic_evaluation(self, expand=True, fingerprint=False):  # pragma: no cover
        x = [sp.symbols(f'x_{i}') for i in range(self.number_of_variables())]

        if fingerprint:
            a = {e: sp.symbols(f'a_{i}') for i, e in enumerate(self.fingerprint_edges())}
            ret = self.evaluate(x, a)[self.output_node]
        else:
            ret = self.evaluate(x)[self.output_node]
        return sp.expand(ret) if expand else ret

    def evaluate(self, x: Any, fingerprint: dict[tuple[int, int], Any] = {}) -> dict[int, int]:
        assert self.output_node is not None, 'root must be set'
        assert len(x) == self.number_of_variables()

        values = {}

        # access nodes in topological ordering
        for i in self._topological_ordering():
            if self.node_types[i] == 0:  # source
                values[i] = x[i]
            elif not self.in_edges[i]:  # removed nodes
                pass
            elif self.node_types[i] == 1:  # addition
                u0 = self.in_edges[i][0]
                v = values[u0]
                if (i, u0) in fingerprint:
                    v = v * fingerprint[i, u0]

                for j in range(1, len(self.in_edges[i])):
                    u = self.in_edges[i][j]
                    if (i, u) in fingerprint:
                        v = v + (values[u] * fingerprint[i, u])
                    else:
                        v = v + values[u]
                values[i] = v
            else:
                v = values[self.in_edges[i][0]]

                for j in range(1, len(self.in_edges[i])):
                    v = v * values[self.in_edges[i][j]]
                values[i] = v

        return values

    # ---------------------------------------------------------------------------
    #    Utilities
    # ---------------------------------------------------------------------------
    def draw(self, width=8, height=6, rotate=False, additional_pos={}) -> None:  # pragma: no cover
        n = self.number_of_nodes()
        vertices = [i for i in range(n) if self.node_types[i] == 0 or i == self.output_node or self.out_edges[i] or self.in_edges[i]]
        G = nx.DiGraph()
        G.add_nodes_from(vertices)

        for i in vertices:
            G.add_edges_from((i, j) for j in self.out_edges[i])
        node_size = [300 if self.node_types[i] == 0 else 900 for i in vertices]
        node_color = [['#fadeeb', '#c4e1f6', '#f9c975'][self.node_types[i]] for i in vertices]
        labels = {i: [f'{i}', f'+({i})', f'x({i})'][self.node_types[i]] for i in vertices}

        for layer, nodes in enumerate(nx.topological_generations(G)):
            # `multipartite_layout` expects the layer as a node attribute, so add the
            # numeric layer value as a node attribute
            for node in nodes:
                G.nodes[node]["layer"] = layer

        # Compute the multipartite_layout using the "layer" node attribute
        pos = nx.multipartite_layout(G, subset_key="layer")
        if rotate:
            pos = {i: (-y, x) for i, (x, y) in pos.items()}
        pos.update(additional_pos)

        # fig, ax = plt.subplots()
        fig = plt.figure(figsize=(width, height))
        nx.draw_networkx(G.subgraph(vertices), pos=pos, node_size=node_size, with_labels=True, node_color=node_color, labels=labels)

        fig.tight_layout()
        plt.show()
