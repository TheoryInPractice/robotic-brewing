from typing import Optional
import networkx as nx
from algebra import ArithmeticCircuit, MultilinearDetector
from algorithm.base import BaseSolver
from random import Random
import bisect
import datetime


class AlgebraicSolver(BaseSolver):
    def __init__(self, G: nx.Graph, rand: Random, num_iterations: int = 30) -> None:
        super().__init__(G)
        self.rand = rand
        self.num_iterations = num_iterations
        self.cert: Optional[ArithmeticCircuit] = None
        self.node_ids: dict[tuple[int, int, int], int] = {}

    def _create_circiut(self, G: nx.Graph, s: int, k: int, c: int, max_distance: int) -> Optional[ArithmeticCircuit]:
        node_ids = {}
        C = ArithmeticCircuit(c)
        prv: set[tuple[int, int]] = set()
        nxt: set[tuple[int, int]] = set()
        for h in range(1, k + 1):
            for v in G:
                if v == s:
                    continue

                vcolor = G.nodes[v]['color'][0]
                if h == 1:
                    d = G.edges[s, v]['weight']
                    if d > max_distance:
                        continue

                    node_id = C.add_addition_gate(vcolor)
                    node_ids[v, h, d] = node_id
                    nxt.add((v, d))
                else:
                    for u, d in prv:
                        if v == u:
                            continue
                        u_id = node_ids[u, h - 1, d]
                        dist = G.edges[u, v]['weight']
                        if d + dist > max_distance:
                            continue

                        mul_id = C.add_multiplication_gate(u_id, vcolor)
                        node_id = node_ids.get((v, h, d + dist), -1)
                        if node_id < 0:
                            node_id = C.add_addition_gate(mul_id)
                            node_ids[v, h, d + dist] = node_id
                        else:
                            C.add_edge(mul_id, node_id)
                        # print(f'edge added: v={v}, u={u}, d={d}, d+={dist}, {u_id},{vcolor} -> {mul_id}; {mul_id} -> {node_id}')
                        nxt.add((v, d + dist))
            prv = nxt.copy()
            nxt.clear()

        # Create root.
        in_nbrs = []
        for v in G:
            if v == s:
                continue
            for d in range(G.edges[v, s]['weight'], max_distance + 1):
                t = (v, k, d - G.edges[v, s]['weight'])
                if t in node_ids:
                    in_nbrs += [node_ids[t]]

        if not in_nbrs:
            return None
        root = C.add_addition_gate(*in_nbrs)
        node_ids[s, k + 1, max_distance] = root
        self.node_ids = node_ids
        return C

    def _test_total_distance(self, start_vertex: int, k: int, c: int, total_distance: int) -> bool:
        # create a circuit
        C = self._create_circiut(self.G, start_vertex, k, c, total_distance)
        if C is None:
            return False

        # run detector
        detector = MultilinearDetector(C, k)
        # print(datetime.datetime.now(), f'Running MLD: guess={total_distance}')
        ret = detector.run(self.rand, self.num_iterations)
        # print(datetime.datetime.now(), f'Running MLD: guess={total_distance}, result={ret}')
        return ret

    def _find_certificate(self, start_vertex: int, k: int, c: int, total_distance: int) -> bool:
        max_num_retries = 5

        for _ in range(max_num_retries):
            # print(datetime.datetime.now(), f'Running MLD: recovering solution: {total_distance}')
            C = self._create_circiut(self.G, start_vertex, k, c, total_distance)
            assert C is not None
            detector = MultilinearDetector(C, k)
            self.cert = detector.find_certificate(self.rand)
            key_nodes = [
                v
                for v
                in self.cert._bfs(self.cert.output_node, reverse=True)
                if self.cert.node_types[v] == 1 and v != self.cert.output_node
            ]

            node_ids_inv = {v: k for k, v in self.node_ids.items()}
            critical_path = [node_ids_inv[v][0] for v in reversed(key_nodes) if v != self.cert.output_node]
            critical_path = [start_vertex] + critical_path + [start_vertex]
            self.set_critical_path(critical_path)

            # verify path
            if self.solution_weight is not None and self.solution_colors is not None and self.solution_weight <= total_distance and len(self.solution_colors) >= c:
                assert self.solution_weight == total_distance
                # print(datetime.datetime.now(), f'Running MLD: solution verified: {total_distance}')
                return True

            # print('solution recovery failed')
        return False

    def solve(self, k: int, start_vertex: int, closed_walk: bool) -> bool:
        assert start_vertex in self.G

        c = self.number_of_colors
        if k != c:
            raise NotImplementedError  # TODO
        if not closed_walk:
            raise NotImplementedError  # TODO
        assert all(isinstance(w, int) for w in nx.get_edge_attributes(self.G, 'weight').values())

        # corner case
        if k <= len(self.G.nodes[start_vertex]['color']):
            self.set_solution([start_vertex])
            return True

        self.remove_unreachable_vertices(start_vertex)

        # preprocessing
        self.split_colors()
        self.create_transitive_closure(create_metric=True)
        self.remove_colorless_vertices(start_vertex)

        # compute the max distance
        mst_cost = sum(self.G.edges[e]['weight'] for e in nx.minimum_spanning_tree(self.G).edges)

        # binary search
        ub = mst_cost * 2 + 1
        ret = bisect.bisect_left(range(ub), True, key=lambda x: self._test_total_distance(start_vertex, k, c, x))

        if ret < ub:
            # recover solution
            recovered = self._find_certificate(start_vertex, k, c, ret)
            return recovered
        else:
            return False
