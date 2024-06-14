from typing import Any

class AS_graph_generator:
    seed: Any
    n_t: Any
    n_m: Any
    n_cp: Any
    n_c: Any
    d_m: Any
    d_cp: Any
    d_c: Any
    p_m_m: Any
    p_cp_m: Any
    p_cp_cp: Any
    t_m: float
    t_cp: float
    t_c: float
    def __init__(self, n, seed) -> None: ...
    G: Any
    def t_graph(self): ...
    def add_edge(self, i, j, kind) -> None: ...
    def choose_peer_pref_attach(self, node_list): ...
    def choose_node_pref_attach(self, node_list): ...
    def add_customer(self, i, j) -> None: ...
    def add_node(self, i, kind, reg2prob, avg_deg, t_edge_prob): ...
    def add_m_peering_link(self, m, to_kind): ...
    def add_cp_peering_link(self, cp, to_kind): ...
    regions: Any
    def graph_regions(self, rn) -> None: ...
    def add_peering_links(self, from_kind, to_kind) -> None: ...
    customers: Any
    providers: Any
    nodes: Any
    def generate(self): ...

def random_internet_as_graph(n, seed: Any | None = ...): ...
