import networkx as nx
from typing import TextIO, Optional

__all__ = [
    'read_pace',
    'load_pace',
    'write_pace',
    'save_pace',
]


def read_pace(input: TextIO) -> tuple[nx.Graph, Optional[int], Optional[int], Optional[int]]:
    G = nx.Graph()
    given_n: Optional[int] = None
    given_m: Optional[int] = None
    given_c: Optional[int] = None
    given_s: Optional[int] = None
    given_t: Optional[int] = None
    given_k: Optional[int] = None
    all_colors: set[int] = set()

    for line in input.readlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith('c'):
            continue  # ignore comments

        tokens = line.split()

        if tokens[0] == 'p':
            assert tokens[1] == 'ip', f'unexpected problem description: {tokens[1]}'

            given_n = int(tokens[2])
            given_m = int(tokens[3])
            given_c = int(tokens[4])
            given_s = int(tokens[5]) if len(tokens) >= 6 else None
            given_t = int(tokens[6]) if len(tokens) >= 7 else None
            given_k = int(tokens[7]) if len(tokens) >= 8 else None

        elif tokens[0] == 'v':
            v = int(tokens[1])
            colors = list(map(int, tokens[2:]))
            assert len(set(colors)) == len(colors), 'colors must be distinct'
            all_colors |= set(colors)
            G.add_node(v, color=colors)

        elif tokens[0] == 'e':
            assert len(tokens) == 4

            u = int(tokens[1])
            v = int(tokens[2])
            w = int(tokens[3]) if tokens[3].isdigit() else float(tokens[3])
            G.add_edge(u, v, weight=w)
        else:
            assert False, f'unexpected line: {line}'  # pragma: no cover

    assert given_n == G.number_of_nodes(), 'inconsistent n'
    assert given_m == G.number_of_edges(), 'inconsistent m'
    assert given_c == len(all_colors), 'inconsistent c'
    return G, given_s, given_t, given_k


def load_pace(path: str) -> tuple[nx.Graph, Optional[int], Optional[int], Optional[int]]:
    with open(path) as f:
        return read_pace(f)


def write_pace(output: TextIO, G: nx.Graph, s: Optional[int] = None, t: Optional[int] = None, k: Optional[int] = None, comments: list[str] = []) -> None:
    n = G.number_of_nodes()
    m = G.number_of_edges()
    assert all(isinstance(v, int) for v in G.nodes()), 'labels must be integers'

    c = len({v for vs in nx.get_node_attributes(G, 'color').values() for v in vs})

    for comment in comments:
        output.write(f'c {comment}\n')

    # p-line
    p_params = [n, m, c]
    if s is not None:
        p_params += [s]
        if t is not None:
            p_params += [t]
            if k is not None:
                p_params += [k]

    output.write(f'p ip {" ".join(map(str, p_params))}\n')

    # vertices
    for v in G.nodes():
        v_params = [v] + G.nodes[v]['color']
        output.write(f'v {" ".join(map(str, v_params))}\n')

    # edges
    for u, v in G.edges():
        output.write(f'e {u} {v} {G.edges[u, v]["weight"]}\n')


def save_pace(path: str, G: nx.Graph, s: Optional[int] = None, t: Optional[int] = None, k: Optional[int] = None, comments: list[str] = []) -> None:
    with open(path, 'w') as f:  # pragma: no cover
        write_pace(f, G, s, t, k, comments)
