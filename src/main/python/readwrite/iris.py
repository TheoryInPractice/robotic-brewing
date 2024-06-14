import networkx as nx


def load_iris(path_prefix: str) -> nx.Graph:
    graph = nx.Graph()

    # Vertice File
    vertex_file = f'{path_prefix}_vertex'
    with open(vertex_file, 'r') as vf:
        for line in vf:
            parts = line.strip().split()
            index = int(parts[0])
            colors = list(map(int, parts[3:]))

            graph.add_node(index, color=colors)

    # Edge File
    edge_file = f'{path_prefix}_edge'
    with open(edge_file, 'r') as ef:
        for line in ef:
            parts = line.strip().split()
            source = int(parts[0])
            target = int(parts[1])
            weight = float(parts[6])

            checked = parts[2] == '1'
            valid = parts[3] == '1'

            graph.add_edge(source, target, weight=weight, checked=checked, valid=valid)

    return graph


def save_iris(G: nx.Graph, path_prefix: str) -> None:
    raise NotImplementedError
