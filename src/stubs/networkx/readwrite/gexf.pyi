from collections.abc import Generator
from typing import Any

def write_gexf(
    G, path, encoding: str = ..., prettyprint: bool = ..., version: str = ...
) -> None: ...
def generate_gexf(
    G, encoding: str = ..., prettyprint: bool = ..., version: str = ...
) -> None: ...
def read_gexf(
    path, node_type: Any | None = ..., relabel: bool = ..., version: str = ...
): ...

class GEXF:
    versions: Any
    d: Any
    xml_type: Any
    python_type: Any
    def construct_types(self) -> None: ...
    convert_bool: Any
    NS_GEXF: Any
    NS_VIZ: Any
    NS_XSI: Any
    SCHEMALOCATION: Any
    VERSION: Any
    version: Any
    def set_version(self, version) -> None: ...

class GEXFWriter(GEXF):
    prettyprint: Any
    encoding: Any
    xml: Any
    edge_id: Any
    attr_id: Any
    all_edge_ids: Any
    attr: Any
    def __init__(
        self,
        graph: Any | None = ...,
        encoding: str = ...,
        prettyprint: bool = ...,
        version: str = ...,
    ) -> None: ...
    graph_element: Any
    def add_graph(self, G) -> None: ...
    def add_nodes(self, G, graph_element) -> None: ...
    def add_edges(self, G, graph_element) -> Generator[None, None, None]: ...
    def add_attributes(self, node_or_edge, xml_obj, data, default): ...
    def get_attr_id(self, title, attr_type, edge_or_node, default, mode): ...
    def add_viz(self, element, node_data): ...
    def add_parents(self, node_element, node_data): ...
    def add_slices(self, node_or_edge_element, node_or_edge_data): ...
    def add_spells(self, node_or_edge_element, node_or_edge_data): ...
    def alter_graph_mode_timeformat(self, start_or_end) -> None: ...
    def write(self, fh) -> None: ...
    def indent(self, elem, level: int = ...) -> None: ...

class GEXFReader(GEXF):
    node_type: Any
    simple_graph: bool
    def __init__(self, node_type: Any | None = ..., version: str = ...) -> None: ...
    xml: Any
    def __call__(self, stream): ...
    timeformat: Any
    def make_graph(self, graph_xml): ...
    def add_node(self, G, node_xml, node_attr, node_pid: Any | None = ...) -> None: ...
    def add_start_end(self, data, xml): ...
    def add_viz(self, data, node_xml): ...
    def add_parents(self, data, node_xml): ...
    def add_slices(self, data, node_or_edge_xml): ...
    def add_spells(self, data, node_or_edge_xml): ...
    def add_edge(self, G, edge_element, edge_attr) -> None: ...
    def decode_attr_elements(self, gexf_keys, obj_xml): ...
    def find_gexf_attributes(self, attributes_element): ...

def relabel_gexf_graph(G): ...