from typing import Any

def core_number(G): ...
def find_cores(G): ...
def k_core(G, k: Any | None = ..., core_number: Any | None = ...): ...
def k_shell(G, k: Any | None = ..., core_number: Any | None = ...): ...
def k_crust(G, k: Any | None = ..., core_number: Any | None = ...): ...
def k_corona(G, k, core_number: Any | None = ...): ...
def k_truss(G, k): ...
def onion_layers(G): ...