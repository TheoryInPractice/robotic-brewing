from collections.abc import Generator
from typing import Any

def bridges(G, root: Any | None = ...) -> Generator[Any, None, None]: ...
def has_bridges(G, root: Any | None = ...): ...
def local_bridges(
    G, with_span: bool = ..., weight: Any | None = ...
) -> Generator[Any, None, Any]: ...