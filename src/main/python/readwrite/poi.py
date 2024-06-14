from typing import TextIO

__all__ = [
    'read_poi_position',
    'load_poi_position',
    'read_poi_list',
    'load_poi_list',
]


def read_poi_position(input: TextIO) -> dict[int, tuple[float, float, float]]:
    ret = {}
    for i, line in enumerate(input.readlines()):
        x, y, z = line.strip().split()
        ret[i] = (float(x), float(y), float(z))
    return ret


def load_poi_position(path: str) -> dict[int, tuple[float, float, float]]:
    with open(path) as f:
        return read_poi_position(f)


def read_poi_list(input: TextIO) -> list[int]:
    return [int(line) for line in input.readlines()]


def load_poi_list(path: str) -> list[int]:
    with open(path) as f:
        return read_poi_list(f)
