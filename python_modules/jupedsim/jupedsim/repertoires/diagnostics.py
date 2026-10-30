# SPDX-License-Identifier: LGPL-3.0-or-later

from collections import Counter
from collections.abc import Mapping


def anchor_cycles(anchors: Mapping[int, int | None]) -> list[tuple[int, ...]]:
    """Find cycles in the predecessor relation.

    Anchors pointing to an id that is not a key of ``anchors`` end a
    chain and are not cycles. An agent anchored to itself is a cycle of
    length one.

    Args:
        anchors: Anchor (predecessor) of each agent, ``None`` if absent.

    Returns:
        Each cycle once, rotated to start at its smallest id, sorted.
    """
    done: set[int] = set()
    cycles = []
    for start in sorted(anchors):
        path = _walk(anchors, start, done)
        cycle = _cycle_in(anchors, path)
        if cycle:
            cycles.append(cycle)
        done.update(path)
    return sorted(cycles)


def _walk(
    anchors: Mapping[int, int | None], start: int, done: set[int]
) -> list[int]:
    path: list[int] = []
    seen: set[int] = set()
    current = start
    while current in anchors and current not in done and current not in seen:
        path.append(current)
        seen.add(current)
        current = anchors[current]
    return path


def _cycle_in(
    anchors: Mapping[int, int | None], path: list[int]
) -> tuple[int, ...]:
    if not path or anchors[path[-1]] not in path:
        return ()
    cycle = path[path.index(anchors[path[-1]]) :]
    first = cycle.index(min(cycle))
    return tuple(cycle[first:] + cycle[:first])


def anchor_structure(anchors: Mapping[int, int | None]) -> dict[str, int]:
    """Count anchored agents, unanchored agents and shared predecessors.

    Args:
        anchors: Anchor (predecessor) of each agent, ``None`` if absent.

    Returns:
        ``anchored``, ``unanchored`` and ``shared`` (predecessors chosen
        by at least two agents).
    """
    chosen = Counter(a for a in anchors.values() if a is not None)
    return {
        "anchored": sum(chosen.values()),
        "unanchored": len(anchors) - sum(chosen.values()),
        "shared": sum(1 for n in chosen.values() if n >= 2),
    }
