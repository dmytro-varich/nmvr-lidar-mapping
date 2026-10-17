"""Find collision-free paths on a two-dimensional occupancy grid."""

import heapq
import math


Cell = tuple[int, int]


def astar(
    start: Cell,
    goal: Cell,
    blocked: set[Cell],
    columns: int,
    rows: int,
) -> list[Cell] | None:
    """Return a grid path including both endpoints, or None if unreachable."""
    if columns <= 0 or rows <= 0:
        raise ValueError("Grid dimensions must be positive.")

    def is_free(cell: Cell) -> bool:
        x, y = cell
        return (
            0 <= x < columns
            and 0 <= y < rows
            and cell not in blocked
        )

    if not is_free(start) or not is_free(goal):
        return None

    # Straight and diagonal movements on the grid.
    movements = (
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1),
    )

    # Each entry stores estimated total cost, travelled cost and cell.
    frontier = [(math.dist(start, goal), 0.0, start)]
    best_cost = {start: 0.0}
    came_from: dict[Cell, Cell] = {}

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)

        # Ignore entries superseded by a cheaper route to the same cell.
        if current_cost > best_cost[current]:
            continue

        if current == goal:
            path = [goal]

            while path[-1] != start:
                path.append(came_from[path[-1]])

            path.reverse()
            return path

        x, y = current

        for dx, dy in movements:
            neighbor = (x + dx, y + dy)

            if not is_free(neighbor):
                continue

            # Prevent diagonal movement through blocked corners.
            if dx != 0 and dy != 0:
                if not is_free((x + dx, y)):
                    continue
                if not is_free((x, y + dy)):
                    continue

            new_cost = current_cost + math.hypot(dx, dy)

            if new_cost >= best_cost.get(neighbor, math.inf):
                continue

            best_cost[neighbor] = new_cost
            came_from[neighbor] = current

            estimated_total = new_cost + math.dist(neighbor, goal)
            heapq.heappush(
                frontier,
                (estimated_total, new_cost, neighbor),
            )

    return None