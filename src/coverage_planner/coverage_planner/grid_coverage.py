"""Build a sweep route through free grid cells using A* connections."""

import math

from coverage_planner.grid_map import PlanningGrid
from coverage_planner.grid_planner import astar


def generate_grid_coverage(
    grid: PlanningGrid,
    altitude: float,
    lane_spacing: float,
) -> tuple[list[tuple[float, float, float]], int, float]:
    """Return waypoints, active sweep-row count and maximum sweep-row gap."""
    if not math.isfinite(altitude) or altitude <= 0:
        raise ValueError("Altitude must be finite and positive.")

    if not math.isfinite(lane_spacing) or lane_spacing <= 0:
        raise ValueError("Lane spacing must be finite and positive.")

    if lane_spacing < grid.resolution:
        raise ValueError(
            "Lane spacing must be at least the grid resolution."
        )

    # Round down so the requested maximum spacing is not exceeded.
    row_step = math.floor(lane_spacing / grid.resolution)

    sweep_rows = list(range(0, grid.rows, row_step))

    # Include the last grid row even when the spacing is not divisible.
    if sweep_rows[-1] != grid.rows - 1:
        sweep_rows.append(grid.rows - 1)

    route_cells = []
    active_rows = 0

    for lane_index, row in enumerate(sweep_rows):
        columns = range(grid.columns)

        if lane_index % 2 == 1:
            columns = reversed(columns)

        targets = [
            (column, row)
            for column in columns
            if (column, row) not in grid.blocked
        ]

        if not targets:
            continue

        active_rows += 1

        for target in targets:
            if not route_cells:
                route_cells.append(target)
                continue

            current = route_cells[-1]

            if current == target:
                continue

            # Adjacent cardinal cells can be connected directly.
            distance_in_cells = (
                abs(target[0] - current[0])
                + abs(target[1] - current[1])
            )

            if distance_in_cells == 1:
                route_cells.append(target)
                continue

            connection = astar(
                start=current,
                goal=target,
                blocked=grid.blocked,
                columns=grid.columns,
                rows=grid.rows,
            )

            if connection is None:
                raise ValueError(
                    f"No free path from cell {current} to cell {target}. "
                    "The selected sweep cells cannot form one route."
                )

            # The first connection cell is already present in the route.
            route_cells.extend(connection[1:])

    if not route_cells:
        raise ValueError("No free cells exist on the selected sweep rows.")

    waypoints = []

    for cell in route_cells:
        x, y = grid.cell_center(cell)
        waypoints.append((x, y, altitude))

    max_row_gap = max(
        (
            (second - first) * grid.resolution
            for first, second in zip(sweep_rows, sweep_rows[1:])
        ),
        default=0.0,
    )

    return waypoints, active_rows, max_row_gap