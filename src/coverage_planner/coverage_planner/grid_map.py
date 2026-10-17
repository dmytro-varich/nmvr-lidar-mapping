"""Convert rectangular obstacles into blocked planning-grid cells."""

import math
from dataclasses import dataclass


Cell = tuple[int, int]
Rectangle = tuple[float, float, float, float]


@dataclass
class PlanningGrid:
    """Store grid geometry and cells forbidden for the drone center."""

    origin_x: float
    origin_y: float
    resolution: float
    columns: int
    rows: int
    blocked: set[Cell]

    def cell_center(self, cell: Cell) -> tuple[float, float]:
        """Convert a cell index to its center coordinates in meters."""
        column, row = cell

        x = self.origin_x + (column + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution

        return x, y

    def world_to_cell(self, x: float, y: float) -> Cell:
        """Convert map coordinates to cell indices without clipping."""
        column = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)

        return column, row


def build_grid(
    origin_x: float,
    origin_y: float,
    width: float,
    length: float,
    resolution: float,
    obstacles: list[Rectangle],
    clearance: float,
) -> PlanningGrid:
    """Block cells intersecting obstacles expanded by a safety clearance."""
    values = (origin_x, origin_y, width, length, resolution, clearance)

    if not all(math.isfinite(value) for value in values):
        raise ValueError("Grid parameters must be finite.")

    if width <= 0 or length <= 0 or resolution <= 0:
        raise ValueError("Grid dimensions and resolution must be positive.")

    if clearance < 0:
        raise ValueError("Clearance must not be negative.")

    expanded_obstacles = []

    for xmin, ymin, xmax, ymax in obstacles:
        if not all(
            math.isfinite(value)
            for value in (xmin, ymin, xmax, ymax)
        ):
            raise ValueError("Obstacle coordinates must be finite.")

        if xmin >= xmax or ymin >= ymax:
            raise ValueError("Obstacle minimums must be below maximums.")

        expanded_obstacles.append((
            xmin - clearance,
            ymin - clearance,
            xmax + clearance,
            ymax + clearance,
        ))

    columns = math.ceil(width / resolution)
    rows = math.ceil(length / resolution)
    blocked = set()

    for row in range(rows):
        for column in range(columns):
            cell_xmin = origin_x + column * resolution
            cell_ymin = origin_y + row * resolution
            cell_xmax = origin_x + (column + 1) * resolution
            cell_ymax = origin_y + (row + 1) * resolution

            # Block partial cells extending beyond the planning area.
            outside = (
                cell_xmax > origin_x + width + 1e-9
                or cell_ymax > origin_y + length + 1e-9
            )

            intersects = any(
                cell_xmin <= xmax
                and cell_xmax >= xmin
                and cell_ymin <= ymax
                and cell_ymax >= ymin
                for xmin, ymin, xmax, ymax in expanded_obstacles
            )

            if outside or intersects:
                blocked.add((column, row))

    return PlanningGrid(
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
        columns=columns,
        rows=rows,
        blocked=blocked,
    )