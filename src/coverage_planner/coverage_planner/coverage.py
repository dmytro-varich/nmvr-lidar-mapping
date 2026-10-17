"""Generate a coverage route for a rectangular area."""

import math


def generate_lawnmower(
    width: float,
    length: float,
    altitude: float,
    lane_spacing: float,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    """Return route waypoints in meters in the map frame."""
    dimensions = (width, length, altitude, lane_spacing)
    if any(not math.isfinite(value) or value <= 0 for value in dimensions):
        raise ValueError("Dimensions, altitude and spacing must be finite and positive.")

    if not all(math.isfinite(value) for value in (origin_x, origin_y)):
        raise ValueError("Origin coordinates must be finite.")

    # Keep the actual lane spacing at or below the requested maximum.
    intervals = math.ceil(length / lane_spacing)
    actual_spacing = length / intervals

    waypoints = []

    for lane in range(intervals + 1):
        y = origin_y + lane * actual_spacing

        # Alternate the direction of travel on successive lanes.
        if lane % 2 == 0:
            start_x, end_x = origin_x, origin_x + width
        else:
            start_x, end_x = origin_x + width, origin_x

        waypoints.append((start_x, y, altitude))
        waypoints.append((end_x, y, altitude))

    return waypoints


def calculate_path_length(
    waypoints: list[tuple[float, float, float]],
) -> float:
    """Return the total length of straight segments between waypoints."""
    return sum(
        math.dist(start, end)
        for start, end in zip(waypoints, waypoints[1:])
    )