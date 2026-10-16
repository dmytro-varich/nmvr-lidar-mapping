"""Generate a coverage route for a rectangular area."""

import math


def generate_lawnmower(
    width: float,
    length: float,
    altitude: float,
    lane_spacing: float,
) -> list[tuple[float, float, float]]:
    """Return route waypoints; all distances are specified in meters."""
    values = (width, length, altitude, lane_spacing)
    if any(not math.isfinite(value) or value <= 0 for value in values):
        raise ValueError("All parameters must be finite numbers greater than zero.")

    # Number of intervals between sweep lanes.
    intervals = math.ceil(length / lane_spacing)

    # Reduce spacing if needed so lanes reach both boundaries.
    actual_spacing = length / intervals

    waypoints = []

    for lane in range(intervals + 1):
        y = lane * actual_spacing

        # Alternate the direction of travel on successive lanes.
        if lane % 2 == 0:
            start_x, end_x = 0.0, width
        else:
            start_x, end_x = width, 0.0

        waypoints.append((start_x, y, altitude))
        waypoints.append((end_x, y, altitude))

    return waypoints