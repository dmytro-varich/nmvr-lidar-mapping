"""Publish a coverage route and visualization markers in RViz."""

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from coverage_planner.coverage import (
    calculate_path_length,
    generate_lawnmower,
)
from rclpy.parameter import Parameter
from coverage_planner.grid_map import build_grid

class CoveragePlannerNode(Node):
    """Generate a rectangular coverage route and publish it periodically."""

    def __init__(self):
        super().__init__('coverage_planner')

        # Parameters are read once when the node starts.
        self.declare_parameter('width', 20.0)
        self.declare_parameter('length', 10.0)
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('lane_spacing', 4.0)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('grid_resolution', 0.5)
        self.declare_parameter('clearance', 0.5)

        # Flat list: xmin, ymin, xmax, ymax for each obstacle.
        self.declare_parameter(
            'obstacles',
            Parameter.Type.DOUBLE_ARRAY,
        )

        width = self.get_parameter('width').value
        length = self.get_parameter('length').value
        altitude = self.get_parameter('altitude').value
        spacing = self.get_parameter('lane_spacing').value
        origin_x = self.get_parameter('origin_x').value
        origin_y = self.get_parameter('origin_y').value
        resolution = self.get_parameter('grid_resolution').value
        clearance = self.get_parameter('clearance').value
        obstacle_values = self.get_parameter('obstacles').value or []

        if len(obstacle_values) % 4 != 0:
            raise ValueError(
                "Each obstacle requires xmin, ymin, xmax and ymax."
            )

        obstacles = [
            tuple(obstacle_values[index:index + 4])
            for index in range(0, len(obstacle_values), 4)
        ]

        self.grid = build_grid(
            origin_x,
            origin_y,
            width,
            length,
            resolution,
            obstacles,
            clearance,
        )

        self.grid_altitude = altitude

        self.waypoints = generate_lawnmower(
            width,
            length,
            altitude,
            spacing,
            origin_x,
            origin_y,
        )

        # Show the survey boundary at the route altitude.
        # Repeat the first corner to close the rectangle.
        self.boundary_points = [
            (origin_x, origin_y, altitude),
            (origin_x + width, origin_y, altitude),
            (origin_x + width, origin_y + length, altitude),
            (origin_x, origin_y + length, altitude),
            (origin_x, origin_y, altitude),
        ]

        self.publisher = self.create_publisher(
            Path,
            '/coverage/path',
            10,
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/coverage/markers',
            10,
        )
        self.grid_publisher = self.create_publisher(
            OccupancyGrid,
            '/coverage/planning_grid',
            10,
        )

        self.grid_message = OccupancyGrid()
        self.grid_message.header.frame_id = 'map'

        info = self.grid_message.info
        info.resolution = float(self.grid.resolution)
        info.width = self.grid.columns
        info.height = self.grid.rows
        info.origin.position.x = float(self.grid.origin_x)
        info.origin.position.y = float(self.grid.origin_y)
        info.origin.position.z = float(self.grid_altitude)
        info.origin.orientation.w = 1.0

        self.grid_message.data = [
            100 if (column, row) in self.grid.blocked else 0
            for row in range(self.grid.rows)
            for column in range(self.grid.columns)
        ]

        # Republish so subscribers can receive data when started later.
        self.timer = self.create_timer(1.0, self.publish_path)

        lane_count = len(self.waypoints) // 2
        actual_spacing = length / (lane_count - 1)
        path_length = calculate_path_length(self.waypoints)

        self.get_logger().info(
            f'Route ready: {len(self.waypoints)} waypoints, '
            f'{lane_count} lanes, '
            f'spacing={actual_spacing:.2f} m, '
            f'length={path_length:.2f} m'
        )

    def publish_path(self):
        """Publish the route and markers with the same timestamp."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y, z in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            pose.header.stamp = path.header.stamp

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)

            # Placeholder orientation; drone heading is not planned yet.
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.publisher.publish(path)
        self.publish_markers(path.header.stamp)
        self.grid_message.header.stamp = path.header.stamp
        self.grid_publisher.publish(self.grid_message)import sys

    def publish_markers(self, stamp):
        """Publish the survey boundary and the first route waypoint."""
        boundary = Marker()
        boundary.header.frame_id = 'map'
        boundary.header.stamp = stamp
        boundary.ns = 'coverage'
        boundary.id = 0
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.pose.orientation.w = 1.0

        # LINE_STRIP uses scale.x as the line width in meters.
        boundary.scale.x = 0.08

        # Blue boundary, fully opaque.
        boundary.color.r = 0.1
        boundary.color.g = 0.6
        boundary.color.b = 1.0
        boundary.color.a = 1.0

        for x, y, z in self.boundary_points:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
            boundary.points.append(point)

        start = Marker()
        start.header.frame_id = 'map'
        start.header.stamp = stamp
        start.ns = 'coverage'
        start.id = 1
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.pose.orientation.w = 1.0

        # The first waypoint is the beginning of the coverage route.
        x, y, z = self.waypoints[0]
        start.pose.position.x = float(x)
        start.pose.position.y = float(y)
        start.pose.position.z = float(z)

        # Sphere scale specifies its diameter along each axis in meters.
        start.scale.x = 0.5
        start.scale.y = 0.5
        start.scale.z = 0.5

        # Orange start marker, fully opaque.
        start.color.r = 1.0
        start.color.g = 0.3
        start.color.b = 0.0
        start.color.a = 1.0

        markers = MarkerArray()
        markers.markers = [boundary, start]
        self.marker_publisher.publish(markers)


def main(args=None):
    """Initialize ROS, run the node and release resources on exit."""
    rclpy.init(args=args)
    node = None

    try:
        node = CoveragePlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()