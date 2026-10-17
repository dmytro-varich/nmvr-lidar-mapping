"""Publish a coverage route and visualization markers in RViz."""

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from coverage_planner.coverage import (
    calculate_path_length,
    generate_lawnmower,
)


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

        width = self.get_parameter('width').value
        length = self.get_parameter('length').value
        altitude = self.get_parameter('altitude').value
        spacing = self.get_parameter('lane_spacing').value
        origin_x = self.get_parameter('origin_x').value
        origin_y = self.get_parameter('origin_y').value

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