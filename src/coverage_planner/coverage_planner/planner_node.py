"""Publish a coverage route for visualization in RViz."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from coverage_planner.coverage import generate_lawnmower


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        # Parameters can be configured at startup without editing the code.
        self.declare_parameter('width', 20.0)
        self.declare_parameter('length', 10.0)
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('lane_spacing', 4.0)

        width = self.get_parameter('width').value
        length = self.get_parameter('length').value
        altitude = self.get_parameter('altitude').value
        spacing = self.get_parameter('lane_spacing').value

        self.waypoints = generate_lawnmower(
            width, length, altitude, spacing
        )

        self.publisher = self.create_publisher(
            Path, '/coverage/path', 10
        )

        # Republish periodically so RViz can receive the route when opened later.
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(
            f'Route ready: {len(self.waypoints)} waypoints'
        )

    def publish_path(self):
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

            # Identity quaternion represents zero rotation.
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.publisher.publish(path)


def main(args=None):
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