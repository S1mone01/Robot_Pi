#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav2_msgs.msg import CostmapFilterInfo
from visualization_msgs.msg import Marker
from shapely.geometry import Polygon, Point as ShapelyPoint


class SpeedLimitPolygonPublisher(Node):
    def __init__(self):
        super().__init__('speed_limit_polygon_publisher')

        # Parameters
        self.declare_parameter('filter_info_topic', '/speed_filter_info')
        self.declare_parameter('speed_mask_topic', '/speed_limit_mask')
        self.declare_parameter('mask_frame', 'map')
        self.declare_parameter('mask_resolution', 0.05)
        self.declare_parameter('mask_width', 400)
        self.declare_parameter('mask_height', 400)
        self.declare_parameter('mask_origin_x', -10.0)
        self.declare_parameter('mask_origin_y', -10.0)
        self.declare_parameter('filter_type', 'percent')
        self.declare_parameter('base', 0.0)
        self.declare_parameter('multiplier', 1.0)
        self.declare_parameter('slow_value', 25)

        # QoS transient_local (latching)
        self.qos_tl = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # subscribe clicked points (RViz -> /clicked_point)
        self.points = []
        self.published_once = False  # ✅ flag per pubblicare solo una volta
        self.create_subscription(PointStamped, '/clicked_point', self.point_callback, 10)

        # publishers
        mask_topic = self.get_parameter('speed_mask_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('filter_info_topic').get_parameter_value().string_value
        self.mask_pub = self.create_publisher(OccupancyGrid, mask_topic, qos_profile=self.qos_tl)
        self.info_pub = self.create_publisher(CostmapFilterInfo, info_topic, qos_profile=self.qos_tl)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.get_logger().info(
            "SpeedLimitPolygonPublisher ready. "
            "Clicca almeno 4 punti in RViz (/clicked_point) per definire il poligono."
        )

    def point_callback(self, msg: PointStamped):
        if self.published_once:
            return  # ✅ ignoriamo nuovi punti se abbiamo già pubblicato

        p = Point(x=msg.point.x, y=msg.point.y, z=0.0)
        self.points.append(p)
        self.get_logger().info(f"Point added: ({p.x:.3f}, {p.y:.3f})  total={len(self.points)}")
        self._publish_marker_lines()

        # se ho abbastanza punti -> pubblica subito
        if len(self.points) >= 4:
            self._publish_mask_and_info()
            self.published_once = True
            self.get_logger().info("✅ Mask e CostmapFilterInfo pubblicati una sola volta.")

    def _publish_marker_lines(self):
        if len(self.points) < 2:
            return
        marker = Marker()
        marker.header.frame_id = self.get_parameter('mask_frame').get_parameter_value().string_value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "speed_filter"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = list(self.points)
        if len(self.points) >= 4:
            marker.points.append(self.points[0])
        self.marker_pub.publish(marker)

    def _publish_mask_and_info(self):
        resolution = self.get_parameter('mask_resolution').get_parameter_value().double_value
        width = self.get_parameter('mask_width').get_parameter_value().integer_value
        height = self.get_parameter('mask_height').get_parameter_value().integer_value
        origin_x = self.get_parameter('mask_origin_x').get_parameter_value().double_value
        origin_y = self.get_parameter('mask_origin_y').get_parameter_value().double_value
        frame = self.get_parameter('mask_frame').get_parameter_value().string_value
        slow_value = int(self.get_parameter('slow_value').get_parameter_value().integer_value)
        base = float(self.get_parameter('base').get_parameter_value().double_value)
        multiplier = float(self.get_parameter('multiplier').get_parameter_value().double_value)
        ftype = self.get_parameter('filter_type').get_parameter_value().string_value.lower()

        # shapely polygon
        shapely_pts = [(p.x, p.y) for p in self.points]
        poly = Polygon(shapely_pts)
        if not poly.is_valid or poly.area == 0.0:
            self.get_logger().warn("Poligono non valido o area=0.")
            return

        # OccupancyGrid
        grid = OccupancyGrid()
        grid.header.frame_id = frame
        grid.header.stamp = self.get_clock().now().to_msg()

        meta = MapMetaData()
        meta.resolution = resolution
        meta.width = width
        meta.height = height
        meta.origin.position.x = origin_x
        meta.origin.position.y = origin_y
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0
        grid.info = meta

        data = [100] * (width * height)
        for j in range(height):
            for i in range(width):
                cell_x = origin_x + (i + 0.5) * resolution
                cell_y = origin_y + (j + 0.5) * resolution
                if poly.contains(ShapelyPoint(cell_x, cell_y)):
                    data[j * width + i] = slow_value

        grid.data = data
        self.mask_pub.publish(grid)

        # CostmapFilterInfo
        info = CostmapFilterInfo()
        info.header.frame_id = frame
        info.header.stamp = self.get_clock().now().to_msg()
        info.type = 1 if ftype in ('percent', 'percentual', 'percentage', 'percentuale') else 2
        info.filter_mask_topic = self.get_parameter('speed_mask_topic').get_parameter_value().string_value
        info.base = base
        info.multiplier = multiplier

        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedLimitPolygonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

