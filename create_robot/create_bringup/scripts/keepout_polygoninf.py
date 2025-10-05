#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from visualization_msgs.msg import Marker
import numpy as np
from shapely.geometry import Point as ShapelyPoint, Polygon, box

class KeepoutPolygonPublisher(Node):
    def __init__(self):
        super().__init__('keepout_polygon_publisher')
        self.points = []
        # <-- MODIFICA: Aggiunta variabile di stato per gestire l'arresto
        self.is_shutting_down = False

        self.declare_parameter('map.resolution', 0.05)
        self.declare_parameter('map.width', 400)
        self.declare_parameter('map.height', 400)
        self.declare_parameter('map.origin_x', -10.0)
        self.declare_parameter('map.origin_y', -10.0)
        
        qos_profile_transient_local = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.clicked_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        self.mask_pub = self.create_publisher(
            OccupancyGrid, 
            '/keepout_mask', 
            qos_profile=qos_profile_transient_local
        )
        self.info_pub = self.create_publisher(
            CostmapFilterInfo, 
            '/costmap_filter_info', 
            qos_profile=qos_profile_transient_local
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )

        self.get_logger().info("Clicca almeno 4 punti in RViz per definire un poligono keepout.")

    def point_callback(self, msg: PointStamped):
        # <-- MODIFICA: Ignora i click se il nodo si sta già spegnendo
        if self.is_shutting_down:
            self.get_logger().warn("Il nodo si sta spegnendo, ulteriori punti verranno ignorati.")
            return
            
        x, y = msg.point.x, msg.point.y
        self.points.append(Point(x=x, y=y, z=0.0))
        self.get_logger().info(f"Aggiunto punto: ({x}, {y})")

        if len(self.points) >= 2:
            self.publish_lines()

        if len(self.points) >= 4:
            self.publish_polygon()

    def publish_lines(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "keepout_lines"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = self.points
        
        if len(self.points) >= 4:
            marker.points.append(self.points[0])

        self.marker_pub.publish(marker)
        self.get_logger().info("Linee di anteprima pubblicate ✅")

    def publish_polygon(self):
        shapely_points = [(p.x, p.y) for p in self.points]
        polygon = Polygon(shapely_points)
        if not polygon.is_valid:
            self.get_logger().warn("Poligono non valido, riprova con punti diversi.")
            return

        resolution = self.get_parameter('map.resolution').get_parameter_value().double_value
        width = self.get_parameter('map.width').get_parameter_value().integer_value
        height = self.get_parameter('map.height').get_parameter_value().integer_value
        origin_x = self.get_parameter('map.origin_x').get_parameter_value().double_value
        origin_y = self.get_parameter('map.origin_y').get_parameter_value().double_value
        
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.data = [-1] * (width * height)

        self.get_logger().info("Generazione maschera accurata in corso...")
        for j in range(height):
            for i in range(width):
                cell_x1 = origin_x + i * resolution
                cell_y1 = origin_y + j * resolution
                cell_x2 = cell_x1 + resolution
                cell_y2 = cell_y1 + resolution
                
                cell_box = box(cell_x1, cell_y1, cell_x2, cell_y2)
                
                if polygon.intersects(cell_box):
                    grid.data[j * width + i] = 100

        self.mask_pub.publish(grid)
        self.get_logger().info("Keepout mask (accurata) pubblicata ✅")

        info = CostmapFilterInfo()
        info.header.frame_id = "map"
        info.type = 0
        info.base = 0.0
        info.multiplier = 1.0
        info.filter_mask_topic = "/keepout_mask"

        self.info_pub.publish(info)
        self.get_logger().info("CostmapFilterInfo pubblicato ✅")
        
        # <-- MODIFICA PRINCIPALE: Usa un timer per spegnere il nodo dopo un ritardo -->
        self.get_logger().info("Operazione completata. Il nodo si spegnerà tra 2 secondi per garantire la consegna dei messaggi.")
        self.is_shutting_down = True
        self.create_timer(2.0, self.shutdown_callback)

    # <-- MODIFICA: Aggiunta la funzione di callback per lo spegnimento -->
    def shutdown_callback(self):
        self.get_logger().info("Arresto del nodo in corso.")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeepoutPolygonPublisher()
    rclpy.spin(node)
    # Quando destroy_node() viene chiamato dal timer, spin() termina e il programma prosegue qui.
    rclpy.shutdown()

if __name__ == "__main__":
    main()
