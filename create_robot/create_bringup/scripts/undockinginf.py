#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

class UndockAndReverseNode(Node):
    def __init__(self):
        super().__init__('undock_and_reverse_node')
        self.undock_pub = self.create_publisher(Empty, '/undock', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Variabili di stato
        self.state = "waiting_subscriber"
        self.undock_count = 0
        self.move_start_time = None
        
        # Timer per gestire la sequenza
        self.timer = self.create_timer(0.1, self.execute_sequence_step)
        
        self.get_logger().info("🤖 Nodo avviato - inizio sequenza undock e reverse")

    def execute_sequence_step(self):
        if self.state == "waiting_subscriber":
            # 1. Aspetta subscriber per /undock
            if self.undock_pub.get_subscription_count() == 0:
                if not hasattr(self, 'waiting_logged'):
                    self.get_logger().info("⏳ Aspetto che ci sia almeno 1 subscriber su /undock...")
                    self.waiting_logged = True
                return
            else:
                self.get_logger().info("✅ Subscriber trovato su /undock")
                self.state = "undocking"
                self.undock_timer = self.create_timer(0.5, self.send_undock_command)

        elif self.state == "undocking":
            # Gestito dal timer separato
            pass

        elif self.state == "waiting_before_move":
            # 3. Attendi 1 secondo prima del movimento
            if not hasattr(self, 'wait_start_time'):
                self.get_logger().info("⏳ Attendo 1 secondo prima di muovermi...")
                self.wait_start_time = self.get_clock().now()
            
            if (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9 >= 1.0:
                self.get_logger().info("⬅️ Inizio movimento indietro per 2 secondi...")
                self.state = "moving_backward"
                self.move_start_time = self.get_clock().now()

        elif self.state == "moving_backward":
            # 4. Movimento indietro per 2 secondi
            if self.move_start_time is None:
                self.move_start_time = self.get_clock().now()
            
            elapsed_time = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
            
            if elapsed_time < 2.0:
                # Continua a muoverti indietro
                twist = Twist()
                twist.linear.x = -0.1  # Velocità negativa = indietro
                self.cmd_vel_pub.publish(twist)
            else:
                # Stop movimento
                twist = Twist()
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("🛑 Movimento terminato")
                self.state = "publishing_pose"

        elif self.state == "publishing_pose":
            # 6. Invia posizione stimata ad AMCL
            self.publish_estimated_pose()
            self.state = "completed"

        elif self.state == "completed":
            self.get_logger().info("✅ Sequenza completata - chiudo il nodo")
            self.timer.cancel()
            rclpy.shutdown()

    def send_undock_command(self):
        if self.undock_count < 2:
            msg = Empty()
            self.undock_pub.publish(msg)
            self.undock_count += 1
            self.get_logger().info(f"📤 Comando undock inviato ({self.undock_count}/2)")
        else:
            self.undock_timer.cancel()
            self.state = "waiting_before_move"

    def publish_estimated_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # Deve essere 'map' per AMCL
        
        # 🔹 Coordinate del dock nella mappa
        pose_msg.pose.pose.position.x = -10.556
        pose_msg.pose.pose.position.y = -3.795
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientamento (quaternion)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = -0.391
        pose_msg.pose.pose.orientation.w = 0.921
        
        # Imposta la covarianza per indicare la certezza della posizione
        # Diagonale principale: [x, y, z, roll, pitch, yaw]
        
        self.initialpose_pub.publish(pose_msg)
        self.get_logger().info(f"📍 Posizione stimata pubblicata su /initialpose: "
                               f"x={pose_msg.pose.pose.position.x:.3f}, y={pose_msg.pose.pose.position.y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = UndockAndReverseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Assicurati di fermare il robot prima di chiudere
        twist = Twist()
        twist.linear.x = 0.0
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
