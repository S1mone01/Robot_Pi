#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty
import math

class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('global_localization_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service client
        self.global_localization_client = self.create_client(Empty, '/reinitialize_global_localization')
        
        # Subscriber a /amcl_pose
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        # Variabili di stato
        self.state = "waiting_service"
        self.move_start_time = None
        self.rotation_start_time = None
        self.current_rotation_angle = 0.0
        self.target_rotation = 2 * math.pi  # 360 gradi in radianti
        
        # Parametri di movimento
        self.linear_speed = 0.15
        self.angular_speed = 0.8
        self.forward_duration = 2.0
        self.pause_duration = 1.0
        
        # Timer principale
        self.timer = self.create_timer(0.1, self.execute_sequence_step)

        # Parametri di soglia per considerarsi localizzato
        self.cov_x_thresh = 0.05
        self.cov_y_thresh = 0.05
        self.cov_yaw_thresh = 0.05
        self.localized = False
        
        self.get_logger().info("Nodo Reinitialize Global Localization avviato")

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Controlla la covarianza per capire se il robot è localizzato"""
        cov = msg.pose.covariance
        cov_x = cov[0]
        cov_y = cov[7]
        cov_yaw = cov[35]

        if (cov_x < self.cov_x_thresh and
            cov_y < self.cov_y_thresh and
            cov_yaw < self.cov_yaw_thresh):
            if not self.localized:
                self.get_logger().info(
                    f"Robot localizzato! cov_x={cov_x:.4f}, cov_y={cov_y:.4f}, cov_yaw={cov_yaw:.4f}"
                )
            self.localized = True
        else:
            self.localized = False

    def execute_sequence_step(self):
        if self.localized and self.state != "completed":
            # Se AMCL è sicuro → ferma subito la sequenza
            self.stop_robot()
            self.state = "completed"
            self.get_logger().info("Localizzazione confermata, sequenza interrotta")
            return

        if self.state == "waiting_service":
            if not self.global_localization_client.wait_for_service(timeout_sec=0.1):
                if not hasattr(self, 'service_waiting_logged'):
                    self.get_logger().info("Aspetto il servizio /reinitialize_global_localization...")
                    self.service_waiting_logged = True
                return
            else:
                self.get_logger().info("Servizio disponibile")
                self.state = "calling_global_localization"

        elif self.state == "calling_global_localization":
            self.call_global_localization()
            self.state = "waiting_after_service"
            self.service_call_time = self.get_clock().now()

        elif self.state == "waiting_after_service":
            elapsed = (self.get_clock().now() - self.service_call_time).nanoseconds / 1e9
            if elapsed >= self.pause_duration:
                self.get_logger().info("Inizio primo giro...")
                self.state = "rotating_1"
                self.rotation_start_time = self.get_clock().now()
                self.current_rotation_angle = 0.0

        elif self.state == "rotating_1":
            self.perform_rotation()
            if self.current_rotation_angle >= self.target_rotation:
                self.stop_robot()
                self.get_logger().info("Primo giro completato")
                self.state = "waiting_before_forward"
                self.wait_start_time = self.get_clock().now()

        elif self.state == "waiting_before_forward":
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            if elapsed >= self.pause_duration:
                self.get_logger().info("Inizio movimento avanti + rotazione...")
                self.state = "moving_forward_rotating"
                self.move_start_time = self.get_clock().now()

        elif self.state == "moving_forward_rotating":
            elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
            if elapsed < self.forward_duration:
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info("Movimento avanti+rotazione completato")
                self.state = "waiting_before_rotation_2"
                self.wait_start_time = self.get_clock().now()

        elif self.state == "waiting_before_rotation_2":
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            if elapsed >= self.pause_duration:
                self.get_logger().info("Inizio secondo giro...")
                self.state = "rotating_2"
                self.rotation_start_time = self.get_clock().now()
                self.current_rotation_angle = 0.0

        elif self.state == "rotating_2":
            self.perform_rotation()
            if self.current_rotation_angle >= self.target_rotation:
                self.stop_robot()
                self.get_logger().info("Secondo giro completato")
                self.state = "completed"

        elif self.state == "completed":
            if self.localized:
                self.get_logger().info("Sequenza di localizzazione completata con successo ✅")
                self.timer.cancel()
            else:
                # Se non localizzato, riprova da capo
                self.get_logger().warn("Localizzazione ancora incerta, riprovo...")
                self.state = "calling_global_localization"

    def call_global_localization(self):
        request = Empty.Request()
        future = self.global_localization_client.call_async(request)
        future.add_done_callback(self.global_localization_callback)
        self.get_logger().info("Chiamata al servizio global localization inviata")

    def global_localization_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Global localization eseguita")
        except Exception as e:
            self.get_logger().error(f"Errore global localization: {e}")

    def perform_rotation(self):
        current_time = self.get_clock().now()
        if self.rotation_start_time is None:
            self.rotation_start_time = current_time
        elapsed_time = (current_time - self.rotation_start_time).nanoseconds / 1e9
        self.current_rotation_angle = self.angular_speed * elapsed_time
        
        if self.current_rotation_angle < self.target_rotation:
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interruzione ricevuta")
    finally:
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

