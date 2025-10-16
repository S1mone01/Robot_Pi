#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from rclpy.action.client import GoalStatus
import time # Importa il modulo time

class DelayedShutdownDockingController(Node):
    """
    Questo nodo naviga verso un punto. Se la navigazione ha successo,
    invia un comando di docking, attende 20 secondi e solo allora si arresta.
    """
    def __init__(self):
        super().__init__('delayed_shutdown_docking_controller')

        # Publisher per il comando di docking
        self.dock_publisher = self.create_publisher(Empty, '/dock', 10)

        # Action client per la navigazione
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info("In attesa del server di navigazione 'navigate_to_pose'...")
        self.nav_action_client.wait_for_server()
        self.get_logger().info("Server di navigazione trovato.")

        self.send_navigation_goal()

    def send_navigation_goal(self):
        """
        Prepara e invia il goal di navigazione.
        """
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        # Coordinate del punto di pre-docking
        pose.pose.position.x = -0.1128992251777689
        pose.pose.position.y = -0.003261919421412198
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = -0.1747426538419712
        pose.pose.orientation.w = 0.9846141401220404

        goal_msg.pose = pose

        self.get_logger().info("Invio del goal di navigazione al punto di pre-docking...")
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback che gestisce la risposta del server di navigazione al goal inviato.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Il goal di navigazione è stato RIFIUTATO.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal di navigazione accettato. In attesa del risultato finale...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """
        Callback eseguita al termine della navigazione.
        Avvia il docking solo in caso di successo.
        """
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigazione completata con SUCCESSO.")
            self.get_logger().info("Attendo 2 secondi prima di avviare il docking...")
            time.sleep(2.0)
            self.initiate_docking()
        else:
            self.get_logger().error(f"Navigazione FALLITA con stato: {status}. Operazione terminata.")
            rclpy.shutdown()

    def initiate_docking(self):
        """
        Invia il comando di docking, attende 20 secondi e poi termina il nodo.
        """
        self.get_logger().info("Invio del comando di docking...")
        self.dock_publisher.publish(Empty())
        self.get_logger().info("Comando di docking inviato. Il nodo si arresterà tra 20 secondi.")
        
        # --- MODIFICA CHIAVE ---
        # Attende 20 secondi prima di procedere all'arresto
        time.sleep(20.0)
        
        self.get_logger().info("Tempo scaduto. Arresto del nodo.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    docking_controller = DelayedShutdownDockingController()

    try:
        rclpy.spin(docking_controller)
    except KeyboardInterrupt:
        docking_controller.get_logger().info('Interruzione da tastiera rilevata.')
    finally:
        if rclpy.ok() and docking_controller.destroy_node():
             docking_controller.destroy_node()

if __name__ == '__main__':
    main()
