#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        # Publisher per il docking
        self.dock_pub = self.create_publisher(Empty, '/dock', 10)

        # Action client NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Quando il client è pronto, inviamo la goal
        self._action_client.wait_for_server()
        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = -10.556
        pose.pose.position.y = -3.795
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = -0.391
        pose.pose.orientation.w = 0.921

        goal_msg.pose = pose

        self.get_logger().info('Invio goal di navigazione...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rifiutato :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accettato, in esecuzione...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: distanza rimanente = {feedback.distance_remaining:.2f} m")

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # ABORTED
            self.get_logger().info('Goal abortito.')
            self.publish_dock()
        elif status == 3:  # SUCCEEDED
            self.get_logger().info('Goal raggiunto! Procedo con il docking.')
            self.publish_dock()
        else:
            self.get_logger().info(f'Goal finito con stato: {status}')

        rclpy.shutdown()

    def publish_dock(self):
        msg = Empty()
        self.dock_pub.publish(msg)
        self.get_logger().info('Comando docking pubblicato')

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

