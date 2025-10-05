#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy


class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')

        # Publisher sul topic /button_a_topic
        self.publisher_ = self.create_publisher(String, '/button_a_topic', 10)

        # Subscriber al topic /joy (dal joy_node)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.last_state_a = 0  # per rilevare i cambi di stato

        self.get_logger().info("ButtonPublisher inizializzato, in ascolto su /joy")

    def joy_callback(self, msg: Joy):
        # Pulsante A su Xbox One è in genere index 0 (dipende dal driver, ma quasi sempre così)
        button_a = msg.buttons[0]

        # Pubblica solo quando viene premuto (evita spam continuo)
        if button_a == 1 and self.last_state_a == 0:
            out_msg = String()
            out_msg.data = "A premuto"
            self.publisher_.publish(out_msg)
            self.get_logger().info("Pubblicato: A premuto")

        self.last_state_a = button_a


def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

