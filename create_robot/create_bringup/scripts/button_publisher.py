#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from create_msgs.msg import MotorSetpoint

class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')
        
        # Publishers per i motori
        self.vacuum_motor_pub = self.create_publisher(MotorSetpoint, '/vacuum_motor', 10)
        self.side_brush_pub = self.create_publisher(MotorSetpoint, '/side_brush_motor', 10)
        self.main_brush_pub = self.create_publisher(MotorSetpoint, '/main_brush_motor', 10)
        
        # Subscriber al topic /joy (dal joy_node)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        self.last_state_a = 0  # per rilevare i cambi di stato
        self.motors_on = False  # stato attuale dei motori
        
        self.get_logger().info("ButtonPublisher inizializzato - Pulsante A controlla motori direttamente")
    
    def joy_callback(self, msg: Joy):
        # Pulsante A su Xbox One è in genere index 0
        button_a = msg.buttons[0]
        
        # Pubblica solo quando viene premuto (transizione da 0 a 1)
        if button_a == 1 and self.last_state_a == 0:
            if self.motors_on:
                self.get_logger().info("Pulsante A premuto! Spengo i motori...")
                self.turn_off_motors()
                self.motors_on = False
            else:
                self.get_logger().info("Pulsante A premuto! Accendo i motori...")
                self.turn_on_motors()
                self.motors_on = True
        
        self.last_state_a = button_a
    
    def turn_on_motors(self):
        try:
            motor_msg = MotorSetpoint()
            motor_msg.duty_cycle = 1.0
            self.vacuum_motor_pub.publish(motor_msg)
            self.side_brush_pub.publish(motor_msg)
            self.main_brush_pub.publish(motor_msg)
            self.get_logger().info("Motori ACCESI (duty_cycle: 1.0)")
        except Exception as e:
            self.get_logger().error(f"Errore nell'accensione dei motori: {e}")
    
    def turn_off_motors(self):
        try:
            motor_msg = MotorSetpoint()
            motor_msg.duty_cycle = 0.0
            self.vacuum_motor_pub.publish(motor_msg)
            self.side_brush_pub.publish(motor_msg)
            self.main_brush_pub.publish(motor_msg)
            self.get_logger().info("Motori SPENTI (duty_cycle: 0.0)")
        except Exception as e:
            self.get_logger().error(f"Errore nello spegnimento dei motori: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Assicurati che i motori siano spenti quando chiudi il nodo
        node.turn_off_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
