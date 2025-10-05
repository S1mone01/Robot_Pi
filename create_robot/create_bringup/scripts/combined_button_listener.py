#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, String
from create_msgs.msg import MotorSetpoint
import subprocess
import time

class CombinedButtonListener(Node):
    def __init__(self):
        super().__init__('combined_button_listener')
        
        # Subscriber per il pulsante dock
        self.dock_subscription = self.create_subscription(
            Empty,
            '/dock_button',
            self.dock_callback,
            10
        )
        
        # Subscriber per il pulsante clean
        self.clean_subscription = self.create_subscription(
            Empty,
            '/clean_button',
            self.clean_callback,
            10
        )
        
        # Subscriber per il pulsante A
        self.button_a_subscription = self.create_subscription(
            String,
            '/button_a_topic',
            self.button_a_callback,
            10
        )
        
        # Subscriber per comandi testuali (es. {"data": "docking"})
        self.command_subscription = self.create_subscription(
            String,
            '/command_topic',
            self.command_callback,
            10
        )
        
        # Publishers per i LED
        self.power_led_pub = self.create_publisher(Bool, '/power_led', 10)
        self.dock_led_pub = self.create_publisher(Bool, '/dock_led', 10)
        
        # Publishers per i motori
        self.vacuum_motor_pub = self.create_publisher(MotorSetpoint, '/vacuum_motor', 10)
        self.side_brush_pub = self.create_publisher(MotorSetpoint, '/side_brush_motor', 10)
        self.main_brush_pub = self.create_publisher(MotorSetpoint, '/main_brush_motor', 10)
        
        # Timer per i cooldown
        self.dock_last_press_time = 0
        self.clean_last_press_time = 0
        self.button_a_last_press_time = 0
        self.dock_cooldown_period = 20.0  # 3 secondi per dock
        self.clean_cooldown_period = 1.0  # 1 secondo per clean
        self.button_a_cooldown_period = 1.0  # 1 secondo per button A
        
        # Stato dei motori
        self.motors_on = False
        
        self.get_logger().info("Combined button listener initialized - Dock: 3s cooldown, Clean: 1s cooldown, Button A: 1s cooldown")

    def dock_callback(self, msg):
        current_time = time.time()
        
        if current_time - self.dock_last_press_time < self.dock_cooldown_period:
            remaining_time = self.dock_cooldown_period - (current_time - self.dock_last_press_time)
            self.get_logger().info(f"Dock button press ignored - cooldown active for {remaining_time:.1f}s more")
            return
        
        self.dock_last_press_time = current_time
        
        self.get_logger().info("Dock button pressed! Starting docking launch and turning on LED...")
        
        self.turn_on_leds()
        
        try:
            subprocess.Popen(["ros2", "launch", "create_bringup", "docking.py"])
            self.get_logger().info("Docking launch started successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to start docking launch: {e}")

    def clean_callback(self, msg):
        current_time = time.time()
        
        if current_time - self.clean_last_press_time < self.clean_cooldown_period:
            remaining_time = self.clean_cooldown_period - (current_time - self.clean_last_press_time)
            self.get_logger().info(f"Clean button press ignored - cooldown active for {remaining_time:.1f}s more")
            return
        
        self.clean_last_press_time = current_time
        
        if self.motors_on:
            self.turn_off_motors()
            self.motors_on = False
        else:
            self.turn_on_motors()
            self.motors_on = True

    def button_a_callback(self, msg):
        current_time = time.time()
        
        if current_time - self.button_a_last_press_time < self.button_a_cooldown_period:
            remaining_time = self.button_a_cooldown_period - (current_time - self.button_a_last_press_time)
            self.get_logger().info(f"Button A press ignored - cooldown active for {remaining_time:.1f}s more")
            return
        
        self.button_a_last_press_time = current_time
        
        if msg.data == "A premuto":
            if self.motors_on:
                self.get_logger().info("Button A pressed! Stopping cleaning motors...")
                self.turn_off_motors()
                self.motors_on = False
            else:
                self.get_logger().info("Button A pressed! Starting cleaning motors...")
                self.turn_on_motors()
                self.motors_on = True

    def command_callback(self, msg):
        if msg.data.lower() == "docking":
            self.get_logger().info('Received docking command via String message')
            from std_msgs.msg import Empty
            self.dock_callback(Empty())

    def turn_on_leds(self):
        try:
            led_msg = Bool()
            led_msg.data = True
            self.power_led_pub.publish(led_msg)
            self.dock_led_pub.publish(led_msg)
            self.get_logger().info("LEDs turned on")
        except Exception as e:
            self.get_logger().error(f"Failed to turn on LEDs: {e}")

    def turn_on_motors(self):
        try:
            motor_msg = MotorSetpoint()
            motor_msg.duty_cycle = 1.0
            self.vacuum_motor_pub.publish(motor_msg)
            self.side_brush_pub.publish(motor_msg)
            self.main_brush_pub.publish(motor_msg)
            self.get_logger().info("Motors turned ON (duty_cycle: 1.0)")
        except Exception as e:
            self.get_logger().error(f"Failed to turn on motors: {e}")

    def turn_off_motors(self):
        try:
            motor_msg = MotorSetpoint()
            motor_msg.duty_cycle = 0.0
            self.vacuum_motor_pub.publish(motor_msg)
            self.side_brush_pub.publish(motor_msg)
            self.main_brush_pub.publish(motor_msg)
            self.get_logger().info("Motors turned OFF (duty_cycle: 0.0)")
        except Exception as e:
            self.get_logger().error(f"Failed to turn off motors: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CombinedButtonListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.turn_off_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

