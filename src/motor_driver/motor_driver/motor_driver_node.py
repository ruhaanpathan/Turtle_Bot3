#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import lgpio
    GPIO_OK = True
except ImportError:
    GPIO_OK = False

_chip = None


def gpio_init(pins):
    global _chip
    if not GPIO_OK:
        return

    if _chip is None:
        # On Raspberry Pi 5, the 40-pin GPIO header is provided by the RP1 chip
        # which exposes itself as gpiochip4 (not 0 like older Pis).
        try:
            _chip = lgpio.gpiochip_open(4)
        except Exception:
            # Fallback to 0 if not on an RPi 5
            _chip = lgpio.gpiochip_open(0)

    for p in pins:
        try:
            lgpio.gpio_claim_output(_chip, p, 0)
        except:
            pass


def gpio_write(pin, val):
    if GPIO_OK and _chip is not None:
        try:
            lgpio.gpio_write(_chip, pin, val)
        except:
            pass


def pwm_change(pin, duty, freq=1000):
    if GPIO_OK and _chip is not None:
        try:
            lgpio.tx_pwm(_chip, pin, freq, duty)
        except:
            pass


class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')

        # Parameters (same pins)
        self.declare_parameter('left_en', 12)
        self.declare_parameter('left_in1', 20)
        self.declare_parameter('left_in2', 21)
        self.declare_parameter('right_en', 13)
        self.declare_parameter('right_in1', 16)
        self.declare_parameter('right_in2', 26)
        self.declare_parameter('wheel_base', 0.15)
        self.declare_parameter('max_speed', 0.5)

        self.l_en = self.get_parameter('left_en').value
        self.l_in1 = self.get_parameter('left_in1').value
        self.l_in2 = self.get_parameter('left_in2').value
        self.r_en = self.get_parameter('right_en').value
        self.r_in1 = self.get_parameter('right_in1').value
        self.r_in2 = self.get_parameter('right_in2').value
        self.wb = self.get_parameter('wheel_base').value
        self.ms = self.get_parameter('max_speed').value

        gpio_init([self.l_in1, self.l_in2, self.l_en, self.r_in1, self.r_in2, self.r_en])

        self.lv = 0.0
        self.av = 0.0
        self.last_cmd_t = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(0.1, self.watchdog)

    def set_motor(self, en, in1, in2, speed):
        duty = min(abs(speed) * 100.0, 100.0)

        if speed > 0.01:
            gpio_write(in1, 1)
            gpio_write(in2, 0)
        elif speed < -0.01:
            gpio_write(in1, 0)
            gpio_write(in2, 1)
        else:
            gpio_write(in1, 0)
            gpio_write(in2, 0)
            duty = 0.0

        pwm_change(en, duty)

    def cmd_cb(self, msg):
        self.last_cmd_t = self.get_clock().now()
        self.lv = msg.linear.x
        self.av = msg.angular.z

        ls = max(-1.0, min(1.0, (self.lv - self.av * self.wb / 2.0) / self.ms))
        rs = max(-1.0, min(1.0, (self.lv + self.av * self.wb / 2.0) / self.ms))

        self.set_motor(self.l_en, self.l_in1, self.l_in2, ls)
        self.set_motor(self.r_en, self.r_in1, self.r_in2, rs)

    def watchdog(self):
        elapsed = (self.get_clock().now() - self.last_cmd_t).nanoseconds * 1e-9
        if elapsed > 0.5:
            self.set_motor(self.l_en, self.l_in1, self.l_in2, 0.0)
            self.set_motor(self.r_en, self.r_in1, self.r_in2, 0.0)

    def destroy_node(self):
        self.set_motor(self.l_en, self.l_in1, self.l_in2, 0.0)
        self.set_motor(self.r_en, self.r_in1, self.r_in2, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    self.get_logger().info("Motor Driver Node Started 🚀")
    main()
