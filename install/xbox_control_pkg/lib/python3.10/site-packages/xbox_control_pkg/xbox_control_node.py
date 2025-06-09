import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import time
import can
# import serial

class XboxControllerReader(Node):
    def __init__(self):
        super().__init__('Xbox_controller_reader')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rpm_publisher = self.create_publisher(Float32MultiArray, '/target_rpms', 10)
        self.get_logger().info("Serial communication disabled for testing")
        self.can_bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=250000)

    def define_wheel_rpms(self, steering_angle_rad, outer_wheel_speed, inner_wheel_speed):
        rpm_outer = (outer_wheel_speed / (np.pi * 0.24)) * 60
        rpm_inner = (inner_wheel_speed / (np.pi * 0.24)) * 60

        if steering_angle_rad > 0:
            rpm_left = rpm_inner
            rpm_right = rpm_outer
        elif steering_angle_rad < 0:
            rpm_left = rpm_outer
            rpm_right = rpm_inner
        else:
            rpm_left = rpm_right = rpm_outer

        self.send_rpm_commands([rpm_left, -rpm_right, rpm_left, -rpm_right])

    def map_range(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joy_callback(self, msg: Joy):
        left_stick_x = msg.axes[0]
        deadzone = 0.1
        left_stick_x = 0.0 if abs(left_stick_x) < deadzone else left_stick_x
        steering_deg = self.map_range(left_stick_x, -1, 1, -90, 90)

        left_trigger = msg.axes[5] if len(msg.axes) > 5 else 0.0
        right_trigger = msg.axes[4] if len(msg.axes) > 4 else 0.0

        reverse_speed = self.map_range(left_trigger, -1, 1, -50, 0)
        forward_speed = self.map_range(right_trigger, -1, 1, 50, 0)

        if forward_speed != 0:
            reverse_speed = 0
        elif reverse_speed != 0:
            forward_speed = 0

        total_speed = forward_speed + reverse_speed
        outer_wheel_speed = (np.pi * 0.24 * total_speed) / 60

        steering_rad = np.radians(steering_deg)
        front_steering_angle_rad, inner_wheel_speed = self.calculate_ackermann_steering(steering_rad, outer_wheel_speed)

        self.get_logger().info(f"LT: {left_trigger}, RT: {right_trigger}")
        self.get_logger().info(
            f'[FRONT] Joystick steering: {steering_deg:.2f}° | '
            f'Ackermann angle: {np.degrees(front_steering_angle_rad):.2f}° | '
            f'Outer speed: {total_speed:.2f} | Inner speed: { (inner_wheel_speed / (np.pi * 0.24)) * 60:.2f}'
        )

    def calculate_ackermann_steering(self, steering_rad, outer_wheel_speed):
        if abs(steering_rad) < np.radians(2):
            self.send_steering_angles([0.0, 0.0, 0.0, 0.0])
            self.define_wheel_rpms(0.0, outer_wheel_speed, outer_wheel_speed)
            return 0.0, outer_wheel_speed

        track_width = 0.89
        wheelbase = 0.2815

        inner_turn_radius = wheelbase / np.tan(abs(steering_rad))
        outer_turn_radius = inner_turn_radius + track_width
        outer_wheel_angle_rad = np.arctan(wheelbase / outer_turn_radius)
        inner_wheel_speed = (inner_turn_radius / outer_turn_radius) * outer_wheel_speed

        if steering_rad < 0:
            angles = [-outer_wheel_angle_rad, -steering_rad, outer_wheel_angle_rad, steering_rad]
        else:
            angles = [steering_rad, outer_wheel_angle_rad, -steering_rad, -outer_wheel_angle_rad]

        self.send_steering_angles(angles)
        self.define_wheel_rpms(steering_rad, outer_wheel_speed, inner_wheel_speed)
        return outer_wheel_angle_rad, inner_wheel_speed

    def send_steering_angles(self, angles):
        formatted = f"D1:{angles[0]};D2:{angles[1]};D3:{angles[2]};D4:{angles[3]}"
        self.get_logger().info(f"(DEBUG) Would send angles: {formatted}")

    def send_rpm_commands(self, rpm_values):
        device_ids = [1, 2, 3, 4]
        try:
            for i, rpm in enumerate(rpm_values):
                device_id = device_ids[i]
                can_id = 0x300 | device_id

                set_value = int(rpm * 30)
                data = [
                    (set_value >> 24) & 0xFF,
                    (set_value >> 16) & 0xFF,
                    (set_value >> 8) & 0xFF,
                    set_value & 0xFF
                ]

                msg = can.Message(
                    arbitration_id=can_id,
                    data=data,
                    is_extended_id=False
                )

                self.can_bus.send(msg)
                print(f"Sent RPM {rpm} to device {device_id} (CAN ID {hex(can_id)})")

        except can.CanError:
            print("Failed to send RPMs")

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
