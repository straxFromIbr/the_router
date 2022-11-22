import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class TrapeController(Node):
    def __init__(self):
        super().__init__("trape_controller")
        self.subscription = self.create_subscription(
            msg_type=String,
            topic="control_topic",
            callback=self.listener_callback,
            qos_profile=10,
        )
        self.subscription  # prevent unused variable warning

        # Setup MOTOR!!!!!
        self.pub_cmdvel = self.create_publisher(Twist, "cmd_vel", 1)
        self._client_motor_power = self.create_client(SetBool, "motor_power")
        self._motor_request(True)
        self._is_controlling = False

        self._speed = 0.4
        self._turn_val = math.pi * 0.5
        self._step = 20
        self._interval = 0.05  # [sec]
        self._delta_speed = self._speed / self._step

        self.current_speed = 0
        self._minimum_abs_speed = 0.07

        self.nb_gear = 4

    def _set_speed(self, target_speed):
        target_speed = float(target_speed)

        delta_speed = (
            self._delta_speed
            if (target_speed - self.current_speed) > 0
            else -1 * self._delta_speed
        )

        cmdvel = Twist()
        while abs(target_speed - self.current_speed) > self._delta_speed * 1.5:
            self.current_speed += delta_speed
            if abs(target_speed) < self._minimum_abs_speed:
                self.current_speed += delta_speed
            cmdvel.linear.x = self.current_speed
            self.pub_cmdvel.publish(cmdvel)

            time.sleep(self._interval)

        self.current_speed = target_speed
        cmdvel.linear.x = self.current_speed
        self.pub_cmdvel.publish(cmdvel)

    def _turn(self, direction):
        cmdvel = Twist()
        cmdvel.linear.x = float(0)
        if direction == "right":
            cmdvel.angular.z = -1 * self._turn_val
        elif direction == "left":
            cmdvel.angular.z = self._turn_val
        self.pub_cmdvel.publish(cmdvel)

    def immd_stop(self):
        cmdvel = Twist()
        target_speed = 0.0
        self.current_speed = target_speed
        cmdvel.linear.x = self.current_speed
        self.pub_cmdvel.publish(cmdvel)

    def listener_callback(self, msg):
        data = json.loads(msg.data)
        direction = data["direction"]
        if direction == "immd_stop":
            self.immd_stop()
            return

        target_speed = self._speed * float(data["speed"]) / self.nb_gear
        if direction == "stop":
            self._set_speed(0.0)
            return

        if direction in ("right", "left"):
            self._turn(direction)
            return

        if data["direction"] == "forward":
            direction = 1
        elif data["direction"] == "backward":
            direction = -1
        else:
            return

        self._set_speed(direction * target_speed)

    def _motor_request(self, request_data=False):
        request = SetBool.Request()
        request.data = request_data
        self._client_motor_power.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    trape_controller = TrapeController()

    rclpy.spin(trape_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trape_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
