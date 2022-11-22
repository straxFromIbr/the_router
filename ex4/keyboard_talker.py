import json
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GetKeyEvent:
    def __init__(self):
        pass

    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class KeyBoardTalker(Node):
    def __init__(self):
        super().__init__("keyboard_talker")
        self.publisher_ = self.create_publisher(String, "keyboard_topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self._getch = GetKeyEvent()

        # Manual Control Parameters
        self.current_gear = 1
        self.max_speed = 4
        self.min_speed = 1
        self.current_direction = None

        # mode booleans
        self.auto_mode = False
        self.manual_mode = True
        self.forward_auto = False
        self.backward_auto = True

    def timer_callback(self):
        msg = String()
        keypress = self._getch()

        # Direction
        if keypress == "w":
            self.current_direction = "forward"
        elif keypress == "s":
            self.current_direction = "backward"
        elif keypress == "a":
            self.current_direction = "left"
        elif keypress == "d":
            self.current_direction = "right"

        # Immediate Stop
        elif keypress == "q":
            self.current_direction = "immd_stop"

        # Speed
        elif keypress == "j":
            # Speed Up
            self.current_gear = min(self.current_gear + 1, self.max_speed)
        elif keypress == "h":
            # Speed Down
            self.current_gear = max(self.current_gear - 1, self.min_speed)

        elif keypress == "o":
            # Set Manual Mode
            self.manual_mode = True
            self.auto_mode = False

        elif keypress == "p":
            # Set Auto Mode
            self.manual_mode = False
            self.auto_mode = True

        elif keypress == "u":
            # Forward Mode
            self.forward_auto = True
            self.backward_auto = False

        elif keypress == "i":
            # Backward Mode
            self.forward_auto = False
            self.backward_auto = True

        else:
            self.current_direction = "stop"

        if self.current_direction is None:
            return

        data = {
            "direction": str(self.current_direction),
            "speed": self.current_gear,
            "manual_mode": self.manual_mode,
            "auto_mode": self.auto_mode,
            "forward_auto": self.forward_auto,
            "backward_auto": self.backward_auto,
        }

        msg.data = json.dumps(data)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        if keypress == "q":
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)

    keyboard_talker = KeyBoardTalker()

    rclpy.spin(keyboard_talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
