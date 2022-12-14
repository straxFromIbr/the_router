import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")

        self.keyboard_subscription = self.create_subscription(
            msg_type=String,
            topic="keyboard_topic",
            callback=self.mode_toggle_callback,
            qos_profile=10,
        )

        self.publisher = self.create_publisher(String, "control_topic", 10)

        self.auto_mode = False
        self.manual_mode = True

        self.first_manual = True
        self.start_time = time.time()

        self.logfile = Path("./control_log.csv")
        with open(self.logfile, "w") as f:
            f.write("")

        self.log_data = []

    def clear_log(self):
        with open(self.logfile, "w") as f:
            f.write("")
        self.log_data = []

    def load_csv(self, reverse=False):
        print("load csv")
        with open(self.logfile) as f:
            log_data = f.readlines()
        if len(log_data) == 0:
            return

        log_data = [data.strip().split(",") for data in log_data]
        log_data = [(float(data[0]), data[1], int(data[2])) for data in log_data]
        start_time = log_data[0][0]
        log_data = [(data[0] - start_time, data[1], data[2]) for data in log_data]

        if not reverse:
            self.log_data = log_data
            return

        last_time = log_data[-1][0]

        log_data.reverse()
        log_data.append((0, "stop", log_data[-1][2]))
        new_list = []
        for index in range(len(log_data) - 1):
            t = last_time - log_data[index][0] - start_time
            c = self.rev_control(log_data[index + 1][1])
            s = log_data[index + 1][2]
            new_list.append((t, c, s))
        self.log_data = new_list

    @staticmethod
    def rev_control(control):
        if control == "forward":
            return "backward"
        if control == "backward":
            return "forward"

        if control == "right":
            return "left"
        if control == "left":
            return "right"

        return control

    def auto_control(self):
        if len(self.log_data) == 0:
            return

        # Read CSV File to generate control parmas
        start_time = time.time()

        current_data = self.log_data.pop(0)
        while True:
            time.sleep(0.01)
            if time.time() - start_time >= current_data[0]:

                direction = current_data[1]
                speed = current_data[2]

                data = {
                    "direction": direction,
                    "speed": speed,
                }

                msg = String()
                msg.data = json.dumps(data)
                self.publisher.publish(msg)

                try:
                    current_data = self.log_data.pop(0)
                except IndexError:
                    break
        data = {
            "direction": "stop",
            "speed": 1,
        }

        msg = String()
        msg.data = json.dumps(data)
        self.publisher.publish(msg)

    def mode_toggle_callback(self, kbd_msg):
        # kbd_msg is JSON expression
        data = json.loads(kbd_msg.data)
        # Toggle mode Auto or Manual
        if data["auto_mode"] == True:
            reverse = not data["forward_auto"] and data["backward_auto"]
            # Load LOG CSV File once
            if self.manual_mode:
                self.load_csv(reverse)
            # Set flags
            self.auto_mode = True
            self.manual_mode = False
            self.auto_control()
            self.first_manual = False

            return

        # Logging
        if data["manual_mode"] == True:
            if self.first_manual:
                # pass through kbd_msg to trape controller as another topic
                self.log(kbd_msg)
            self.auto_mode = False
            self.manual_mode = True
            self.publisher.publish(kbd_msg)
            return

    def log(self, kbd_msg: String) -> None:
        # Write on CSV File
        # Ex. Timestamp, Direction, Speed
        data = json.loads(kbd_msg.data)

        time_stamp = time.time() - self.start_time
        time_stamp_str = str(time_stamp)
        direction = data["direction"]
        speed = data["speed"]
        log_message = f"{time_stamp_str},{direction},{speed}\n"
        self.write_to_file(log_message)

    def write_to_file(self, str_msg):
        with open(self.logfile, "a") as f:
            f.write(str_msg)


def main(args=None):
    rclpy.init(args=args)

    main_controller = MainController()

    rclpy.spin(main_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
