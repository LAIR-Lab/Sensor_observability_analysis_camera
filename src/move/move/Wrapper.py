from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from std_msgs.msg import String

#this remap needs to be reworked fr depending on the json file for lerobot, this is just a reference for now.
remap={
    0: [5, 95],
    1: [-180, 80],
    2: [-90, 100],
    3: [-90, 100],
    4: [-80, 90],
    5: [-130, 130]
}

class Moveit(Node):
    def __init__(self):
        super().__init__(
            "so101_moveit"
        )

        self.create_subscription(
            String,
            "/Pos/send_action",
            self.callback,
            10
        )

        self.config = SO101FollowerConfig(
            port = '/dev/ttyACM0', # or '/dev/ttyACM1' depending on your system
            id = "wtf"
            )

        self.follower = SO101Follower(self.config)

        self.follower.connect(calibrate=False)
        self.follower.send_action({'shoulder_pan.pos': 0, 'shoulder_lift.pos': 0, 'elbow_flex.pos': 0, 'wrist_flex.pos': 0, 'wrist_roll.pos': 0, 'gripper.pos': 96})

    def callback(self, msg):
        data = {key: (value) for key, value in msg.data.items()}
        self.get_logger().info(
            f"SOA : {data}"
        )

def remap_to_percent(value, min_val, max_val):
    return ((value - min_val) / (max_val - min_val)) * 100

def percent_to_range(value, min_val, max_val):
    return (value / 100) * (max_val - min_val) + min_val

def main():
    rclpy.init()
    node = Moveit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# {'shoulder_pan.pos': 0, 'shoulder_lift.pos': 0, 'elbow_flex.pos': 0, 'wrist_flex.pos': 0, 'wrist_roll.pos': 0, 'gripper.pos': 96}


'''
Used of remapping:-
 'Gripper.pos' : [5:95],
 'wrist_roll.pos' : [-180:80],
 'wrist_flex.pos' : [-90:100],
 'elbow_flex.pos' : [-90:100],
 'shoulder_lift.pos' : [-80:90],
 'shoulder_pan.pos' : [-130:130]
''' 