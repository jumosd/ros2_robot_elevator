import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class ElevatorStatusPublihserNode(Node):
    def __init__(self):
        super().__init__(node_name="elevator_status_publisher_node")



def main():

    rclpy.init()

