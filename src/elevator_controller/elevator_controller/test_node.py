import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose



class PrintHello(Node):
    def __init__(self):
        super().__init__("print_hello")
        print("--헬로노드 생성됨--")
        self.timer = self.create_timer(1.0,self.timer_callback)
       
    def timer_callback(self):
         self.turtles_pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.subscriber,
            10
        )

    def subscriber(self, data):
        print(data)



def main():
    rclpy.init()
    print_node = PrintHello()

    rclpy.spin(print_node)

    print_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
