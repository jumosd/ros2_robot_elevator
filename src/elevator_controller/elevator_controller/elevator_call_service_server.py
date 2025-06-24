import rclpy
from rclpy.node import Node
from elevator_controller_interfaces.srv import CallElevator
from Robot import Robot


class ElevatorCallServiceServer(Node):
    def __init__(self):
        super().__init__(node_name="elevator_call_service_server")
        self.create_service(
            CallElevator,
            'call_elevator',
            self.call_elevator
        )

    def call_elevaotr(self,request , response):
        """
        엘리베이터를 부르는 코드작성
        """



        self.get_logger().info(
            f"호출 요청: line_id={request.line_id}, source_floor={request.source_floor}, direction={request.direction}"
        )




          # 예시 응답 (실제 로직에 맞게 수정)
        response.el_id = "EL001"
        response.mode = "auto"
        response.current_floor = request.source_floor
        response.direction = "none"
        response.door_status = "close"
        response.registed_up_hall_call = ""
        response.registed_dn_hall_call = ""
        response.registed_car_call = ""
        response.message_id = "msg001"
        response.thing_info = request.thing_info
        return response


def main():
    rclpy.init()
    node = ElevatorCallServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()