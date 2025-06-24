import requests
from Authorization import Authorization

class Robot:
    """
    엘리베이터를 불러오는 로봇 입니다.
    elevator_call을 이용해서 엘리베이터를 호출할 수 있습니다.
    robot_status_sender를 이용해서 로봇의 상태를 전송할 수 있습니다.

    """
    API_URL = "https://apis.hyundaielevator.com"

    def __init__(self,
                 API_KEY_STR : str ,
                 API_SECRET_HEX_STR: str, 
                 current_floor:str = "1",
                 destination_floor:str = "1",
                 arrival_floor:str = "1"):
        """
        param
         API_KEY_STR: API 키 문자열
         API_SECRET_HEX_STR: API 비밀 키 (16진수 문자열)
         current_floor: 현재 층 (기본값: 1)
         destination_floor: 목적 층 (기본값: 1)
         arrival_floor: 도착 층 (기본값: 1)
        """
        self._current_floor = current_floor
        self._destination_floor = destination_floor
        self._arrival_floor = arrival_floor
        self._API_KEY_STR = API_KEY_STR
        self._API_SECRET_HEX_STR = API_SECRET_HEX_STR
        self._auth = Authorization(self._API_KEY_STR, self._API_SECRET_HEX_STR)

    def elevator_call(self, line_id: str, destination_floor: str):
        """
        엘리베이터를 호출하는 메소드입니다.
        :param line_id: 호출 라인 아이디
        :param call_type: 요청 콜 종류 (hall: 홀콜 요청 / robot: 로봇 전용)
        :param direction: 이동 방향 (up: 상행 / down: 하행)
        """
        timestamp = self._auth.generate_timestamp()
        nonce = self._auth.generate_nonce()
        signature_hex_str = self._auth.generate_signature(nonce, timestamp)
        url = f"{self.API_URL}/api/v1/el/call/thing"

        self._destination_floor = destination_floor


        API_HEADER = {
            'Content-Type': "application/json",
            'Authorization': f"{self._auth.get_signature_method()} apiKey={self._API_KEY_STR},ts={timestamp},nonce={nonce},signature={signature_hex_str}"
        }
        if self._current_floor <= self._destination_floor:
            direction = "up"
        else:
            direction = "down"

        API_BODY = {
            "lineId": line_id,
            "callType": "hall",  # 요청 콜 종류 (hall: 홀콜 요청 / robot: 로봇 전용)
            "sourceFloor": self._current_floor,
            "direction": direction,
            "destinationFloor": self._destination_floor
        }

        try:
            response = requests.post(url, headers=API_HEADER, json=API_BODY)
            return response
        except requests.exceptions.RequestException as e:
            print(f"elevator call failed: {e}")
            return None
    
    def elevator_call_cancel(self, message_id : str):
        """
        엘리베이터를 호출을취소 하는 메소드입니다.
        :param line_id: 호출 라인 아이디
        :param call_type: 요청 콜 종류 (hall: 홀콜 요청 / robot: 로봇 전용)
        :param direction: 이동 방향 (up: 상행 / down: 하행)
        """
        timestamp = self._auth.generate_timestamp()
        nonce = self._auth.generate_nonce()
        signature_hex_str = self._auth.generate_signature(nonce, timestamp)
        url = f"{self.API_URL}/api/v1/el/call/thing/messageid/{message_id}"

        API_HEADER = {
            'Content-Type': "application/json",
            'Authorization': f"{self._auth.get_signature_method()} apiKey={self._API_KEY_STR},ts={timestamp},nonce={nonce},signature={signature_hex_str}"
        }
      

        try:
            response = requests.delete(url, headers=API_HEADER,)
            return response
        except requests.exceptions.RequestException as e:
            print(f"elevator call failed: {e}")
            return None
    
    def get_elevator_status(self, message_id: str):
        """
        엘리베이터의 상태를 확인하는 메소드입니다.
        :param message_id: 메시지 아이디
        """
        timestamp = self._auth.generate_timestamp()
        nonce = self._auth.generate_nonce()
        signature_hex_str = self._auth.generate_signature(nonce, timestamp)
        url = f"{self.API_URL}/api/v1/el/call/thing/messageid/{message_id}/status"

        API_HEADER = {
            'Content-Type': "application/json",
            'Authorization': f"{self._auth.get_signature_method()} apiKey={self._API_KEY_STR},ts={timestamp},nonce={nonce},signature={signature_hex_str}"
        }

        try:
            response = requests.get(url, headers=API_HEADER)
            return response
        except requests.exceptions.RequestException as e:
            print(f"elevator status check failed: {e}")
            return None

    def robot_status_sender(self, message_id: str, robot_status: str):
        """
        로봇의 상태를 전송하는 메소드입니다.
        :param message_id: 메시지 아이디
        :param robot_status: 로봇 상태 (예: sourceFloorWaiting, sourceFloorGettingOn 등)
        """
        timestamp = self._auth.generate_timestamp()
        nonce = self._auth.generate_nonce()
        signature_hex_str = self._auth.generate_signature(nonce, timestamp)
        url = f"{self.API_URL}/api/v1/el/call/thing/messageid/{message_id}/status/{robot_status}"
        API_HEADER = {
            'Content-Type': "application/json",
            'Authorization': f"{self._auth.get_signature_method()} apiKey={self._API_KEY_STR},ts={timestamp},nonce={nonce},signature={signature_hex_str}"
        }
        API_BODY = {
            "messageid": message_id,
            "status": robot_status
        }
       
        try:
            response = requests.put(url, headers=API_HEADER, json=API_BODY)
            return response
        except requests.exceptions.RequestException as e:
            print(f"robot_status_sender call failed: {e}")
            return None 
    

    def get_current_floor(self):
        """로봇의 현재 층을 확인하는 메소드입니다."""
        return self._current_floor
    def set_current_floor(self, current_floor: str):
        """로봇의 현재 층을 설정하는 메소드입니다."""
        self._current_floor = current_floor
        return self._current_floor