# ElevatorStatus.msg
# elInfoList는 여러 개의 엘리베이터 정보를 담으므로,
# 개별 엘리베이터 상태를 먼저 정의하고, 나중에 리스트로 포함시킬 것입니다.

# 개별 엘리베이터 상태 (API의 elInfoList의 각 객체에 해당)
string el_id            # EL 일련번호
string mode             # EL 운행 모드 ( auto, fault, insp, parking, etc )
string current_floor    # EL 현재 층 (표기층)
string direction        # EL 이동 방향 ( none, up, down )
string door_status      # 도어 상태 ( open, close, opening, closing )
string registed_up_hall_call # 상행 홀콜 등록 현황 (쉼표 구분 문자열)
string registed_dn_hall_call # 하행 홀콜 등록 현황 (쉼표 구분 문자열)
string registed_car_call   # 카콜 등록 현황 (쉼표 구분 문자열)