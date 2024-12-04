import pigpio  # pigpio 라이브러리 사용
from time import sleep  # sleep 함수 사용

# 핀 설정
inputPin = 23  # 입력 핀 23번

# pigpio 라이브러리의 핀 설정
pi = pigpio.pi()

# GPIO 23번 핀을 입력 모드로 설정
pi.set_mode(inputPin, pigpio.INPUT)

# Pull-up 또는 Pull-down 설정 (옵션, 여기서는 풀업을 설정)
pi.set_pull_up_down(inputPin, pigpio.PUD_UP)

# 입력 값 확인 함수
def check_input_signal():
    while True:
        input_state = pi.read(inputPin)  # 핀 23번의 입력 상태 읽기
        if input_state == 1:
            print("신호 받음")  # High 신호일 때
        else:
            print("신호 못 받음")  # Low 신호일 때
        
        sleep(1)  # 1초마다 확인

# 신호 확인 함수 실행
check_input_signal()
