import pigpio  # pigpio 라이브러리 사용
from time import sleep  # sleep 함수 사용

# 핀 설정
inputPin23 = 23  # 입력 핀 23번
inputPin24 = 24  # 입력 핀 24번

# pigpio 라이브러리의 핀 설정
pi = pigpio.pi()

# GPIO 23, 24번 핀을 입력 모드로 설정
pi.set_mode(inputPin23, pigpio.INPUT)
pi.set_mode(inputPin24, pigpio.INPUT)

# Pull-up 또는 Pull-down 설정 (옵션, 여기서는 풀업을 설정)
pi.set_pull_up_down(inputPin23, pigpio.PUD_UP)
pi.set_pull_up_down(inputPin24, pigpio.PUD_UP)

# 입력 값 확인 함수
def check_input_signal():
    global flag_23, flag_24
    while True:
        input_state23 = pi.read(inputPin23)  # 핀 23번의 입력 상태 읽기
        input_state24 = pi.read(inputPin24)  # 핀 24번의 입력 상태 읽기
        flag_23, flag_24 = input_state23, input_state24


