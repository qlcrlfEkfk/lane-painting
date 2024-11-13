import pigpio  # pigpio 라이브러리 사용
from time import sleep  # sleep 함수 사용

servoPin01 = 12  # 서보 모터가 연결된 하드웨어 PWM 핀 보드번호 32번
servoPin02 = 13  # 서보 모터가 연결된 하드웨어 PWM 핀 보드번호 33번
servoPin03 = 18  # 서보 모터가 연결된 하드웨어 PWM 핀 보드번호 12번
SERVO_MIN_PULSEWIDTH = 500   # 서보의 최소 위치에 대응하는 펄스 폭 (마이크로초)
SERVO_MAX_PULSEWIDTH = 2400  # 서보의 최대 위치에 대응하는 펄스 폭 (마이크로초)

# pigpio 라이브러리의 핀 설정
pi = pigpio.pi()

# 서보 위치 제어 함수
def setServoPos01(degree):
    # 각도(degree)를 펄스 폭(pulse width)으로 변환
    degree = degree + 180
    if degree > 180:
        degree = 180
    elif degree < 0:
        degree = 0

    pulsewidth = SERVO_MIN_PULSEWIDTH + (degree * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
    print("Degree: {} to {} (Pulse Width)".format(degree, pulsewidth))

    # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin01, pulsewidth)

# 서보 위치 제어 함수
def setServoPos02(degree):
    # 각도(degree)를 펄스 폭(pulse width)으로 변환
    degree = degree + 180
    if degree > 180:
        degree = 180
    elif degree < 0:
        degree = 0

    pulsewidth = SERVO_MIN_PULSEWIDTH + (degree * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
    print("Degree: {} to {} (Pulse Width)".format(degree, pulsewidth))

    # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin02, pulsewidth)
    
# 서보 위치 제어 함수
def setServoPos03(detected):
    if detected:
        pulsewidth = SERVO_MIN_PULSEWIDTH + (170 * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
        print("Servo03 is going down")
    else:
        pulsewidth = SERVO_MIN_PULSEWIDTH + (10 * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
        print("Servo03 is going up")

        # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin03, pulsewidth)

# if __name__ == "__main__":  
#     try:
#         while True:
#             setServoPos01(0)      # 서보를 0도로 이동
#             sleep(2)
#             setServoPos01(30)      # 서보를 30도로 이동
#             sleep(2)
#             setServoPos01(60)      # 서보를 60도로 이동
#             sleep(2)
#             setServoPos01(90)     # 서보를 90도로 이동
#             sleep(2)
#             setServoPos01(120)     # 서보를 50도로 이동
#             sleep(2)
#             setServoPos01(150)    # 서보를 120도로 이동
#             sleep(2)
#             setServoPos01(180)    # 서보를 180도로 이동
#             sleep(2)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # 서보 PWM 신호를 종료하고 pigpio를 종료
#         pi.set_servo_pulsewidth(servoPin03, 0)
#         pi.stop()