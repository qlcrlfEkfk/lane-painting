import pigpio  # pigpio 라이브러리 사용
from time import sleep  # sleep 함수 사용

servoPin01 = 18  # 서보 모터가 연결된 하드웨어 PWM 핀 보드번호 32번
servoPin02 = 12  # 서보 모터가 연결된 하드웨어 PWM 핀 보드번호 33번
servoPin03 = 13  # 서보 모터가 연결된 하드웨어 PWM 핀 보드번호 12번
SERVO_MIN_PULSEWIDTH = 500   # 서보의 최소 위치에 대응하는 펄스 폭 (마이크로초)
SERVO_MAX_PULSEWIDTH = 2400  # 서보의 최대 위치에 대응하는 펄스 폭 (마이크로초)

# pigpio 라이브러리의 핀 설정
pi = pigpio.pi()

# 서보 위치 제어 함수
def setServoPos01(degree):
    # 각도(degree)를 펄스 폭(pulse width)으로 변환
    degree = degree
    degree = max(30, min(150, degree))  # 각도를 0~180도로 제한
    pulsewidth = SERVO_MIN_PULSEWIDTH + (degree * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
    print("Degree 001: {} to {} (Pulse Width)".format(degree, pulsewidth))

    # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin01, pulsewidth)

# 서보 위치 제어 함수
def setServoPos02(degree):
    # 각도(degree)를 펄스 폭(pulse width)으로 변환
    degree = degree
    degree = max(30, min(150, degree))  # 각도를 0~180도로 제한
    pulsewidth = SERVO_MIN_PULSEWIDTH + (degree * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
    print("Degree 002 : {} to {} (Pulse Width)".format(degree, pulsewidth))

    # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin02, pulsewidth)
    
# 서보 위치 제어 함수
def setServoPos03(degree):
    if degree < 90:
        pulsewidth = SERVO_MIN_PULSEWIDTH + (40 * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
        print("turn left")
    elif degree == 90:
        pulsewidth = SERVO_MIN_PULSEWIDTH + (90 * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
        print("turn stop")
    elif degree > 90:
        pulsewidth = SERVO_MIN_PULSEWIDTH + (140 * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
        print("turn right")

        # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin03, pulsewidth)


# def setServoPos1_2(degree, distance):
    