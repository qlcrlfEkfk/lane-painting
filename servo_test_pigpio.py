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
    degree = max(30, min(150, degree))  # 각도를 0~180도로 제한
    pulsewidth = SERVO_MIN_PULSEWIDTH + (degree * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
    print("Servo01 - Degree: {} -> Pulse Width: {}".format(degree, pulsewidth))

    # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin01, pulsewidth)

def setServoPos02(degree):
    # 각도(degree)를 펄스 폭(pulse width)으로 변환
    degree = max(30, min(150, degree))  # 각도를 0~180도로 제한
    pulsewidth = SERVO_MIN_PULSEWIDTH + (degree * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
    print("Servo02 - Degree: {} -> Pulse Width: {}".format(degree, pulsewidth))

    # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin02, pulsewidth)

def setServoPos03(detected):
    # 특정 상태에 따라 서보 위치를 설정
    if detected:
        pulsewidth = SERVO_MIN_PULSEWIDTH + (40 * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
        print("Servo03 is moving down (Pulse Width: {})".format(pulsewidth))
    else:
        pulsewidth = SERVO_MIN_PULSEWIDTH + (150 * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180.0)
        print("Servo03 is moving up (Pulse Width: {})".format(pulsewidth))

    # 펄스 폭을 설정하여 서보 모터의 위치를 제어
    pi.set_servo_pulsewidth(servoPin03, pulsewidth)

if __name__ == "__main__":
    try:
        print("Starting servo test...")
        while True:  # 0도부터 180도까지 30도씩 
            setServoPos01(90)
            setServoPos02(90)
            setServoPos03(False)
            
    except KeyboardInterrupt:
        print("Test interrupted by user.")
    finally:
        # 서보 PWM 신호를 종료하고 pigpio를 종료
        print("Stopping servo and cleaning up.")
        pi.set_servo_pulsewidth(servoPin01, 0)
        pi.set_servo_pulsewidth(servoPin02, 0)
        pi.set_servo_pulsewidth(servoPin03, 0)
        pi.stop()
