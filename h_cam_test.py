import cv2
import socket
import pickle
import struct
import threading
from collections import deque

# 소켓 설정
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = "172.30.1.33"  # 윈도우 PC의 IP 주소
port = 5000  # 윈도우에서 설정한 포트
client_socket.connect((host_ip, port))
print(f"Connected to {host_ip}:{port}")

# 최신 데이터를 저장하는 deque 설정
frame_buffer = deque(maxlen=1)  # 최신 프레임만 저장 (최대 크기 1)

# 데이터 수신 스레드 함수
def receive_frames():
    data = b""
    payload_size = struct.calcsize("Q")
    while True:
        try:
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    print("Connection closed by server.")
                    return
                data += packet

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += client_socket.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            # 프레임 디코딩
            frame = pickle.loads(frame_data)

            # 최신 프레임으로 교체
            frame_buffer.clear()
            frame_buffer.append(frame)

        except Exception as e:
            print(f"Error receiving frame: {e}")
            break

    client_socket.close()

# 화면 출력 스레드 함수
def display_frames():
    while True:
        if frame_buffer:
            frame = frame_buffer[-1]  # 최신 프레임 가져오기
            cv2.imshow("Received", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()

# 스레드 생성 및 시작
receiver_thread = threading.Thread(target=receive_frames, daemon=True)
display_thread = threading.Thread(target=display_frames, daemon=True)

receiver_thread.start()
display_thread.start()

# 메인 스레드 대기
receiver_thread.join()
display_thread.join()
