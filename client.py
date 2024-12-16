import cv2
import socket
import pickle
import struct

def start_camera_stream(server_ip, server_port):
    # 소켓 초기화
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))
    print(f"Connected to server {server_ip}:{server_port}")

    cap = cv2.VideoCapture(0)  # 카메라 초기화 (0번 카메라)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 전송 품질 설정
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break

            # 프레임 압축 및 직렬화
            _, frame_encoded = cv2.imencode('.jpg', frame, encode_param)
            frame_data = pickle.dumps(frame_encoded)

            # 메시지 크기 정보 전송
            message_size = struct.pack(">L", len(frame_data))
            client_socket.sendall(message_size + frame_data)

            # 화면에 캡처된 프레임 출력 (디버깅용)
            cv2.imshow("Camera Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        client_socket.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 라즈베리파이2의 IP와 포트를 입력
    start_camera_stream("172.30.1.65", 8001)  # 예시 IP와 포트
