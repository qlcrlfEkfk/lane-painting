import cv2
import socket
import pickle
import struct
import threading
import numpy as np
import time  # 딜레이를 위해 추가
import math
import servo_pigpio as sp

save_angle = 0 
local_frame = None
frame_lock = threading.Lock()

# 색상 필터링 함수
def color_filter(image):
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)  # BGR 이미지를 HLS 색상 공간으로 변환
    
    # 흰색 범위 설정
    lower_white = np.array([0, 190, 0])
    upper_white = np.array([255, 255, 255])
    white_mask = cv2.inRange(hls, lower_white, upper_white)  # 흰색 마스크 생성
    
    # 노란색 범위 설정
    yellow_lower = np.array([15, 30, 100])
    yellow_upper = np.array([35, 204, 255])
    yellow_mask = cv2.inRange(hls, yellow_lower, yellow_upper)  # 노란색 마스크 생성
    
    # 검은색 제외
    black_lower = np.array([0, 0, 0])
    black_upper = np.array([180, 255, 50])
    black_mask = cv2.inRange(hls, black_lower, black_upper)
    black_mask = cv2.bitwise_not(black_mask)  # 검은색 영역을 제외하기 위해 반전
    
    # 흰색과 노란색 마스크 결합
    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
    # final_mask = cv2.bitwise_and(combined_mask, black_mask)  # 검은색 제외한 마스크 적용
    masked = cv2.bitwise_and(image, image, mask=combined_mask)  # 필터링된 이미지 반환

    #나중에 마스크 처리부분만 확인시 주석 해제
    # cv2.imshow("Combined Mask", combined_mask)
    return masked

# 관심 영역 설정 함수 (오리지널)
def reg_of_int(img):
    img_h = img.shape[0]
    img_w = img.shape[1]
    # 사각형 형태로 관심 영역을 설정
    region = np.array([[(img_w // 2 - 320, img_h), (img_w // 2 + 320, img_h), (img_w // 2 + 320, 0), (img_w // 2 - 320, 0)]], dtype=np.int32)
    mask = np.zeros_like(img)  # 이미지와 동일한 크기의 빈 마스크 생성
    cv2.fillPoly(mask, region, 255)  # 관심 영역을 흰색으로 채움
    masked_img = cv2.bitwise_and(img, mask)  # 관심 영역이 아닌 부분을 제거
    return masked_img

# 이진화 처리 함수
def apply_threshold(image):
    _gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
    _, thresh = cv2.threshold(_gray, 1, 255, cv2.THRESH_BINARY)  # 임계값을 사용하여 이진화 처리
    return thresh

# 히스토그램을 통해 중심선 기반 탐지
def plothistogram(image):
    histogram = np.sum(image[image.shape[0] // 2:, :], axis=0)  # 이미지 하단 절반의 픽셀 합 계산
    midpoint = int(histogram.shape[0] / 2)
    center_region = histogram[midpoint // 2: midpoint + midpoint // 2]  # 중심 근처의 값만 사용
    centerbase = np.argmax(center_region) + (midpoint // 2)  # 최대값 인덱스 찾기
    return centerbase

# 슬라이딩 윈도우를 사용하여 차선 중심 탐지
def slide_window_search(binary_warped, center_current):
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))  # 시각화를 위한 출력 이미지
    nwindows = 21  # 슬라이딩 윈도우 수
    window_height = int(binary_warped.shape[0] / nwindows)  # 각 윈도우의 높이
    nonzero = binary_warped.nonzero()  # 0이 아닌 픽셀 좌표 추출
    nonzero_y = np.array(nonzero[0])
    nonzero_x = np.array(nonzero[1])
    margin = 250  # 윈도우 폭
    minpix = 60  # 최소 픽셀 수
    center_lane = []  # 중심선을 찾기 위한 리스트
    color = [0, 255, 0]
    thickness = 2
    global save_angle

    for w in range(9,21):
        # 윈도우 경계 설정
        win_y_low = binary_warped.shape[0] - (w + 1) * window_height
        win_y_high = binary_warped.shape[0] - w * window_height
        # 오리지널 코드
        win_xcenter_low = center_current - margin
        win_xcenter_high = center_current + margin
        # 시각화를 위해 윈도우 그리기
        cv2.rectangle(out_img, (win_xcenter_low, win_y_low), (win_xcenter_high, win_y_high), color, thickness)
        # 윈도우 내부의 픽셀 추출
        good_center = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xcenter_low) & (nonzero_x < win_xcenter_high)).nonzero()[0]
        center_lane.append(good_center)
        if len(good_center) > minpix:
            center_current = int(np.mean(nonzero_x[good_center]))  # 새로운 중심 갱신

    try:
        center_lane = np.concatenate(center_lane)
    except ValueError:
        print("Warning: No lanes found")
        return {'center_fitx': [], 'ploty': []}, out_img

    centerx = nonzero_x[center_lane]
    centery = nonzero_y[center_lane]
    if len(centerx) == 0:
        print("Warning: No center lane detected")
        return {'center_fitx': [], 'ploty': []}, out_img

    # 2차 함수로 차선을 근사화하여 곡선 그리기
    center_fit = np.polyfit(centery, centerx, 2)
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    center_fitx = center_fit[0] * ploty ** 2 + center_fit[1] * ploty + center_fit[2]
    ctx = np.trunc(center_fitx)  # 소수점 버림
    return {'center_fitx': ctx, 'ploty': ploty}, out_img
    
    # 1차 함수로 차선을 근사화하여 직선 그리기
    # center_fit = np.polyfit(centery, centerx, 1)
    # ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    # center_fitx = center_fit[0] * ploty + center_fit[1]
    # save_angle = (np.degrees(np.arctan(center_fit[0])))
    # ctx = np.trunc(center_fitx)  # 소수점 버림
    # return {'center_fitx': ctx, 'ploty': ploty}, out_img

# changed code
def process_frame(frame, ym_per_pix, xm_per_pix):
    # Original frame dimensions
    height, width, _ = frame.shape
    extended_height = height + 360  # Extend Y-axis by 200 pixels

    # Create a black canvas with extended height
    extended_frame = np.zeros((extended_height, width, 3), dtype=np.uint8)
    extended_frame[:height, :, :] = frame  # Copy original frame to the top
    
    # 색상 필터 적용
    filtered_image = color_filter(extended_frame)
    # 관심 영역(ROI) 설정
    roi_image = reg_of_int(filtered_image)
    # 임계값 처리
    thresholded_image = apply_threshold(roi_image)
    # 히스토그램을 기반으로 중심선 찾기
    centerbase = plothistogram(thresholded_image)
    # 슬라이딩 윈도우 탐색
    draw_info, visualization_img = slide_window_search(thresholded_image, centerbase)

    if isinstance(draw_info, dict) and 'center_fitx' in draw_info and draw_info['center_fitx'] != []:
        ploty = draw_info['ploty']  # y 좌표 배열
        center_fitx = draw_info['center_fitx']  # 검출된 중심선의 x 좌표 배열

        # 서보모터 활성화
        sp.setServoPos03(True)

        # 중심선에 초록색 점 그리기
        for y, x in zip(ploty.astype(int), center_fitx.astype(int)):
            cv2.circle(extended_frame, (x, y), 1, (0, 255, 0), -1)

        # 가상의 y 값 840까지 계산을 확장
        y_extended = np.linspace(0, 840, num=840)
        x_extended = np.interp(y_extended, ploty, center_fitx)  # 기존 선을 기반으로 보간 2차

        mvp_x1 = 0
        mvp_y1 = 0
        flag = False 
        # 초록선 확장 #############################################
        for i in range(len(y_extended) - 1):
            y1, x1 = int(y_extended[i]), int(x_extended[i])
            
            distance = math.sqrt((x1 - 320) ** 2 + (y1 - 336) ** 2)
            if((y1 > 336) and distance >= 384 and not flag):
                mvp_x1, mvp_y1 = x1, y1
                flag = True
            
            y2, x2 = int(y_extended[i + 1]), int(x_extended[i + 1])
            cv2.line(extended_frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
        #############################################

        # 초록선 방정식 계산
        green_line_slope = (x_extended[-1] - x_extended[0]) / (y_extended[-1] - y_extended[0]) if (y_extended[-1] - y_extended[0]) != 0 else 0
        green_line_intercept = x_extended[0] - green_line_slope * y_extended[0]

        red_point_x, red_point_y = mvp_x1, mvp_y1  # 초기값

        # 교차점 빨간 점 표시
        if mvp_y1 is not None and red_point_y is not None:
            cv2.circle(extended_frame, (mvp_x1, mvp_y1), 5, (0, 0, 255), -1)
        else:
            print("교차점 없음")
        ############################

        # 초록색 선의 기울기 계산
        green_line_angle = np.degrees(np.arctan(green_line_slope))  # 기울기를 각도로 변환

        ############################
        # 빨간색 선 그리기 (중심 기준)
        length = 120  # 선의 길이
        dx = int(length * np.cos(np.radians(green_line_angle)))
        dy = int(length * np.sin(np.radians(green_line_angle)))

        ############################

        # 파란 점 중심으로 아래쪽 반원 그리기
        center_point = (320, 336)
        radius = 384

        # 엘립스를 사용하여 아래쪽 반원 그리기
        cv2.ellipse(extended_frame, center_point, (radius, radius), 0, 0, 180, (0, 165, 255), 1)

        # 중심점 기준으로 빨간색 선 그리기
        cv2.line(extended_frame, (red_point_x, red_point_y), (red_point_x + dx, red_point_y - dy), (0, 0, 255), 10)
        cv2.line(extended_frame, (red_point_x, red_point_y), (red_point_x - dx, red_point_y + dy), (0, 0, 255), 10)

        # 카메라 중심 파란색 선 그리기 & 모터 중심점
        cv2.line(extended_frame, (320, 0), (320, 840), (255, 0, 0), 2)
        cv2.circle(extended_frame, (320, 336), 5, (255, 0, 0), -1)

        # 모터 중심점과 빨강 교차점 그리기
        cv2.line(extended_frame, (320, 336), (red_point_x, red_point_y), (0, 255, 255), 2)

        # 두 점의 좌표 1모터 각
        x1, y1 = 320, 336  # 첫 번째 점
        x2, y2 = red_point_x, red_point_y  # 두 번째 점
        dx01 = x2 - x1
        dy01 = y2 - y1
        angle_radians01 = np.arctan2(-dy01, dx01)  # 역탄젠트 사용 (라디안 단위)
        angle_degrees01 = np.degrees(angle_radians01) + 180# 라디안을 각도로 변환
        sp.setServoPos01(angle_degrees01)
        print(f"1번 모터 기울기 각도: {angle_degrees01:.2f}도")

        angle_radians02 = np.arctan2(-dy, dx)  # 역탄젠트 계산
        angle_degrees02 = np.degrees(angle_radians02)  # 라디안을 각도로 변환
        print(f"변환전 2번 모터 기울기 각도 {round(angle_degrees02)}도")
        angle_degrees02 = angle_degrees02 + angle_degrees01
        angle_degrees02 = round(angle_degrees02)
        sp.setServoPos02(angle_degrees02)
        print(f"2번 모터 기울기 각도 {angle_degrees02}도")

    else:
        # 선이 검출되지 않았을 경우 모든 서보모터 정지
        sp.setServoPos03(False)
        sp.setServoPos01(90)  # 첫 번째 서보모터 정지
        sp.setServoPos02(90)  # 추가 서보모터 정지 (필요시 추가)
        print("선이 검출되지 않음: 모든 서보모터 정지")

    return filtered_image, roi_image, thresholded_image, visualization_img, extended_frame
#로컬 프레임 저장/읽기
def set_local_frame(frame):
    global local_frame
    with frame_lock:
        local_frame = frame

def get_local_frame():
    global local_frame
    with frame_lock:
        return local_frame

def display_frames(server_frame, visualization_img):
    # 병합된 프레임 가져오기
    combined_frame = combine_frames(server_frame)

    # 병합된 프레임 창 표시
    if combined_frame is not None:
        cv2.imshow("Combined Video (Lane + Server)", combined_frame)
    else:
        print("Combined frame is None")

    # visualization_img 창 표시
    if visualization_img is not None:
        cv2.imshow("Visualization", visualization_img)
    else:
        print("Visualization image is None")

# 서버에서 프레임 수신 및 병합
def receive_frames(client_socket):
    global local_frame
    data = b""
    payload_size = struct.calcsize(">L")
    ym_per_pix, xm_per_pix = 0.56 / 480, 0.37 / 640
    while True:
        while len(data) < payload_size:
            packet = client_socket.recv(4096)
            if not packet:
                return
            data += packet

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        while len(data) < msg_size:
            data += client_socket.recv(4096)

        frame_data = data[:msg_size]
        data = data[msg_size:]
        try:
            server_frame = pickle.loads(frame_data)
            # 디버깅용 출력
            if server_frame is None:
                print("Received server_frame is None")
            elif not isinstance(server_frame, np.ndarray):
                print("Received server_frame is not a NumPy array")
            elif len(server_frame.shape) != 3:
                print(f"Received server_frame has unexpected shape: {server_frame.shape}")
            else:
                print(f"Received server_frame shape: {server_frame.shape}")

            # 서버 프레임에 차선 인식 적용
            _, _, _, visualization_img, processed_server_frame = process_frame(server_frame, ym_per_pix, xm_per_pix)

            display_frames(processed_server_frame, visualization_img)
        
        except Exception as e:
            print(f"Error decoding server_frame: {e}")

      

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# 로컬 카메라 프레임 캡처
def capture_local_camera():
    global local_frame
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open local camera.")
        return
    
    ym_per_pix = 0.56 / 480  # 세로 픽셀당 미터 (픽셀을 실제 거리로 변환하기 위한 값)
    xm_per_pix = 0.37 / 640  # 가로 픽셀당 미터 (픽셀을 실제 거리로 변환하기 위한 값)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame from local camera.")
            break
        # 각 단계별 처리 결과 반환
        _, _, _, visualization_img, lane_result = process_frame(frame, ym_per_pix, xm_per_pix)
        
        # 각 단계별 이미지를 화면에 표시
        # cv2.imshow("Color Filter", filtered_image)
        # cv2.imshow("Thresholded Image", thresholded_image)
        set_local_frame(lane_result)
        # visualization_img를 별도의 창에 표시
        cv2.imshow("Local Visualization", visualization_img)

# test code please delete this message^^

# 서버와 클라이언트 프레임을 병합
def combine_frames(server_frame):
    local_frame_snapshot = get_local_frame()  # 로컬 프레임 안전하게 가져오기
    
    if server_frame is None or not isinstance(server_frame, np.ndarray):
        print("warning: server_frame is invalid")
        return None
    
    if len(server_frame.shape) !=3:
        print("warning: server_frame does not have 3 channels")
        return None

    if local_frame_snapshot is not None:
        # 서버 프레임과 로컬 프레임 크기 맞추기
        height, width, _ = server_frame.shape[:2]
        resized_local_frame = cv2.resize(local_frame, (width, height))

        # 두 프레임 병합 (좌: 서버, 우: 로컬)
        combined_frame = cv2.hconcat([server_frame, resized_local_frame])
    else:
        # 로컬 프레임이 없을 경우 서버 프레임만 사용
        combined_frame = server_frame
        # 두 프레임을 병합 (좌: 서버, 우: 로컬)
    
    return combined_frame

if __name__ == "__main__":
    server_ip = "172.30.1.13"  # 서버의 IP 주소 입력
    port = 8485

    # 소켓 설정 및 연결
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, port))
    print(f"Connected to server {server_ip}:{port}")

    # 멀티 쓰레드로 작업 분리
    local_camera_thread = threading.Thread(target=lambda: capture_local_camera(), daemon=True)
    local_camera_thread.start()

    # 서버로부터 프레임 수신
    try:
        receive_frames(client_socket)

    finally:
        client_socket.close()
        cv2.destroyAllWindows()
