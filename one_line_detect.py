import cv2
import numpy as np
import time  # 딜레이를 위해 추가
import math

# 색상 필터링 함수
def color_filter(image):
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)  # BGR 이미지를 HLS 색상 공간으로 변환
    
    # 흰색 범위 설정
    lower_white = np.array([0, 230, 0])
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
    final_mask = cv2.bitwise_and(combined_mask, black_mask)  # 검은색 제외한 마스크 적용
    masked = cv2.bitwise_and(image, image, mask=final_mask)  # 필터링된 이미지 반환

    return masked

# 관심 영역 설정 함수
def reg_of_int(img):
    img_h = img.shape[0]
    img_w = img.shape[1]
    # 사각형 형태로 관심 영역을 설정
    region = np.array([[(img_w // 2 - 200, img_h), (img_w // 2 + 200, img_h), (img_w // 2 + 200, 0), (img_w // 2 - 200, 0)]], dtype=np.int32)
    mask = np.zeros_like(img)  # 이미지와 동일한 크기의 빈 마스크 생성
    cv2.fillPoly(mask, region, 255)  # 관심 영역을 흰색으로 채움
    masked_img = cv2.bitwise_and(img, mask)  # 관심 영역이 아닌 부분을 제거
    return masked_img

# 이진화 처리 함수
def apply_threshold(image):
    _gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
    ret, thresh = cv2.threshold(_gray, 1, 255, cv2.THRESH_BINARY)  # 임계값을 사용하여 이진화 처리
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
    nwindows = 10  # 슬라이딩 윈도우 수
    window_height = int(binary_warped.shape[0] / nwindows)  # 각 윈도우의 높이
    nonzero = binary_warped.nonzero()  # 0이 아닌 픽셀 좌표 추출
    nonzero_y = np.array(nonzero[0])
    nonzero_x = np.array(nonzero[1])
    margin = 240  # 윈도우 폭
    minpix = 60  # 최소 픽셀 수
    center_lane = []  # 중심선을 찾기 위한 리스트
    color = [0, 255, 0]
    thickness = 2

    for w in range(4,7):
        # 윈도우 경계 설정
        win_y_low = binary_warped.shape[0] - (w + 1) * window_height
        win_y_high = binary_warped.shape[0] - w * window_height
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
    # center_fit = np.polyfit(centery, centerx, 2)
    # ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    # center_fitx = center_fit[0] * ploty ** 2 + center_fit[1] * ploty + center_fit[2]
    # ctx = np.trunc(center_fitx)  # 소수점 버림
    # return {'center_fitx': ctx, 'ploty': ploty}, out_img
    
    # 2차 함수로 차선을 근사화하여 곡선 그리기
    center_fit = np.polyfit(centery, centerx, 1)
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    center_fitx = center_fit[0] * ploty + center_fit[1]
    print(np.degrees(np.arctan(center_fit[0])))
    ctx = np.trunc(center_fitx)  # 소수점 버림
    return {'center_fitx': ctx, 'ploty': ploty}, out_img

# 프레임 처리 함수
def process_frame(frame, ym_per_pix, xm_per_pix):
    filtered_image = color_filter(frame)
    roi_image = reg_of_int(filtered_image)
    thresholded_image = apply_threshold(roi_image)
    centerbase = plothistogram(thresholded_image)
    draw_info, visualization_img = slide_window_search(thresholded_image, centerbase)

    if isinstance(draw_info, dict) and 'center_fitx' in draw_info and draw_info['center_fitx'] != []:
        ploty = draw_info['ploty']
        center_fitx = draw_info['center_fitx']

        # 메인 프레임에 초록색 점을 직접 그리기
        for y, x in zip(ploty.astype(int), center_fitx.astype(int)):
            cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)  # 프레임에 초록색 점 그리기

    return filtered_image, roi_image, thresholded_image, visualization_img, frame

# 동영상(카메라) 처리 함수
def process_video():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    ym_per_pix = 0.56 / 480  # 세로 픽셀당 미터 (픽셀을 실제 거리로 변환하기 위한 값)
    xm_per_pix = 0.37 / 640  # 가로 픽셀당 미터 (픽셀을 실제 거리로 변환하기 위한 값)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        
        # 각 단계별 처리 결과 반환
        filtered_image, roi_image, thresholded_image, visualization_img, lane_result = process_frame(frame, ym_per_pix, xm_per_pix)
        
        # 각 단계별 이미지를 화면에 표시
        cv2.imshow("Color Filter", filtered_image)
        cv2.imshow("Thresholded Image", thresholded_image)
        cv2.imshow("Slide Window Search & Lane Detection", visualization_img)
        cv2.imshow("Result", frame)  # 메인 프레임에 그려진 결과 표시
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    process_video()
