import cv2
import os

# 비디오 캡처 (웹캠 사용)
cap = cv2.VideoCapture(1)  # 기본 웹캠 사용

# 이미지 저장 디렉토리 설정
save_dir = 'captured_images'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

img_counter = 0  # 캡처된 이미지 파일에 사용될 카운터*

print("Press 'C' to capture an image. Press 'Q' to quit.")


while True:
    ret, frame = cap.read()  # 카메라에서 프레임 읽기
    if not ret:
        print("Failed to grab frame")
        break
    
    cv2.imshow("Camera", frame)  # 현재 카메라 영상 화면에 표시

    key = cv2.waitKey(1) & 0xFF  # 키 입력 대기

    # 'C' 키를 눌렀을 때 프레임을 이미지로 저장
    if key == ord('c'):
        img_name = f"{save_dir}/image_{img_counter:04d}.png"  # 이미지 파일 이름 설정
        cv2.imwrite(img_name, frame)  # 이미지 저장
        print(f"Image saved: {img_name}")
        img_counter += 1  # 이미지 카운터 증가
    
    # 'Q' 키를 눌렀을 때 프로그램 종료
    elif key == ord('q'):
        print("Exiting...")
        break

# 자원 해제 및 창 닫기
cap.release()
cv2.destroyAllWindows()
