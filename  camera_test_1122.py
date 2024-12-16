import cv2
import numpy as np

def process_frame_with_extension(frame):
    # Original frame dimensions
    height, width, _ = frame.shape
    extended_height = height + 200  # Extend Y-axis by 200 pixels

    # Create a black canvas with extended height
    extended_frame = np.zeros((extended_height, width, 3), dtype=np.uint8)
    extended_frame[:height, :, :] = frame  # Copy original frame to the top

    # Convert to grayscale and detect edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)

    # Define ROI for lane detection
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height), (width, height), (width // 2 + 50, height // 2), (width // 2 - 50, height // 2)
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)

    # Hough Transform to detect lines
    lines = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=150)

    if lines is not None:
        # Collect points from detected lines
        points = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            points.append((x1, y1))
            points.append((x2, y2))
        
        # Sort points by x-coordinates
        points = sorted(points, key=lambda p: p[0])
        points = np.array(points)

        # Fit a 2nd degree polynomial
        poly = np.polyfit(points[:, 0], points[:, 1], 2)  # 2nd degree polynomial
        poly_func = np.poly1d(poly)

        # Generate x and y values for the polynomial curve
        x_vals = np.linspace(0, width, 500)
        y_vals = poly_func(x_vals)

        # Draw the curve in the original frame and extended area
        for i in range(len(x_vals) - 1):
            x1, y1 = int(x_vals[i]), int(y_vals[i])
            x2, y2 = int(x_vals[i + 1]), int(y_vals[i + 1])
            if 0 <= y1 < height:  # Draw within the original frame
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            elif y1 >= height:  # Draw in the extended area
                y1_ext = y1 - height
                y2_ext = y2 - height
                if 0 <= y1_ext < 200 and 0 <= y2_ext < 200:
                    cv2.line(extended_frame[height:], (x1, y1_ext), (x2, y2_ext), (255, 0, 0), 2)

    return extended_frame

# Real-time video capture
cap = cv2.VideoCapture(0)  # Use 0 for webcam, or provide a video file path

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Process frame with extended Y-axis
    processed_frame = process_frame_with_extension(frame)

    # Display the result
    cv2.imshow('Lane Detection with Extended Y-Axis', processed_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

cap.release()
cv2.destroyAllWindows()