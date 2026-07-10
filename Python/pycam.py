import cv2
import numpy as np

# Open the webcam (0 is usually the built-in webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press 'q' to close the camera window.")

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def white_balance_gray_world(img):
    result = img.astype(np.float32)
    avg_b = np.mean(result[:, :, 0])
    avg_g = np.mean(result[:, :, 1])
    avg_r = np.mean(result[:, :, 2])
    avg_gray = (avg_b + avg_g + avg_r) / 3.0
    result[:, :, 0] *= (avg_gray / max(avg_b, 1e-6))
    result[:, :, 1] *= (avg_gray / max(avg_g, 1e-6))
    result[:, :, 2] *= (avg_gray / max(avg_r, 1e-6))
    return np.clip(result, 0, 255).astype(np.uint8)


def is_pillar_shaped(contour, min_aspect_ratio=1.2, min_solidity=0.85):
    x, y, w, h = cv2.boundingRect(contour)
    if w == 0:
        return False
    aspect_ratio = h / float(w)
    contour_area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = contour_area / float(hull_area) if hull_area > 0 else 0
    return aspect_ratio >= min_aspect_ratio and solidity >= min_solidity


while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    balanced_frame = white_balance_gray_world(frame)
    blurred_frame = cv2.GaussianBlur(balanced_frame, (5, 5), 0)
    hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

    h_ch, s_ch, v_ch = cv2.split(hsv_frame)
    v_ch = clahe.apply(v_ch)
    hsv_frame = cv2.merge([h_ch, s_ch, v_ch])

    # --- DEFINE COLOR RANGES ---

    low_red1 = np.array([0, 140, 100])
    high_red1 = np.array([8, 255, 255])
    low_red2 = np.array([172, 140, 100])
    high_red2 = np.array([180, 255, 255])

    # GREEN: RECALIBRATE using the updated hsv_calibrator.py (now with white
    # balance + CLAHE applied) -- your old numbers [32,37,0]-[88,137,158]
    # were found on the raw uncorrected feed and won't be right anymore.
    low_green = np.array([32, 37, 0])
    high_green = np.array([88, 137, 158])

    low_pink = np.array([160, 150, 100])
    high_pink = np.array([172, 255, 255])

    # --- CREATE MASKS ---
    mask_red = cv2.inRange(hsv_frame, low_red1, high_red1) + cv2.inRange(hsv_frame, low_red2, high_red2)
    mask_green = cv2.inRange(hsv_frame, low_green, high_green)
    mask_pink = cv2.inRange(hsv_frame, low_pink, high_pink)

    # --- MORPHOLOGICAL CLEANUP ---
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

    mask_pink = cv2.morphologyEx(mask_pink, cv2.MORPH_OPEN, kernel)
    mask_pink = cv2.morphologyEx(mask_pink, cv2.MORPH_CLOSE, kernel)

    # --- DETECT AND DRAW BOXES ---
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_pink, _ = cv2.findContours(mask_pink, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours_red:
        if cv2.contourArea(contour) > 1500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "RED PILLAR", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    for contour in contours_green:
        if cv2.contourArea(contour) > 1500 and is_pillar_shaped(contour):
            hull = cv2.convexHull(contour)
            x, y, w, h = cv2.boundingRect(hull)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "GREEN PILLAR", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    for contour in contours_pink:
        if cv2.contourArea(contour) > 1500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 0, 255), 2)
            cv2.putText(frame, "PINK OBJECT", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 0, 255), 2)

    cv2.imshow("Live Object Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()