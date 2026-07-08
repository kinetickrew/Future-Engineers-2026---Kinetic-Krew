import cv2
import numpy as np

# Open the webcam
cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press 'q' to close the camera window.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Blur to reduce pixel noise
    blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

    # 1. CONVERT TO LAB COLOR SPACE
    lab_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2Lab)

    # --- DEFINE LAB COLOR RANGES ---
    # OpenCV LAB ranges: L (0-255), A (0-255, >128 is Red), B (0-255, <128 is Blue)
    
    # Red: High Lightness, High A (Red), Mildly positive B (Yellow-ish Red)
    low_red = np.array([40, 165, 135])
    high_red = np.array([255, 255, 180])
    
    # Green: Low-to-Mid Lightness, Low A (Green), High B (Yellow-Green)
    low_green = np.array([40, 40, 135])
    high_green = np.array([200, 110, 255])

    # Hot Pink: Mid-to-High Lightness, Very High A (Red element), Low B (Blue element)
    low_pink = np.array([50, 175, 70])
    high_pink = np.array([220, 255, 120])

    # --- CREATE MASKS ---
    mask_red = cv2.inRange(lab_frame, low_red, high_red)
    mask_green = cv2.inRange(lab_frame, low_green, high_green)
    mask_pink = cv2.inRange(lab_frame, low_pink, high_pink)

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

    # Process Red
    for contour in contours_red:
        if cv2.contourArea(contour) > 1500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "RED PILLAR", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Process Green
    for contour in contours_green:
        if cv2.contourArea(contour) > 1500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "GREEN PILLAR", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Process Pink
    for contour in contours_pink:
        if cv2.contourArea(contour) > 1500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 0, 255), 2)
            cv2.putText(frame, "PINK OBJECT", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 0, 255), 2)

    # Display live feed
    cv2.imshow("Live LAB Object Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()