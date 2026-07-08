import cv2
import numpy as np

# Open the webcam (0 is usually the built-in webcam)
cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press 'q' to close the camera window.")

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Blur to reduce high-frequency pixel noise
    blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

    # Convert to HSV color space
    hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

    # --- DEFINE COLOR RANGES ---
    
    # Tighter Red (Strictly blocks orange and pink)
    low_red1 = np.array([0, 185, 120])
    high_red1 = np.array([6, 255, 255])
    low_red2 = np.array([174, 185, 120])
    high_red2 = np.array([180, 255, 255])
    
    # Stable Green
    low_green = np.array([40, 80, 70])
    high_green = np.array([80, 255, 255])

    # NEW: Hot Pink / Magenta Range
    low_pink = np.array([160, 150, 100])
    high_pink = np.array([172, 255, 255])

    # --- CREATE MASKS ---
    mask_red = cv2.inRange(hsv_frame, low_red1, high_red1) + cv2.inRange(hsv_frame, low_red2, high_red2)
    mask_green = cv2.inRange(hsv_frame, low_green, high_green)
    mask_pink = cv2.inRange(hsv_frame, low_pink, high_pink) # New Pink Mask

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

    # Process Red Objects
    for contour in contours_red:
        if cv2.contourArea(contour) > 1500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "RED PILLAR", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Process Green Objects
    for contour in contours_green:
        if cv2.contourArea(contour) > 1500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "GREEN PILLAR", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Process Pink Objects (BGR for a nice hot pink box: Magenta is high Blue and Red)
    for contour in contours_pink:
        if cv2.contourArea(contour) > 1500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 0, 255), 2) 
            cv2.putText(frame, "PINK OBJECT", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 0, 255), 2)

    # Display the live feed
    cv2.imshow("Live Object Detection", frame)

    # Break loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up and close windows
cap.release()
cv2.destroyAllWindows() 