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

    # Convert the frame from BGR (standard OpenCV format) to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- DEFINE COLOR RANGES ---
    # HSV values can be tricky. Here are rough estimates for bright colors:
    
    # Red has two ranges because it sits at the very beginning and end of the HSV spectrum
    low_red1 = np.array([0, 120, 70])
    high_red1 = np.array([10, 255, 255])
    low_red2 = np.array([170, 120, 70])
    high_red2 = np.array([180, 255, 255])
    
    # Green range
    low_green = np.array([35, 40, 30])
    high_green = np.array([85, 255, 255])

    # Create masks (black and white images where white = the color we want)
    mask_red1 = cv2.inRange(hsv_frame, low_red1, high_red1)
    mask_red2 = cv2.inRange(hsv_frame, low_red2, high_red2)
    mask_red = mask_red1 + mask_red2
    
    mask_green = cv2.inRange(hsv_frame, low_green, high_green)


    # --- DETECT AND DRAW BOXES ---
    # We find contours (outlines) of the colored sections
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Process Red Objects
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if area > 1000:  # Filter out tiny specks/noise
            x, y, w, h = cv2.boundingRect(contour)
            # Draw a red box (BGR format: Blue=0, Green=0, Red=255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "RED PILLAR", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            print("Detected: RED PILLAR")

    # Process Green Objects
    for contour in contours_green:
        area = cv2.contourArea(contour)
        if area > 1000:  # Filter out noise
            x, y, w, h = cv2.boundingRect(contour)
            # Draw a green box (BGR format: Blue=0, Green=255, Red=0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "GREEN PILLAR", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print("Detected: GREEN PILLAR")

    # Display the live feed
    cv2.imshow("Live Object Detection", frame)

    # Break loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up and close windows
cap.release()
cv2.destroyAllWindows()