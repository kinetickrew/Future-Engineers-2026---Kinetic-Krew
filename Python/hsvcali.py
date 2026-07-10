"""
HSV Calibration Tool
---------------------
Run this FIRST, in the actual lighting you care about (IKEA orange bulb + sunlight mix),
with the green object in frame. Drag the sliders until ONLY the green object is white
in the mask window. Then copy the printed low_green / high_green values into your
main detection script.

Press 'q' to quit and print the final values.
"""
import cv2
import numpy as np

def nothing(x):
    pass


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


clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", 35, 179, nothing)
cv2.createTrackbar("H Max", "Trackbars", 85, 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", 40, 255, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 40, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)

print("Adjust sliders until only the green object shows white in the Mask window.")
print("Press 'q' to quit and print final values.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    balanced = white_balance_gray_world(frame)
    blurred = cv2.GaussianBlur(balanced, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    h_ch, s_ch, v_ch = cv2.split(hsv)
    v_ch = clahe.apply(v_ch)
    hsv = cv2.merge([h_ch, s_ch, v_ch])

    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print(f"\nFinal values:")
        print(f"low_green  = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"high_green = np.array([{h_max}, {s_max}, {v_max}])")
        break

cap.release()
cv2.destroyAllWindows()