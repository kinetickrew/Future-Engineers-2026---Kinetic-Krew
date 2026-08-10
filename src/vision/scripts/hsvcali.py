"""
General HSV Calibration Tool (single-range OR red-style wraparound)
--------------------------------------------------------------------
Works for BOTH:
  - Normal colors (green, blue, etc.) -- use only Range 1 sliders.
  - Red (or anything that wraps around 0/180 on the hue wheel) -- turn on
    "Wrap Mode" and use Range 1 for the low end (near H=0) and Range 2 for
    the high end (near H=179).

Applies the same white balance + CLAHE preprocessing as the main detection
script, so the numbers you find here will match what that script sees.

Controls:
- "Wrap Mode" slider: 0 = single range (e.g. green), 1 = dual range (e.g. red)
- "H1 Min"/"H1 Max": Range 1 hue bounds (always used)
- "H2 Min"/"H2 Max": Range 2 hue bounds (only used when Wrap Mode = 1)
- "S Min"/"S Max", "V Min"/"V Max": shared saturation/value bounds

Press 'q' to quit and print the final values in the right format.
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
cv2.createTrackbar("Wrap Mode", "Trackbars", 0, 1, nothing)   # 0 = single range, 1 = red-style dual range
cv2.createTrackbar("H1 Min", "Trackbars", 35, 179, nothing)
cv2.createTrackbar("H1 Max", "Trackbars", 85, 179, nothing)
cv2.createTrackbar("H2 Min", "Trackbars", 170, 179, nothing)  # only used in Wrap Mode
cv2.createTrackbar("H2 Max", "Trackbars", 179, 179, nothing)  # only used in Wrap Mode
cv2.createTrackbar("S Min", "Trackbars", 40, 255, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 40, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)

print("For GREEN (or any non-wrapping color): leave Wrap Mode at 0, tune H1/S/V only.")
print("For RED (wraps around 0/180): set Wrap Mode to 1, tune H1 for the low end")
print("  (near 0) and H2 for the high end (near 179), shared S/V for both.")
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

    wrap_mode = cv2.getTrackbarPos("Wrap Mode", "Trackbars")
    h1_min = cv2.getTrackbarPos("H1 Min", "Trackbars")
    h1_max = cv2.getTrackbarPos("H1 Max", "Trackbars")
    h2_min = cv2.getTrackbarPos("H2 Min", "Trackbars")
    h2_max = cv2.getTrackbarPos("H2 Max", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")

    lower1 = np.array([h1_min, s_min, v_min])
    upper1 = np.array([h1_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower1, upper1)

    if wrap_mode == 1:
        lower2 = np.array([h2_min, s_min, v_min])
        upper2 = np.array([h2_max, s_max, v_max])
        mask = mask + cv2.inRange(hsv, lower2, upper2)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("\nFinal values:")    
        if wrap_mode == 1:
            print(f"low_red1  = np.array([{h1_min}, {s_min}, {v_min}])")
            print(f"high_red1 = np.array([{h1_max}, {s_max}, {v_max}])")
            print(f"low_red2  = np.array([{h2_min}, {s_min}, {v_min}])")
            print(f"high_red2 = np.array([{h2_max}, {s_max}, {v_max}])")
        else:
            print(f"low_color  = np.array([{h1_min}, {s_min}, {v_min}])")
            print(f"high_color = np.array([{h1_max}, {s_max}, {v_max}])")
        break

cap.release()
cv2.destroyAllWindows()

