import cv2
import os
from datetime import datetime

# ==========================
# CHANGE THESE PATHS
# ==========================
RED_FOLDER = r"/home/pi/Desktop/Future-Engineers-2026---Kinetic-Krew/Red"
GREEN_FOLDER = r"/home/pi/Desktop/Future-Engineers-2026---Kinetic-Krew/Green"
# ==========================

os.makedirs(RED_FOLDER, exist_ok=True)
os.makedirs(GREEN_FOLDER, exist_ok=True)

# Open webcam (0 = default camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Controls:")
print("R = Save to Red")
print("G = Save to Green") 
print("Q or ESC = Quit")

red_count = len(os.listdir(RED_FOLDER))
green_count = len(os.listdir(GREEN_FOLDER))

while True:
    ret, frame = cap.read()

    if not ret:
        break

    display = frame.copy()

    cv2.putText(display, "R = RED", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.putText(display, "G = GREEN", (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.putText(display, "Q / ESC = Quit", (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    cv2.putText(display,
                f"Red: {red_count}   Green: {green_count}",
                (10, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    cv2.imshow("Dataset Collector", display)

    key = cv2.waitKey(1) & 0xFF

    # Save to RED
    if key == ord('r'):
        filename = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
        cv2.imwrite(os.path.join(RED_FOLDER, filename), frame)
        red_count += 1
        print("Saved to RED:", filename)

    # Save to GREEN
    elif key == ord('g'):
        filename = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
        cv2.imwrite(os.path.join(GREEN_FOLDER, filename), frame)
        green_count += 1
        print("Saved to GREEN:", filename)

    # Quit
    elif key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()