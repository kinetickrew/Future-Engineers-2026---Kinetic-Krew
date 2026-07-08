import cv2
import numpy as np
import onnxruntime as ort
import time

MODEL_PATH  = '/Users/KanavC/Desktop/Future Engineers 2026 - Kinetic Krew/Python/best_yolo26m.onnx'
IMG_SIZE    = 640

# 1. Force the absolute simplest session initialization
print("Initializing ONNX Session...")
session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
input_name = session.get_inputs()[0].name

# 2. Open camera and FORCE low resolution at the hardware level
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("\n--- Diagnostic Loop Running ---")
print("Press 'q' to quit.\n")

while True:
    start_time = time.time()

    # Measure Camera Read Speed
    t0 = time.time()
    ret, frame = cap.read()
    if not ret:
        break
    cam_time = (time.time() - t0) * 1000

    # Measure Preprocess Speed
    t1 = time.time()
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    inp = np.expand_dims(img, axis=0)
    prep_time = (time.time() - t1) * 1000

    # Measure AI Inference Speed
    t2 = time.time()
    outputs = session.run(None, {input_name: inp})
    inf_time = (time.time() - t2) * 1000

    # Calculate FPS
    total_time = time.time() - start_time
    fps = 1.0 / total_time if total_time > 0 else 0

    # Print exact breakdown so we know what is lagging
    print(f"FPS: {fps:4.1f} | Cam: {cam_time:3.0f}ms | Prep: {prep_time:3.0f}ms | Model: {inf_time:3.0f}ms", end="\r")

    # Simple display without any processing math
    cv2.imshow("Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()