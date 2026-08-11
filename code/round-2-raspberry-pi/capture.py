import cv2
import os

# -------------------------------------------------------------------
# Camera settings (Windows DirectShow)
# -------------------------------------------------------------------
CAMERA_INDEX = 1

# Exposure
MIN_EXPOSURE = -10
MAX_EXPOSURE = -1
DEFAULT_EXPOSURE = -6

# White balance
DEFAULT_WB_TEMP = 4500        # Kelvin (if camera supports manual temperature)
MIN_WB_TEMP = 2500
MAX_WB_TEMP = 8000
WB_STEP = 200

# Flags
auto_exposure = False         # we set manual exposure in setup
auto_wb = False               # we start with manual WB

# -------------------------------------------------------------------
# Helper functions
# -------------------------------------------------------------------
def set_manual_exposure(cap, value):
    """Turn off auto exposure and set absolute exposure value."""
    # Some cameras use -1...-10, some use 1...1000. We'll just send the value.
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)          # 0 = manual
    cap.set(cv2.CAP_PROP_EXPOSURE, value)
    return value

def get_exposure(cap):
    return cap.get(cv2.CAP_PROP_EXPOSURE)

def set_manual_wb_temp(cap, temp):
    """Set white balance temperature (in Kelvin) if supported.
       Falls back to manually adjusting Blue/Red gains if temperature isn't available."""
    # First, try using temperature
    if cap.set(cv2.CAP_PROP_WB_TEMPERATURE, temp):
        return temp, True
    # If not supported, try direct Blue/Red channel gains
    # The camera might use integer values; we approximate a range
    blue_val = int(2000 * (temp / 4000))   # crude mapping – adjust if needed
    red_val = int(2000 * (4000 / temp))
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)       # manual mode
    cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, blue_val)
    cap.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, red_val)
    return temp, False   # False indicates we used channel gains

def get_wb_temp(cap):
    """Try to read current WB temperature; fallback to reading Blue/Red gains."""
    temp = cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
    if temp != -1:   # supported
        return temp
    # Fallback: estimate temperature from Blue/Red gains (very rough)
    blue = cap.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U)
    red = cap.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V)
    if blue > 0 and red > 0:
        # Simple guess: if red > blue, warmer; else cooler
        return int(4000 * (red / blue))
    return DEFAULT_WB_TEMP

def set_auto_wb(cap, enable):
    """Enable/disable auto white balance."""
    if enable:
        cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    else:
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# -------------------------------------------------------------------
# Create output folders
# -------------------------------------------------------------------
os.makedirs("red", exist_ok=True)
os.makedirs("green", exist_ok=True)
os.makedirs("magenta", exist_ok=True)

# -------------------------------------------------------------------
# Open camera
# -------------------------------------------------------------------
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Initial manual settings
current_exposure = set_manual_exposure(cap, DEFAULT_EXPOSURE)
set_auto_wb(cap, False)   # start with manual WB
current_wb_temp = set_manual_wb_temp(cap, DEFAULT_WB_TEMP)[0]

red_count = len(os.listdir("red"))
green_count = len(os.listdir("green"))
magenta_count = len(os.listdir("magenta"))

print("=== Photo Capture with Exposure & White Balance ===")
print("  'r' = save RED cuboid photo")
print("  'g' = save GREEN cuboid photo")
print("  'm' = save MAGENTA (parking marker) photo")
print("  'e' = increase exposure (+0.5)")
print("  'd' = decrease exposure (-0.5)")
print("  'w' = toggle Auto White Balance on/off")
print("  'u' = warmer white balance (+200K)")
print("  'j' = cooler white balance (-200K)")
print("  'q' = quit")
print(f"  Exposure: {current_exposure:.1f} | WB: {current_wb_temp}K | AutoWB: {auto_wb}")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Read current values (for display)
    current_exposure = get_exposure(cap)
    current_wb_temp = get_wb_temp(cap) if not auto_wb else 0

    # Overlay info
    line1 = f"Exp: {current_exposure:.1f}  |  WB: {current_wb_temp}K  |  AutoWB: {'ON' if auto_wb else 'OFF'}"
    line2 = f"Red: {red_count}  Green: {green_count}  Magenta: {magenta_count}"
    cv2.putText(frame, line1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
    cv2.putText(frame, line1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
    cv2.putText(frame, line2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
    cv2.putText(frame, line2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
    cv2.imshow("Capture", frame)

    key = cv2.waitKey(1) & 0xFF

    # --- Photo capture ---
    if key == ord('r'):
        filename = f"red/red_{red_count+1:04d}.jpg"
        cv2.imwrite(filename, frame)
        red_count += 1
        print(f"Saved {filename} (total red: {red_count})")

    elif key == ord('g'):
        filename = f"green/green_{green_count+1:04d}.jpg"
        cv2.imwrite(filename, frame)
        green_count += 1
        print(f"Saved {filename} (total green: {green_count})")

    elif key == ord('m'):
        filename = f"magenta/magenta_{magenta_count+1:04d}.jpg"
        cv2.imwrite(filename, frame)
        magenta_count += 1
        print(f"Saved {filename} (total magenta: {magenta_count})")

    # --- Exposure control ---
    elif key == ord('e'):
        current_exposure = set_manual_exposure(cap, current_exposure + 0.5)
        print(f"Exposure increased to {current_exposure:.1f}")

    elif key == ord('d'):
        current_exposure = set_manual_exposure(cap, current_exposure - 0.5)
        print(f"Exposure decreased to {current_exposure:.1f}")

    # --- White balance control ---
    elif key == ord('w'):
        auto_wb = not auto_wb
        set_auto_wb(cap, auto_wb)
        if not auto_wb:
            # Restore last manual temperature
            current_wb_temp = set_manual_wb_temp(cap, current_wb_temp)[0]
        print(f"Auto WB {'ON' if auto_wb else 'OFF'}")

    elif key == ord('u'):
        if not auto_wb:
            current_wb_temp = min(MAX_WB_TEMP, current_wb_temp + WB_STEP)
            current_wb_temp, _ = set_manual_wb_temp(cap, current_wb_temp)
            print(f"WB temperature increased to {current_wb_temp}K")

    elif key == ord('j'):
        if not auto_wb:
            current_wb_temp = max(MIN_WB_TEMP, current_wb_temp - WB_STEP)
            current_wb_temp, _ = set_manual_wb_temp(cap, current_wb_temp)
            print(f"WB temperature decreased to {current_wb_temp}K")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"\nDone! Red: {red_count} photos, Green: {green_count} photos, Magenta: {magenta_count} photos")