import time
import cv2
import numpy as np
import dynamixel_sdk as dxl

# ---------------------------
# DYNAMIXEL CONFIG
# ---------------------------
ADDR_MX_TORQUE_ENABLE       = 24
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_MOVING_SPEED        = 32
PROTOCOL_VERSION = 1.0
DXL_IDS = [1, 2, 3, 4]

DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 1000000
TORQUE_ENABLE = 1

# Setup
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# Enable torque
for DXL_ID in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 100)

# ---------------------------
# INIT FUNCTION
# ---------------------------
def init():
    JOINT3_POS = 300     # 45 degrees
    JOINT4_DOWN = 220    # downward camera pose

    print("Initializing arm...")
    packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_GOAL_POSITION, JOINT3_POS)
    packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_GOAL_POSITION, JOINT4_DOWN)
    time.sleep(1.5)

# ---------------------------
# DETECT RED M&Ms
# ---------------------------
def detect_red_mms(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower1 = np.array([0, 100, 50])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 100, 50])
    upper2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    red_centers = []
    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        (x, y), radius = cv2.minEnclosingCircle(c)
        
        if radius > 5 and radius < 50 and area > 20 and circularity > 0.7:
            red_centers.append((int(x), int(y)))
    
    return red_centers

# ---------------------------
# MOVE ABOVE M&M
# ---------------------------
def move_to_mnm(dot_id, x, y):
    print(f"[INFO] Moving to M&M {dot_id} at pixel ({x},{y})")

    # TODO: Replace with your real mapping
    motor_pos_1 = 200 + (x // 4)
    motor_pos_2 = 300 + (y // 4)

    # Go to M&M
    packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_GOAL_POSITION, motor_pos_1)
    packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_GOAL_POSITION, motor_pos_2)

    time.sleep(1)

    print(f"[INFO] Hovering above M&M {dot_id}")

    time.sleep(1)

# Return to a safe home pose
def go_home():
    HOME1 = 512
    HOME2 = 512

    packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_GOAL_POSITION, HOME1)
    packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_GOAL_POSITION, HOME2)

    print("[INFO] Returned Home")
    time.sleep(1)

# ---------------------------
# MAIN
# ---------------------------
init()
time.sleep(2)

cap = cv2.VideoCapture(2)
print("Camera warming up…")
time.sleep(2)

detect_mode = True      # Detect until 'P' is pressed
saved_centers = []      # Saved M&M coordinates after pressing P

print("\nLIVE MODE:")
print(" - Detecting red M&Ms")
print(" - Press 'P' to STOP detection and start moving")
print(" - Press ESC to exit\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    key = cv2.waitKey(1) & 0xFF

    # ---------------------------------------------------------
    # DETECTION MODE (before pressing P)
    # ---------------------------------------------------------
    if detect_mode:
        centers = detect_red_mms(frame)

        for i, (x, y) in enumerate(centers, 1):
            cv2.circle(frame, (x, y), 10, (0,255,0), 2)
            cv2.putText(frame, f"M&M{i}", (x+15, y+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        cv2.imshow("Camera", frame)

        if key == ord('p'):
            print("\n[INFO] STOPPING DETECTION MODE")
            detect_mode = False
            saved_centers = centers.copy()
            print("Saved M&M positions:", saved_centers)
            print("\nCamera will now only display video.")
            continue

    # ---------------------------------------------------------
    # AFTER PRESSING P → NO DETECTION, ONLY CAMERA FEED
    # ---------------------------------------------------------
    else:
        cv2.imshow("Camera", frame)

        # Move to each M&M once
        for i, (x, y) in enumerate(saved_centers, 1):
            move_to_mnm(i, x, y)
            go_home()

        # After moving once, clear list so it doesn't repeat
        saved_centers = []

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()

for DXL_ID in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)

portHandler.closePort()

