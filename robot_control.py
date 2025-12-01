import time
import numpy as np
import dynamixel_sdk as dxl
from ik import inverse_kinematics

# Dynamixel Configuration
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED  = 32

PROTOCOL_VERSION = 1.0
DXL_IDS = [1, 2, 3, 4]
DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 1000000
TORQUE_ENABLE = 1

# Servo Calibration Constants
TICKS_PER_DEG = 1023.0 / 300.0

JOINT_ZERO_POS = {
    1: 510,  # Base facing forward
    2: 683,  # Shoulder Upright
    3: 98,   # Elbow
    4: 217,  # Wrist
}

JOINT_SIGNS = {
    1:  -1, # +Angle = CCW (usually)
    2: 1,   # +Angle = Forward/Down
    3: 1,   # +Angle = Down
    4:  +1  # +Angle = Up
}

class RobotArm:
    def __init__(self):
        # Initialize port and packet handler
        self.portHandler = dxl.PortHandler(DEVICENAME)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
        
        # Open port and set baud rate
        if not self.portHandler.openPort():
            raise RuntimeError("Failed to open the port")
        
        if not self.portHandler.setBaudRate(BAUDRATE):
            raise RuntimeError("Failed to change the baudrate")
        
        # Enable Torque and set speed
        for DXL_ID in DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 100)

    def angle_to_servo(self, joint_id, angle_deg):
        """
        Converts a geometric angle (degrees) to servo ticks (0-1023).
        """
        zero_tick = JOINT_ZERO_POS[joint_id]
        sign = JOINT_SIGNS[joint_id]
        
        # Linear mapping
        target_tick = zero_tick + (angle_deg * TICKS_PER_DEG * sign)
        
        return int(np.clip(target_tick, 0, 1023))

    def move_robot(self, angles_deg):
        """ Moves all 4 motors """
        print(f"   >>> Moving to angles: {angles_deg}")
        
        # Convert to servo ticks
        servos = []
        for i, angle in enumerate(angles_deg):
            joint_id = i + 1
            tick = self.angle_to_servo(joint_id, angle)
            servos.append(tick)
            
        print(f"   >>> Servo Ticks: {servos}")
        
        for i, tick in enumerate(servos):
            dxl_id = i + 1
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_GOAL_POSITION, tick)
        
        time.sleep(2.0) # Wait for move to complete

    def go_home(self):
        """Move robot to home/zero position"""
        print("[INFO] Going Home...")
        # Directly write the Zero positions
        for jid, tick in JOINT_ZERO_POS.items():
            self.packetHandler.write2ByteTxRx(self.portHandler, jid, ADDR_MX_GOAL_POSITION, tick)
        time.sleep(1.5)

    def lift_and_home(self, x, y, z, lift_mm=-40):
        """
        After picking, lift the arm straight up by lift_mm
        then go home.
        """
        z_lifted = z + lift_mm
        try:
            lift_angles = inverse_kinematics(x, y, z_lifted)
            print(f"[INFO] Lifting by {lift_mm} mm...")
            self.move_robot(lift_angles)
        except Exception as e:
            print(f"[WARNING] Lift failed: {e}")

        self.go_home()

    def close(self):
        """Close the port handler"""
        self.portHandler.closePort()