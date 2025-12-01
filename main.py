import cv2
import time
from red_detection import detect_red_mms
from coordinate_transform import pixel_to_world
from robot_control import RobotArm
from ik import inverse_kinematics

def main():
    # Initialize robot arm
    try:
        robot = RobotArm()
    except Exception as e:
        print(f"[ERROR] Failed to initialize robot arm: {e}")
        return

    # Go to home position initially
    robot.go_home()

    # Open camera
    cap = cv2.VideoCapture(2)  # Adjust camera index if needed
    time.sleep(1)

    print("\nPress 'p' to Pick Red M&Ms, 'ESC' to Quit.")

    try:
        while True:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Failed to capture frame")
                break
            
            # Detect red M&Ms
            centers = detect_red_mms(frame)
            
            # Draw detected M&M centers
            for (x,y) in centers:
                cv2.circle(frame, (x,y), 5, (0,255,0), -1)
                
            cv2.imshow("Vision", frame)
            key = cv2.waitKey(1)
            
            if key == ord('p'):
                print(f"\n[INFO] Detected {len(centers)} red M&Ms.")
                for i, (u, v) in enumerate(centers):
                    print(f"\n--- Target {i+1} ---")
                    
                    # Convert pixel to world coordinates
                    xw, yw, zw = pixel_to_world(u, v)
                    print(f"   Pixel: ({u}, {v}) -> World: ({xw:.1f}, {yw:.1f}, {zw:.1f})")
                    
                    try:
                        # Calculate inverse kinematics
                        angles = inverse_kinematics(xw, yw, zw)
                        
                        # Move robot to target
                        robot.move_robot(angles)
                        print("   Action Complete.")
                        
                        # Lift and return home
                        robot.lift_and_home(xw, yw, zw)
                    except Exception as e:
                        print(f"   [ERROR] Movement failed: {e}")
                
            if key == 27:  # ESC key
                break

    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        robot.close()

if __name__ == "__main__":
    main()