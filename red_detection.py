import cv2
import numpy as np

def detect_red_mms(frame):
    """
    Detect red M&Ms in the given frame.
    
    Args:
        frame (numpy.ndarray): Input image frame
    
    Returns:
        list: List of (x, y) pixel coordinates of red M&M centers
    """
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define color ranges for red (two ranges due to hsv wraparound)
    lower1 = np.array([0, 100, 50])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 100, 50])
    upper2 = np.array([180, 255, 255])
    
    # Create masks for red color ranges
    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Morphological operations to remove noise
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter and store centers of red M&Ms
    red_centers = []
    for c in contours:
        # Filter by area to remove very small or very large contours
        if cv2.contourArea(c) > 20:
            (x, y), r = cv2.minEnclosingCircle(c)
            if r > 5:
                red_centers.append((int(x), int(y)))
    
    return red_centers