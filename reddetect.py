import cv2
import numpy as np

# --- Load image ---
img = cv2.imread("mmimage.jpeg")   # replace with your filename
if img is None:
    raise ValueError("Could not load image!")

# --- Convert to HSV ---
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# --- Red HSV thresholds (two ranges) ---
low1  = np.array([0, 120, 50])
high1 = np.array([10, 255, 255])

low2  = np.array([170, 120, 50])
high2 = np.array([180, 255, 255])

mask1 = cv2.inRange(hsv, low1, high1)
mask2 = cv2.inRange(hsv, low2, high2)
mask = cv2.bitwise_or(mask1, mask2)

# --- Morphology to reduce noise ---
kernel = np.ones((5, 5), np.uint8)
mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# --- Find contours ---
contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

detected = img.copy()

# --- Draw circles around detected smarties ---
for c in contours:
    area = cv2.contourArea(c)
    if area < 50:   # ignore tiny blobs
        continue

    (x, y), radius = cv2.minEnclosingCircle(c)
    center = (int(x), int(y))
    radius = int(radius)

    # Draw detection
    cv2.circle(detected, center, radius, (0, 255, 0), 2)
    cv2.circle(detected, center, 2, (0, 255, 0), -1)

# --- Show results ---
cv2.imshow("Original", img)
cv2.imshow("Red Mask", mask_clean)
cv2.imshow("Detected Red Smarties", detected)
cv2.waitKey(0)
cv2.destroyAllWindows()

