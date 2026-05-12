import numpy as np
import cv2
import time
import os
from datetime import datetime
from robodk.robolink import Robolink

RDK = Robolink()
cam = RDK.Item('Camera 1')

SNAPSHOT_PATH = 'C:/temp/snapshot.png'
SAVE_DIR      = 'C:/temp/detections/'
os.makedirs(SAVE_DIR, exist_ok=True)

cam.setParam('Open', 1)

YELLOW_LOW  = np.array([20,  100, 100])
YELLOW_HIGH = np.array([35, 255, 255])

while True:
    RDK.Cam2D_Snapshot(SNAPSHOT_PATH, cam)
    time.sleep(0.1)

    img = cv2.imread(SNAPSHOT_PATH)
    if img is None:
        time.sleep(0.1)
        continue

    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Pick the contour with the largest area (highest confidence)
    best = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100 and area > best_area:
            best = cnt
            best_area = area

    if best is not None:
        peri   = cv2.arcLength(best, True)
        approx = cv2.approxPolyDP(best, 0.04 * peri, True)

        x, y, w, h = cv2.boundingRect(best)
        cx = x + w // 2
        cy = y + h // 2
        print(f"Rectangle found: center=({cx}, {cy})  size={w}x{h}px  top-left=({x}, {y})")

        output = img.copy()
        cv2.drawContours(output, [approx], -1, (0, 255, 0), 2)
        cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        save_path = f"{SAVE_DIR}detection_{timestamp}.png"
        cv2.imwrite(save_path, output)
        #print(f"Saved: {save_path}")

    time.sleep(0.1)