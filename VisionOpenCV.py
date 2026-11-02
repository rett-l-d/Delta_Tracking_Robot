from json import encoder

import numpy as np
import cv2
import time
import os
import threading
from datetime import datetime
from robodk.robolink import Robolink
from robodk.robomath import *
import robolink

# ─────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────

CAMERA_OFFSET   = 747.0     # mm upstream from robot pick point
CAMERA_X_ROBOT = -747.0
CAMERA_Y_ROBOT = 0.0
PICK_WINDOW_MM  = 50.0      # ±mm around pick point where robot can reach

FOV_W_MM        = 261.5     # calculated from camera params
FOV_H_MM        = 196.1
PX_PER_MM_X     = 640 / FOV_W_MM   # ~2.45 px/mm
PX_PER_MM_Y     = 480 / FOV_H_MM

SNAPSHOT_PATH   = 'C:/temp/snapshot.png'
SAVE_DIR        = 'C:/temp/detections/'
os.makedirs(SAVE_DIR, exist_ok=True)

YELLOW_LOW      = np.array([20, 100, 100])
YELLOW_HIGH     = np.array([35, 255, 255])

PICK_Z          = -350.0    # Z depth to pick (mm) — adjust to your setup
SAFE_Z          =  100.0    # Z safe travel height (mm)

MIN_ENCODER_DELTA = 30    # mm of belt travel before same object can be re-detected



# ─────────────────────────────────────────────
# ROBODK SETUP
# ─────────────────────────────────────────────

RDK   = Robolink()
cam   = RDK.Item('Camera 1')
robot = RDK.Item('Fanuc M-1iA/0.5SL', robolink.ITEM_TYPE_ROBOT)

cam.setParam('Open', 1)



# ─────────────────────────────────────────────
# DETECTION LOOP
# ─────────────────────────────────────────────


#print(f"px/mm X: {PX_PER_MM_X:.2f}  Y: {PX_PER_MM_Y:.2f}")
#print(f"Camera offset: {CAMERA_OFFSET}mm  |  Pick window: ±{PICK_WINDOW_MM}mm")
#print("Running — Ctrl+C to stop\n")

def getOpenCVresults(shared, queue_lock, object_queue):
   

    #####FUTURE ADD DETECTION NEAR CENTER OF fov ONLY, TO AVOID PARTIAL VIEWS

    SNAP_DISTANCE_MM = 300
    SCALE = 15.2  # mm per encoder unit
    last_encoder = 0

    while True:

        encoder_now = shared['encoder_value']


        # # Debounce — skip if same object detected again within MIN_ENCODER_DELTA
        # if abs(encoder_now - last_encoder) < 5:
        #     continue

        time.sleep(0.1)
 
        RDK.Cam2D_Snapshot(SNAPSHOT_PATH, cam)
        img = cv2.imread(SNAPSHOT_PATH)
        if img is None:
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

            box_x, box_y, box_w, box_h = cv2.boundingRect(best)

            IMAGE_W = img.shape[1]
            IMAGE_H = img.shape[0]

            EDGE_MARGIN = 5

            touching_edge = (
                box_x <= EDGE_MARGIN or
                box_y <= EDGE_MARGIN or
                (box_x + box_w) >= IMAGE_W - EDGE_MARGIN or
                (box_y + box_h) >= IMAGE_H - EDGE_MARGIN
            )

            if touching_edge:
              #  print("Partial rectangle rejected")
              
                continue

            peri = cv2.arcLength(best, True)
            approx = cv2.approxPolyDP(best, 0.04 * peri, True)

            if len(approx) < 4 or len(approx) > 10:
                print("Not rectangle-like")
                continue
            
            rect = cv2.minAreaRect(best)
            (center_x, center_y), _, angle = rect
            cx_px = int(center_x)
            cy_px = int(center_y)

            cx_mm = (cx_px - 320) / PX_PER_MM_X
            cy_mm = (cy_px - 240) / PX_PER_MM_Y

            x_robot = CAMERA_X_ROBOT + cx_mm
            y_robot = -1 * (CAMERA_Y_ROBOT + cy_mm)

        


           
                
            is_new_object = True

            

            for existing in object_queue:

                dy = abs(existing['y_at_detection'] - y_robot)

                dangle = abs(existing['rotation'] - angle)

                if dy < 15 and dangle < 15:
                    is_new_object = False
                    continue

            # print(f"Detected: pixel=({cx_px},{cy_px})  mm=({cx_mm:.1f},{cy_mm:.1f})  encoder={encoder_now:.1f}")
            # print(
            #         f"Detected: "
            #         f"robot=({x_robot:.1f},{y_robot:.1f}) "
            #         f"encoder={encoder_now:.1f}"
            #     )
            if is_new_object:
                # Add to queue with position and encoder stamp
                with queue_lock:
                    object_queue.append({
                        'x_at_detection': x_robot,
                        'y_at_detection': y_robot,
                        'encoder_at_detection': encoder_now,
                        'rotation': angle,
                        'timestamp': time.time()
                    })

            #     # Save annotated snapshot
                # output = img.copy()
                # cv2.drawContours(output, [approx], -1, (0, 255, 0), 2)
                # cv2.circle(output, (cx_px, cy_px), 5, (0, 0, 255), -1)

                # timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                # save_path = f"{SAVE_DIR}detection_{timestamp}.png"
                # cv2.imwrite(save_path, output)

            # print(len(object_queue))


        last_encoder = encoder_now
       # return last_encoder