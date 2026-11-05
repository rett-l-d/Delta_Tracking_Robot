
import numpy as np
import cv2
import time
import os
import socket
import struct

from datetime import datetime
from robodk import robolink


# ============================================================
# ROBO​DK CAMERA CONFIGURATION
# ============================================================

RDK = robolink.Robolink()

cam = RDK.Item(
    'Camera 1',
    robolink.ITEM_TYPE_CAMERA
)

if not cam.Valid():
    raise Exception('Camera 1 not found in RoboDK.')

SNAPSHOT_PATH = 'C:/temp/snapshot.png'

SAVE_DIR = 'C:/temp/detections/'

os.makedirs(SAVE_DIR, exist_ok=True)

cam.setParam('Open', 1)


# ============================================================
# PLC TCP CONFIGURATION
# ============================================================

PLC_IP = '127.0.0.1'
PLC_PORT = 5000

CONNECT_TIMEOUT = 3.0
RECEIVE_TIMEOUT = 0.05

plc_socket = None
plc_rx_buffer = bytearray()

last_connection_attempt = 0

# Five REAL values are sent to PLC:
# ReadData[0] = X pixel
# ReadData[1] = Y pixel
# ReadData[2] = Encoder value (placeholder)
# ReadData[3] = Detection flag
# ReadData[4] = Object ID (example)


# ============================================================
# PLC CONNECTION
# ============================================================

def connect_plc():

    global plc_socket
    global plc_rx_buffer
    global last_connection_attempt

    if plc_socket is not None:
        return True

    # Avoid repeated connection attempts every camera frame.
    if time.time() - last_connection_attempt < 2:
        return False

    last_connection_attempt = time.time()

    sock = None

    try:

        print(
            f'Connecting to PLC {PLC_IP}:{PLC_PORT}...'
        )

        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        sock.settimeout(CONNECT_TIMEOUT)

        sock.connect((PLC_IP, PLC_PORT))

        # Short timeout for continuous OpenCV operation.
        sock.settimeout(RECEIVE_TIMEOUT)

        plc_socket = sock
        plc_rx_buffer.clear()

        print('PLC TCP CONNECTION SUCCESSFUL!')

        return True

    except Exception as error:

        print('PLC connection failed:', repr(error))

        if sock is not None:

            try:
                sock.close()
            except Exception:
                pass

        plc_socket = None

        return False


# ============================================================
# PLC DISCONNECTION
# ============================================================

def disconnect_plc():

    global plc_socket
    global plc_rx_buffer

    if plc_socket is not None:

        try:
            plc_socket.close()
        except Exception:
            pass

    plc_socket = None
    plc_rx_buffer.clear()

    print('PLC connection closed.')


# ============================================================
# SEND FIVE REAL VALUES TO PLC
# ============================================================

def send_to_plc(values):

    global plc_socket

    if plc_socket is None:
        return False

    if len(values) != 5:

        raise ValueError(
            'PLC expects exactly five REAL values.'
        )

    try:

        # Five REAL values = 5 x 4 bytes = 20 bytes.
        # Little-endian IEEE-754 format.
        payload = struct.pack(
            '<5f',
            *[float(value) for value in values]
        )

        plc_socket.sendall(payload)

        print(
            'Sent to PLC:',
            [round(value, 3) for value in values]
        )

        return True

    except Exception as error:

        print('PLC SEND ERROR:', repr(error))

        disconnect_plc()

        return False


# ============================================================
# RECEIVE FOUR REAL VALUES FROM PLC
# ============================================================

def receive_from_plc():

    global plc_socket
    global plc_rx_buffer

    if plc_socket is None:
        return None

    try:

        # Receive available data without blocking OpenCV.
        try:

            data = plc_socket.recv(1024)

            if not data:

                raise ConnectionError(
                    'PLC disconnected.'
                )

            plc_rx_buffer.extend(data)

        except socket.timeout:

            return None

        # One PLC packet = four REAL values = 16 bytes.
        if len(plc_rx_buffer) >= 16:

            packet = bytes(
                plc_rx_buffer[:16]
            )

            del plc_rx_buffer[:16]

            values = struct.unpack(
                '<4f',
                packet
            )

            values = list(values)

            print(
                'PLC values:',
                [round(value, 3) for value in values]
            )

            return values

        return None

    except Exception as error:

        print('PLC RECEIVE ERROR:', repr(error))

        disconnect_plc()

        return None


# ============================================================
# CAMERA DETECTION CONFIGURATION
# ============================================================

YELLOW_LOW = np.array(
    [20, 100, 100]
)

YELLOW_HIGH = np.array(
    [35, 255, 255]
)

MIN_CONTOUR_AREA = 100

def start_camera_detection():
    # ============================================================
    # MAIN PROGRAM
    # ============================================================

    print('Starting RoboDK OpenCV camera...')

    connect_plc()

    try:

        while True:

            # ----------------------------------------------------
            # PLC CONNECTION / RECONNECTION
            # ----------------------------------------------------

            if plc_socket is None:

                connect_plc()

            # ----------------------------------------------------
            # CAMERA SNAPSHOT
            # ----------------------------------------------------

            RDK.Cam2D_Snapshot(
                SNAPSHOT_PATH,
                cam
            )

            time.sleep(0.1)

            img = cv2.imread(
                SNAPSHOT_PATH
            )

            if img is None:

                print('Camera image not available.')

                receive_from_plc()

                time.sleep(0.1)

                continue

            # ----------------------------------------------------
            # HSV CONVERSION
            # ----------------------------------------------------

            hsv = cv2.cvtColor(
                img,
                cv2.COLOR_BGR2HSV
            )

            mask = cv2.inRange(
                hsv,
                YELLOW_LOW,
                YELLOW_HIGH
            )

            # ----------------------------------------------------
            # CONTOUR DETECTION
            # ----------------------------------------------------

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            best = None
            best_area = 0

            for cnt in contours:

                area = cv2.contourArea(cnt)

                if (
                    area > MIN_CONTOUR_AREA
                    and area > best_area
                ):

                    best = cnt
                    best_area = area

            output = img.copy()

            # ----------------------------------------------------
            # PROCESS DETECTED RECTANGLE
            # ----------------------------------------------------

            if best is not None:

                peri = cv2.arcLength(
                    best,
                    True
                )

                approx = cv2.approxPolyDP(
                    best,
                    0.04 * peri,
                    True
                )

                x, y, w, h = cv2.boundingRect(
                    best
                )

                cx = x + w // 2
                cy = y + h // 2

                print(
                    f'Rectangle found: '
                    f'center=({cx}, {cy}) '
                    f'size={w}x{h}px '
                    f'top-left=({x}, {y})'
                )

                # Draw contour
                cv2.drawContours(
                    output,
                    [approx],
                    -1,
                    (0, 255, 0),
                    2
                )

                # Draw bounding rectangle
                cv2.rectangle(
                    output,
                    (x, y),
                    (x + w, y + h),
                    (255, 0, 0),
                    2
                )

                # Draw centre
                cv2.circle(
                    output,
                    (cx, cy),
                    5,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    output,
                    f'Center: {cx}, {cy}',
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                # ------------------------------------------------
                # SEND DETECTION TO PLC
                # ------------------------------------------------

                if plc_socket is not None:

                    detection_data = [
                        float(cx),
                        float(cy),
                        0.0,
                        1.0,
                        1.0
                    ]

                    send_to_plc(
                        detection_data
                    )

            # ----------------------------------------------------
            # RECEIVE PLC DATA
            # ----------------------------------------------------

            plc_values = receive_from_plc()

            # ----------------------------------------------------
            # SAVE DETECTION IMAGE
            # ----------------------------------------------------

            if best is not None:

                timestamp = datetime.now().strftime(
                    '%Y%m%d_%H%M%S_%f'
                )

                save_path = (
                    f'{SAVE_DIR}detection_{timestamp}.png'
                )

                cv2.imwrite(
                    save_path,
                    output
                )

            # ----------------------------------------------------
            # DISPLAY CAMERA
            # ----------------------------------------------------

            cv2.imshow(
                'RoboDK OpenCV Detection',
                output
            )

            cv2.imshow(
                'Yellow Detection Mask',
                mask
            )

            if cv2.waitKey(1) & 0xFF == ord('q'):

                break


    except KeyboardInterrupt:

        print('OpenCV program stopped.')


    finally:

        disconnect_plc()

        cam.setParam('Open', 0)

        cv2.destroyAllWindows()

        print('Camera and PLC connection closed.')
