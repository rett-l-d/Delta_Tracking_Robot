import socket
import struct
import time
from robodk.robolink import Robolink

# -----------------------------
# Connect to RoboDK
# -----------------------------
RDK = Robolink()
robot = RDK.Item('Fanuc SR-3iA')

if not robot.Valid():
    raise RuntimeError("Robot 'Fanuc SR-3iA' was not found in RoboDK")

# -----------------------------
# Configuration
# -----------------------------
PLC_IP = "127.0.0.1"  # CODESYS runtime IP
PLC_PORT = 2005

PACKET_SIZE = 16  # 4 x REAL (float), little-endian


def receive_packet(client, packet_size):
    """Receive exactly packet_size bytes from the socket."""
    data = bytearray()

    while len(data) < packet_size:
        chunk = client.recv(packet_size - len(data))

        if not chunk:
            raise ConnectionError("Connection closed by PLC")

        data.extend(chunk)

    return bytes(data)


def get_joint_values():
    """Return the robot's current joint positions as four floats."""
    joints = robot.Joints().list()

    if len(joints) != 4:
        raise ValueError(f"Expected 4 robot joints, got {len(joints)}")

    return [float(value) for value in joints]

def move_robot():
    while True:
        client = None

        # -----------------------------
        # Connect to PLC (Retry Forever)
        # -----------------------------
        while client is None:
            try:
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.settimeout(5)
                client.connect((PLC_IP, PLC_PORT))
                client.settimeout(None)

                print(f"Connected to {PLC_IP}:{PLC_PORT}")

            except (ConnectionRefusedError, TimeoutError, OSError) as e:
                print(f"Unable to connect ({e}). Retrying in 2 seconds...")

                if client is not None:
                    client.close()
                    client = None

                time.sleep(2)

        
        # -----------------------------
        # Receive Commands and Send Joint Feedback
        # -----------------------------
        try:
            # Read the robot's current joint positions and send them to CODESYS.
            actual_joints = get_joint_values()
            response = struct.pack("<ffff", *actual_joints)
            client.sendall(response)
            time.sleep(2)
            while True:
                data = receive_packet(client, PACKET_SIZE)
                j1, j2, j3, j4 = struct.unpack("<ffff", data)

                print(
                    f"Command: "
                    f"J1={j1:8.2f}  "
                    f"J2={j2:8.2f}  "
                    f"J3={j3:8.2f}  "
                    f"J4={j4:8.2f}"
                )

                # MoveJ waits for the commanded move to finish.
                robot.MoveJ([j1, j2, j3, j4])

                # Read the robot's current joint positions and send them to CODESYS.
                actual_joints = get_joint_values()
                response = struct.pack("<ffff", *actual_joints)
                client.sendall(response)

                print(
                    f"Feedback: "
                    f"J1={actual_joints[0]:8.2f}  "
                    f"J2={actual_joints[1]:8.2f}  "
                    f"J3={actual_joints[2]:8.2f}  "
                    f"J4={actual_joints[3]:8.2f}"
                )

        except KeyboardInterrupt:
            print("\nStopped by user")
            break

        except Exception as e:
            print("Error:", e)

        finally:
            if client is not None:
                client.close()