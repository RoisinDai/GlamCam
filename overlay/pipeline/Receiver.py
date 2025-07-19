import socket
import struct
import numpy as np
import cv2
import json
import threading
import queue
from Config import KINECT_HOST, KINECT_PORT, UNITY_HOST, UNITY_PORT


def receive_exact(conn, length):
    """Receive exactly 'length' bytes from the connection."""
    data = b""
    while len(data) < length:
        chunk = conn.recv(length - len(data))
        if not chunk:
            raise ConnectionError("Connection closed or incomplete data.")
        data += chunk
    return data


def receive_kinect_data():
    """Thread target for Kinect data reception."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((KINECT_HOST, KINECT_PORT))
        s.listen(1)
        print("Waiting for Kinect C# connection...")
        conn, addr = s.accept()
        print("Kinect connected by", addr)
        with conn:
            while True:
                try:
                    # 1. Read total message length
                    raw_total_len = receive_exact(conn, 4)
                    total_len = struct.unpack("<I", raw_total_len)[0]
                    if total_len <= 0 or total_len > 20_000_000:
                        print(f"Invalid or too large total message length: {total_len}")
                        continue

                    # 2. Read JPEG length and JPEG bytes
                    jpg_len = struct.unpack("<I", receive_exact(conn, 4))[0]
                    if jpg_len <= 0 or jpg_len > total_len:
                        print(f"Invalid or too large jpeg length: {jpg_len}")
                        continue
                    jpg_bytes = receive_exact(conn, jpg_len)

                    # 3. Read joints JSON length and JSON bytes
                    joints_len = struct.unpack("<I", receive_exact(conn, 4))[0]
                    if joints_len < 0 or joints_len > (total_len - 4 - jpg_len - 4):
                        print(f"Invalid or too large joints length: {joints_len}")
                        continue
                    joints_bytes = receive_exact(conn, joints_len)

                    # 4. Decode JSON
                    try:
                        joints_json = joints_bytes.decode("utf-8")
                        joints_data = json.loads(joints_json)
                    except Exception as ex:
                        print("Failed to decode joint JSON:", ex)
                        joints_data = None

                    # 5. Drain the queue to only keep the latest data
                    if kinect_queue.full():
                        try:
                            kinect_queue.get_nowait()
                        except queue.Empty:
                            pass

                    # 6. Put the latest data into the queue
                    kinect_queue.put((jpg_bytes, joints_data))

                except Exception as e:
                    print("Kinect receiver error:", e)
                    break


def receive_unity_data():
    """Thread target for Unity data reception."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((UNITY_HOST, UNITY_PORT))
        s.listen(1)
        print("Waiting for Unity connection...")
        conn, addr = s.accept()
        print("Unity connected by", addr)
        with conn:
            while True:
                try:
                    # 1. Receive Camera A
                    raw_lenA = receive_exact(conn, 4)
                    if not raw_lenA:
                        print("No data received for Camera A.")
                        continue
                    lenA = struct.unpack("<I", raw_lenA)[0]
                    imgA_bytes = receive_exact(conn, lenA)

                    # 2. Receive Camera B
                    raw_lenB = receive_exact(conn, 4)
                    if not raw_lenB:
                        print("No data received for Camera B.")
                        continue
                    lenB = struct.unpack("<I", raw_lenB)[0]
                    imgB_bytes = receive_exact(conn, lenB)

                    # 3. Drain the queue to only keep the latest data
                    if unity_queue.full():
                        try:
                            unity_queue.get_nowait()
                        except queue.Empty:
                            pass

                    # 4. Put the latest data into the queue
                    unity_queue.put((imgA_bytes, imgB_bytes))

                except Exception as e:
                    print("Unity receiver error:", e)
                    break


def get_latest_from_queue(q, last_value=None):
    """Get the latest item from the queue, or return last_value if empty."""
    item = None
    try:
        item = q.get_nowait()
        while True:
            item = q.get_nowait()
    except queue.Empty:
        return item if item is not None else last_value


def decode_frame(frame_bytes):
    """
    Decode the received frame bytes into an OpenCV image.
    :param frame_bytes: The raw frame bytes received from the Kinect C# server.
    :return: Decoded OpenCV image or None if decoding fails.
    """
    if frame_bytes is None or len(frame_bytes) == 0:
        print("No frame data received.")
        return None
    arr = np.frombuffer(frame_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img


#########################################################################################
# Main loop to receive and display data from Kinect and Unity for testing purposes
#########################################################################################


if __name__ == "__main__":
    # Queues for latest data (maxsize=1 to keep only the freshest)
    kinect_queue = queue.Queue(maxsize=1)
    unity_queue = queue.Queue(maxsize=1)

    # Start receiver threads
    threading.Thread(target=receive_kinect_data, daemon=True).start()
    threading.Thread(target=receive_unity_data, daemon=True).start()

    kinect_latest = None
    unity_latest = None

    while True:
        # Get latest Kinect data if available
        kinect_latest = get_latest_from_queue(kinect_queue, kinect_latest)
        # Get latest Unity data if available
        unity_latest = get_latest_from_queue(unity_queue, unity_latest)

        if kinect_latest is not None and unity_latest is not None:
            k_jpg_bytes, k_joints = kinect_latest
            imgA_bytes, imgB_bytes = unity_latest

            img_kinect = decode_frame(k_jpg_bytes)
            imgA = decode_frame(imgA_bytes)
            imgB = decode_frame(imgB_bytes)

            if img_kinect is not None:
                if k_joints is not None:
                    for body in k_joints:
                        for joint_name, joint_info in body.items():
                            x_raw, y_raw = joint_info.get("X", None), joint_info.get(
                                "Y", None
                            )
                            state = joint_info.get("State", "")
                            try:
                                x = float(x_raw)
                                y = float(y_raw)
                            except (TypeError, ValueError):
                                continue  # skip invalid numbers
                            # Only draw if valid and within image bounds
                            if (
                                0 <= x < img_kinect.shape[1]
                                and 0 <= y < img_kinect.shape[0]
                            ):
                                color = (
                                    (0, 255, 0) if state == "Tracked" else (0, 255, 255)
                                )
                                cv2.circle(img_kinect, (int(x), int(y)), 7, color, 2)
                    cv2.imshow("Kinect Stream", img_kinect)
            if imgA is not None:
                cv2.imshow("Unity Camera A", imgA)
            if imgB is not None:
                cv2.imshow("Unity Camera B", imgB)

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
