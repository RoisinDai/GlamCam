import socket
import struct
import numpy as np
import cv2
import json
import threading
import queue


class StreamReceiver:
    def __init__(self, host, port, parse_packet_fn, name="Receiver"):
        self.host = host
        self.port = port
        self.parse_packet_fn = parse_packet_fn
        self.name = name
        self.queue = queue.Queue(maxsize=1)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen(1)
            print(
                f"[{self.name}] Waiting for connection on {self.host}:{self.port} ..."
            )
            conn, addr = s.accept()
            print(f"[{self.name}] Connected by {addr}")
            with conn:
                while True:
                    try:
                        data = self.parse_packet_fn(conn)
                        if data is not None:
                            # Drain the queue to only keep the latest data
                            if self.queue.full():
                                try:
                                    self.queue.get_nowait()
                                except queue.Empty:
                                    pass
                            self.queue.put(data)
                    except Exception as e:
                        print(f"[{self.name}] Receiver error:", e)
                        break

    def get_latest(self, last_value=None):
        """Get the latest item from the queue, or return last_value if empty."""
        item = None
        try:
            item = self.queue.get_nowait()
            while True:
                item = self.queue.get_nowait()
        except queue.Empty:
            return item if item is not None else last_value


def receive_exact(conn, length):
    """Receive exactly 'length' bytes from the connection."""
    data = b""
    while len(data) < length:
        chunk = conn.recv(length - len(data))
        if not chunk:
            raise ConnectionError("Connection closed or incomplete data.")
        data += chunk
    return data


def parse_kinect_packet(conn):
    # 1. Read total message length
    raw_total_len = receive_exact(conn, 4)
    total_len = struct.unpack("<I", raw_total_len)[0]
    if total_len <= 0 or total_len > 20_000_000:
        print(f"Invalid or too large total message length: {total_len}")
        return None

    # 2. Read JPEG length and JPEG bytes
    jpg_len = struct.unpack("<I", receive_exact(conn, 4))[0]
    if jpg_len <= 0 or jpg_len > total_len:
        print(f"Invalid or too large jpeg length: {jpg_len}")
        return None
    jpg_bytes = receive_exact(conn, jpg_len)

    # 3. Read joints JSON length and JSON bytes
    joints_len = struct.unpack("<I", receive_exact(conn, 4))[0]
    if joints_len < 0 or joints_len > (total_len - 4 - jpg_len - 4):
        print(f"Invalid or too large joints length: {joints_len}")
        return None
    joints_bytes = receive_exact(conn, joints_len)

    # 4. Decode JSON
    try:
        joints_json = joints_bytes.decode("utf-8")
        joints_data = json.loads(joints_json)
    except Exception as ex:
        print("Failed to decode joint JSON:", ex)
        joints_data = None

    return (jpg_bytes, joints_data)


def parse_unity_packet(conn):
    # 1. Receive Camera A
    raw_lenA = receive_exact(conn, 4)
    if not raw_lenA:
        print("No data received for Camera A.")
        return None
    lenA = struct.unpack("<I", raw_lenA)[0]
    imgA_bytes = receive_exact(conn, lenA)

    # 2. Receive Camera B
    raw_lenB = receive_exact(conn, 4)
    if not raw_lenB:
        print("No data received for Camera B.")
        return None
    lenB = struct.unpack("<I", raw_lenB)[0]
    imgB_bytes = receive_exact(conn, lenB)

    return (imgA_bytes, imgB_bytes)


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
    from Config import KINECT_HOST, KINECT_PORT, UNITY_HOST, UNITY_PORT

    kinect_receiver = StreamReceiver(
        KINECT_HOST, KINECT_PORT, parse_kinect_packet, name="KinectReceiver"
    )
    unity_receiver = StreamReceiver(
        UNITY_HOST, UNITY_PORT, parse_unity_packet, name="UnityReceiver"
    )

    kinect_latest = None
    unity_latest = None

    while True:
        # Get latest Kinect data if available
        kinect_latest = kinect_receiver.get_latest(kinect_latest)
        # Get latest Unity data if available
        unity_latest = unity_receiver.get_latest(unity_latest)

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
                                    (0, 255, 0) if state == "Tracked"
                                    else (0, 255, 255)
                                )
                                cv2.circle(
                                    img_kinect, (int(x), int(y)), 7, color, 2)
                    cv2.imshow("Kinect Stream", img_kinect)
            if imgA is not None:
                cv2.imshow("Unity Camera A", imgA)
            if imgB is not None:
                cv2.imshow("Unity Camera B", imgB)

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
