import socket
import struct
import numpy as np
import cv2
import json

# TCP host and port for Kinect C# connection
HOST = "127.0.0.1"
PORT = 5005


def receive_exact(conn, length):
    """Receive exactly 'length' bytes from the connection."""
    data = b""
    while len(data) < length:
        chunk = conn.recv(length - len(data))
        if not chunk:
            raise ConnectionError("Connection closed or incomplete data.")
        data += chunk
    return data


def receive_frame_and_joints(conn, max_total_len=20_000_000):
    """
    Receive a frame (JPEG) and the joint JSON data from the Kinect C# server.
    :param conn: The socket connection to the Kinect C# server.
    :param max_total_len: The maximum allowed total message length.
    :return: (frame_bytes, joints_dict) or (None, None) if error.
    """
    try:
        # 1. Read total message length
        raw_total_len = receive_exact(conn, 4)
        total_len = struct.unpack("<I", raw_total_len)[0]
        if total_len <= 0 or total_len > max_total_len:
            print(f"Invalid or too large total message length: {total_len}")
            return None, None

        # 2. Read JPEG length and JPEG bytes
        jpg_len = struct.unpack("<I", receive_exact(conn, 4))[0]
        if jpg_len <= 0 or jpg_len > total_len:
            print(f"Invalid or too large jpeg length: {jpg_len}")
            return None, None
        jpg_bytes = receive_exact(conn, jpg_len)

        # 3. Read joints JSON length and JSON bytes
        joints_len = struct.unpack("<I", receive_exact(conn, 4))[0]
        if joints_len < 0 or joints_len > (total_len - 4 - jpg_len - 4):
            print(f"Invalid or too large joints length: {joints_len}")
            return None, None
        joints_bytes = receive_exact(conn, joints_len)

        # 4. Decode JSON
        try:
            joints_json = joints_bytes.decode("utf-8")
            joints_data = json.loads(joints_json)
        except Exception as ex:
            print("Failed to decode joint JSON:", ex)
            joints_data = None

        return jpg_bytes, joints_data

    except Exception as e:
        print("Error receiving frame and joints:", e)
        return None, None


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


################################################################################
# Main Kinect Frame Receiver
################################################################################


def main():
    """
    Main function to set up the socket connection and process incoming frames and joints.
    """
    # Create a TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print("Waiting for Kinect C# connection...")
        conn, addr = s.accept()
        print("Connected by", addr)
        with conn:
            while True:
                frame_bytes, joints_data = receive_frame_and_joints(conn)
                img = decode_frame(frame_bytes)
                if img is not None:
                    # Optionally draw joints on the image (as circles)
                    if joints_data:
                        for body in joints_data:
                            for joint_name, joint_info in body.items():
                                x_raw, y_raw = joint_info.get(
                                    "X", None
                                ), joint_info.get("Y", None)
                                state = joint_info.get("State", "")
                                try:
                                    x = float(x_raw)
                                    y = float(y_raw)
                                except (TypeError, ValueError):
                                    continue  # skip invalid numbers
                                # Only draw if valid and within image bounds
                                if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
                                    color = (
                                        (0, 255, 0)
                                        if state == "Tracked"
                                        else (0, 255, 255)
                                    )
                                    cv2.circle(img, (int(x), int(y)), 7, color, 2)
                    cv2.imshow("Kinect Stream", img)
                else:
                    print("Failed to decode frame.")
                # Stop on ESC key
                if cv2.waitKey(1) == 27:
                    break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
