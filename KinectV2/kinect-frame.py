import socket
import struct
import numpy as np
import cv2

# TCP host and port for Kinect C# connection
HOST = "127.0.0.1"
PORT = 5005


def receive_frame(conn, max_len=10_000_000):
    """
    Receive a frame from the Kinect C# server.
    :param conn: The socket connection to the Kinect C# server.
    :param max_len: The maximum allowed frame length.
    :return: The received frame data or None if an error occurred.
    """

    # Receive 4 bytes for the frame size
    raw_len = b""
    while len(raw_len) < 4:
        chunk = conn.recv(4 - len(raw_len))
        if not chunk:
            print("Failed to receive frame length.")
            return None
        raw_len += chunk

    frame_len = struct.unpack("<I", raw_len)[0]
    if frame_len <= 0 or frame_len > max_len:
        print(f"Invalid or too large frame length: {frame_len}")
        return None

    # Receive the actual frame
    frame_data = b""
    while len(frame_data) < frame_len:
        chunk = conn.recv(frame_len - len(frame_data))
        if not chunk:
            print("Connection closed while receiving frame data.")
            return None
        frame_data += chunk

    return frame_data


def decode_frame(frame_bytes):
    """
    Decode the received frame bytes into an OpenCV image.
    :param frame_bytes: The raw frame bytes received from the Kinect C# server.
    :return: Decoded OpenCV image or None if decoding fails.
    """

    if frame_bytes is None or len(frame_bytes) == 0:
        print("No frame data received.")
        return None

    # Decode JPEG to image
    arr = np.frombuffer(frame_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img


################################################################################
# Testing the Kinect Frame Receiver
################################################################################


def main():
    """
    Main function to set up the socket connection and process incoming frames.
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
                frame_bytes = receive_frame(conn)
                img = decode_frame(frame_bytes)
                if img is not None:
                    cv2.imshow("Kinect Stream", img)
                else:
                    print("Failed to decode frame.")
                # Stop on ESC key
                if cv2.waitKey(1) == 27:
                    break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
