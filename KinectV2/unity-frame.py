import socket
import struct
import numpy as np
import cv2

HOST = "127.0.0.1"
PORT = 5005


def receive_exact(conn, length):
    """Receive exactly length bytes from the connection."""
    data = b""
    while len(data) < length:
        chunk = conn.recv(length - len(data))
        if not chunk:
            raise ConnectionError("Connection closed.")
        data += chunk
    return data


def receive_image(conn):
    """Receive a single image (4-byte length prefix + image bytes) and decode to cv2 image."""
    raw_len = receive_exact(conn, 4)
    if not raw_len:
        return None
    img_len = struct.unpack("<I", raw_len)[0]
    img_bytes = receive_exact(conn, img_len)
    return decode_image(img_bytes)


def decode_image(img_bytes):
    """Decode image bytes (jpg/png) into a cv2 image."""
    img_arr = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
    return img


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print("Waiting for Unity connection...")
        conn, addr = s.accept()
        print("Connected by", addr)
        with conn:
            while True:
                try:
                    imgA = receive_image(conn)
                    imgB = receive_image(conn)

                    if imgA is not None:
                        cv2.imshow("Camera A", imgA)
                    if imgB is not None:
                        cv2.imshow("Camera B", imgB)

                    if cv2.waitKey(1) == 27:  # ESC to quit
                        break

                except Exception as e:
                    print("Error:", e)
                    break

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
