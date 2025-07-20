from flask import Flask, Response
import cv2
import numpy as np
import socket

app = Flask(__name__)

HOST = 'localhost'
PORT = 5007 # same port as the producer

def socket_frame_generator():
    """Yield JPEG bytes from socket streaming producer."""
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((HOST, PORT))

    try:
        while True:
            # Receive 4 bytes for frame length
            length_data = b''
            while len(length_data) < 4:
                more = client.recv(4 - len(length_data))
                if not more:
                    return
                length_data += more
            frame_length = int.from_bytes(length_data, 'big')

            # Receive the frame data
            frame_data = b''
            while len(frame_data) < frame_length:
                more = client.recv(frame_length - len(frame_data))
                if not more:
                    return
                frame_data += more

            # MJPEG stream expects JPEG bytes, so just yield
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame_data + b"\r\n"
            )
    finally:
        client.close()

@app.route("/video_feed")
def video_feed():
    return Response(
        socket_frame_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/")
def index():
    with open("index.html", "r") as f:
        return f.read()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
