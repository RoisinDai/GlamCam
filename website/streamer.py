from flask import Flask, Response, request
import numpy as np
import socket
import json

app = Flask(__name__)

HOST = 'localhost'
PORT = 5007 # Video producer port
UNITY_HOST = 'localhost'   # need to update...
UNITY_PORT = 5008    

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

@app.route("/select", methods=["POST"])
def select():
    data = request.json
    print("[UI Selection] ", data)

    # Send to Unity via TCP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((UNITY_HOST, UNITY_PORT))
        message = json.dumps(data).encode("utf-8")
        s.sendall(message)
        s.close()
        print("[Sent to Unity] ", message)
    except Exception as e:
        print("[Error sending to Unity]", e)
    return "", 204

@app.route("/")
def index():
    with open("index.html", "r") as f:
        return f.read()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
