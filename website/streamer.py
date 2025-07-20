"""streamer.py is responsible for taking the numpy livestream video feed as input and embedding it onto the website"""

from flask import Flask, Response
import cv2

app = Flask(__name__)


def generate():
    # use our default camera for now, we will replace it with the live video feed
    cap = cv2.VideoCapture(0) 
    # infinite loop to continuously read and stream frames
    while True:
        success, frame = cap.read()
        if not success:
            break
        # encodes numpy frame into jpg
        _, buffer = cv2.imencode(".jpg", frame)
        # convert jpg buffer to real bytes to send over HTTP
        frame_bytes = buffer.tobytes()
        yield (
            b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


@app.route("/video_feed")
def video_feed():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    with open("index.html", "r") as f:
        return f.read()


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)

