from flask import Flask, Response, request
import numpy as np
import socket
import json
import threading
import asyncio
import websockets
import struct
from Config import frame_height, frame_width
from Config import (
    UI_UNITY_HOST as UNITY_HOST,
    UI_UNITY_PORT as UNITY_PORT,
    LIVE_FEED_HOST as HOST,
    LIVE_FEED_PORT as PORT,
    KINECT_HAND_HOST as KINECT_HOST,
    KINECT_HAND_PORT as KINECT_PORT,
    WS_HOST as WS_HOST,
    WS_PORT as WS_PORT,
)


app = Flask(__name__)


latest_right_hand = {"x": -1, "y": -1}  # start centered
hand_joints = ["HandRight", "HandLeft", "HandTipRight", "HandTipLeft"]

# ---- FLASK HTTP ENDPOINTS ----


def socket_frame_generator():
    """Yield JPEG bytes from socket streaming producer."""
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((HOST, PORT))
    try:
        while True:
            # Receive 4 bytes for frame length
            length_data = b""
            while len(length_data) < 4:
                more = client.recv(4 - len(length_data))
                if not more:
                    return
                length_data += more
            frame_length = int.from_bytes(length_data, "big")
            # Receive the frame data
            frame_data = b""
            while len(frame_data) < frame_length:
                more = client.recv(frame_length - len(frame_data))
                if not more:
                    return
                frame_data += more
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame_data + b"\r\n"
            )
    finally:
        client.close()


@app.route("/video_feed")
def video_feed():
    return Response(
        socket_frame_generator(), mimetype="multipart/x-mixed-replace; boundary=frame"
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


# ---------- KINECT TCP HAND DATA LISTENER -----------
def kinect_hand_tcp_listener():
    global latest_right_hand
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((KINECT_HOST, KINECT_PORT))
    server.listen(1)
    print(
        f"[KINECT TCP] Listening for Kinect hand data on {KINECT_HOST}:{KINECT_PORT} ..."
    )
    while True:
        conn, addr = server.accept()
        print(f"[KINECT TCP] Connected: {addr}")
        with conn:
            while True:
                try:
                    # Receive 4 bytes: totalLen
                    total_len_bytes = conn.recv(4)
                    if not total_len_bytes or len(total_len_bytes) < 4:
                        print("No length header received.")
                        break  # Changed from continue to break, as connection is likely closed
                    total_len = struct.unpack("<I", total_len_bytes)[0]

                    # Now receive 4 bytes: dataLen
                    data_len_bytes = conn.recv(4)
                    if not data_len_bytes or len(data_len_bytes) < 4:
                        print("No dataLen header received.")
                        break
                    data_len = struct.unpack("<I", data_len_bytes)[0]

                    # Now receive the JSON data
                    data = b""
                    while len(data) < data_len:
                        chunk = conn.recv(data_len - len(data))
                        if not chunk:
                            break
                        data += chunk

                    if len(data) < data_len:
                        print("Incomplete data received.")
                        break

                    # Parse hands JSON array
                    hands_list = json.loads(data.decode())
                    # print("[KINECT TCP] Received hands data:", hands_list)
                    if len(hands_list) == 0:
                        latest_right_hand = {
                            "x": -1,
                            "y": -1,
                        }  # Use -1 to indicate no hands detected
                        # print("[KINECT TCP] No hands detected.")
                        continue

                    hands_dict = hands_list[0]
                    try:
                        latest_right_hand = {
                            "x": hands_dict["HandRight"]["X"] / frame_width,
                            "y": hands_dict["HandRight"]["Y"] / frame_height,
                        }
                    except KeyError as e:
                        latest_right_hand = {
                            "x": -1,
                            "y": -1,
                        }  # Use -1 to indicate no right hand detected
                        print("[KINECT TCP] Error:", e)

                except Exception as e:
                    print("[KINECT TCP] Error:", e)
                    break  # On error, break loop and wait for new connection


# -------------- WEBSOCKET SERVER -------------------
async def ws_handler(websocket):
    print("[WS] Client connected")
    try:
        while True:
            msg = json.dumps(latest_right_hand)
            await websocket.send(msg)
            await asyncio.sleep(0.033)
    except Exception:
        print("[WS] Client disconnected")


async def start_ws_server():
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        print(f"[WS] WebSocket server running on {WS_HOST}:{WS_PORT}")
        await asyncio.Future()  # Run forever


def start_websocket_server():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(start_ws_server())


if __name__ == "__main__":
    # Start Kinect hand TCP listener in a thread
    t1 = threading.Thread(target=kinect_hand_tcp_listener, daemon=True)
    t1.start()

    # Start WebSocket server in a thread
    t2 = threading.Thread(target=start_websocket_server, daemon=True)
    t2.start()

    # Start Flask app in main thread
    print(f"[FLASK] Starting HTTP server on port 5001")
    app.run(host="0.0.0.0", port=5001)
