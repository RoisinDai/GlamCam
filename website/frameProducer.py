import socket
import cv2
import numpy as np

from Receiver import (
    StreamReceiver,
    parse_kinect_packet,
    parse_unity_packet,
    decode_frame,
)
from PerFrame import process_frame

from Config import (
    # TCP settings
    KINECT_HOST,
    KINECT_PORT,
    UNITY_HOST,
    UNITY_PORT,
    # UI to Unity settings
    UI_UNITY_HOST,
    UI_UNITY_PORT,
    # Live feed settings
    LIVE_FEED_HOST,
    LIVE_FEED_PORT,
    # Kinect hand data settings
    KINECT_HAND_HOST,
    KINECT_HAND_PORT,
    # WebSocket settings
    WS_HOST,
    WS_PORT,
)


def main() -> None:
    unity_receiver = StreamReceiver(
        UNITY_HOST, UNITY_PORT, parse_unity_packet, name="UnityReceiver"
    )

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((LIVE_FEED_HOST, LIVE_FEED_PORT))
    server.listen(1)
    print("Waiting for consumer to connect...")
    conn, addr = server.accept()
    print(f"Consumer connected from {addr}.")

    kinect_receiver = StreamReceiver(
        KINECT_HOST, KINECT_PORT, parse_kinect_packet, name="KinectReceiver"
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
            u_img_bytes, u_joints = unity_latest

            # Obtain the live video feed from Kinect
            kinect_video = decode_frame(k_jpg_bytes)

            # Obtain the joints coordinates from Kinect
            # TODO: Write now, only the first body is used. Need to handle multiple bodies in the future.
            kinect_joints = None
            if k_joints is not None and len(k_joints) > 0:
                kinect_joints = k_joints[0]

            # Obtain clothing frame from Unity
            unity_clothes = decode_frame(u_img_bytes)

            # Obtain the joints coordinates from Unity
            unity_joints = u_joints

            # Process only if all inputs are available
            if (
                unity_joints is not None
                and unity_clothes is not None
                and kinect_video is not None
                and kinect_joints is not None
            ):
                # Feed the four inputs to overlay and process them
                processed_frame = process_frame(
                    unity_joints,
                    unity_clothes,
                    kinect_joints,
                    kinect_video,
                )

                if processed_frame is not None:
                    # Display the processed frame
                    processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                    # ---- SOCKET SEND LOGIC ----
                    _, img_encoded = cv2.imencode(".jpg", processed_frame)
                    data = img_encoded.tobytes()
                    length = len(data)
                    try:
                        conn.sendall(length.to_bytes(4, "big") + data)
                    except Exception as e:
                        print("Connection closed.")
                        break

                    # Display the processed frame individually
                    cv2.imshow("Processed Frame", processed_frame)

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
    conn.close()
    server.close()


if __name__ == "__main__":
    main()
