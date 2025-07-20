import socket
import cv2
import numpy as np

from Receiver import (
    StreamReceiver,
    parse_kinect_packet,
    parse_unity_packet,
    decode_frame,
)
from Config import KINECT_HOST, KINECT_PORT, UNITY_HOST, UNITY_PORT
from PerFrame import process_frame


def main() -> None:
    # Socket setup
    HOST = "localhost"
    PORT = 5007
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(1)
    print("Waiting for consumer to connect...")
    conn, addr = server.accept()
    print(f"Consumer connected from {addr}.")

    kinect_receiver = StreamReceiver(
        KINECT_HOST, KINECT_PORT, parse_kinect_packet, name="KinectReceiver"
    )
    unity_receiver = StreamReceiver(
        UNITY_HOST, UNITY_PORT, parse_unity_packet, name="UnityReceiver"
    )

    kinect_latest = None
    unity_latest = None

    while True:
        kinect_latest = kinect_receiver.get_latest(kinect_latest)
        unity_latest = unity_receiver.get_latest(unity_latest)

        if kinect_latest is not None and unity_latest is not None:
            k_jpg_bytes, k_joints = kinect_latest
            imgA_bytes, imgB_bytes = unity_latest

            kinect_video = decode_frame(k_jpg_bytes)
            kinect_joints = k_joints[0] if k_joints and len(k_joints) > 0 else None
            unity_clothes = decode_frame(imgB_bytes)
            unity_joints = decode_frame(imgA_bytes)

            if (
                unity_joints is not None
                and unity_clothes is not None
                and kinect_video is not None
                and kinect_joints is not None
            ):
                # processed_frame = process_frame(
                #     unity_joints,
                #     unity_clothes,
                #     kinect_joints,
                #     kinect_video,
                # )

                # if processed_frame is not None:
                #     processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                #     # ---- SOCKET SEND LOGIC ----
                #     _, img_encoded = cv2.imencode(".jpg", processed_frame)
                #     data = img_encoded.tobytes()
                #     length = len(data)
                #     try:
                #         conn.sendall(length.to_bytes(4, "big") + data)
                #     except Exception as e:
                #         print("Connection closed.")
                #         break

                # ---- SOCKET SEND LOGIC ----
                kinect_video = cv2.cvtColor(kinect_video, cv2.COLOR_BGR2RGB)
                _, img_encoded = cv2.imencode(".jpg", kinect_video)
                data = img_encoded.tobytes()
                length = len(data)
                try:
                    conn.sendall(length.to_bytes(4, "big") + data)
                except Exception as e:
                    print("Connection closed.")
                    break

                print("Sent processed frame to consumer.")
                cv2.imshow("processed", kinect_video)

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
    conn.close()
    server.close()


if __name__ == "__main__":
    main()
