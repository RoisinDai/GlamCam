# THE MAIN ENTRY POINT FOR THE PIPELINE

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
    # Start receiver threads
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

            # Obtain the live video feed from Kinect
            kinect_video = decode_frame(k_jpg_bytes)
            # Obtain the joints coordinates from Kinect
            # TODO: Write now, only the first body is used. Need to handle multiple bodies in the future.
            kinect_joints = None
            if k_joints is not None and len(k_joints) > 0:
                kinect_joints = k_joints[0]

            # Obtain clothing frame from Unity
            unity_clothes = decode_frame(imgB_bytes)
            # Obtain joints coordinates frame from Unity
            unity_joints = decode_frame(imgA_bytes)

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
                    cv2.imshow("processed", processed_frame)

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
