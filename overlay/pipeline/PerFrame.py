from __future__ import annotations

from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import cv2
import Compute
import KinectUtils
import UnityUtils
from UnityUtils import segment_joints, segment_clothes


def process_frame(
    unity_joints_frame, unity_clothes_frame, kinect_joints_coords, live_human_frame
) -> np.ndarray:
    """
    Process four input frames and output a single frame.
    Args:
        unity_joints_frame: Unity joint coordinates as red dots on a green background (numpy.ndarray, BGR or RGB).
        unity_clothes_frame: Unity clothing image with green background (numpy.ndarray, BGR or RGB).
        kinect_joints_coords: Coordinates of joints from Kinect (dictionary or similar structure).
        live_human_frame: Live video feed of a real human (numpy.ndarray, BGR or RGB).
    Returns:
        Output frame (numpy.ndarray): The composited output frame.
    """

    # Obtain the coordinates of the joints from Unity
    unity_coords = UnityUtils.get_joints_coord(segment_joints(unity_joints_frame))
    if len(unity_coords) == 0:
        return None

    # Obtain the coordinates of the joints from Kinect
    kinect_coords = KinectUtils.get_joints_coord(kinect_joints_coords)
    if len(kinect_coords) == 0:
        return None

    # Segment the clothing from the Unity frame
    cloth_transparent = UnityUtils.apply_mask_to_image(
        unity_clothes_frame, segment_clothes(unity_clothes_frame)
    )

    # Run the ICP algorithm to align Kinect coordinates with Unity coordinates
    affine_matrix = Compute.run_icp(unity_coords, kinect_coords)

    # Enture both images are RGBA for blending
    if live_human_frame.shape[2] == 3:
        live_human_rgba = np.concatenate(
            [
                cv2.cvtColor(live_human_frame, cv2.COLOR_BGR2RGB),
                np.full(live_human_frame.shape[:2] + (1,), 255, dtype=np.uint8),
            ],
            axis=2,
        )
    elif live_human_frame.shape[2] == 4:
        live_human_rgba = cv2.cvtColor(live_human_frame, cv2.COLOR_BGRA2RGBA)
    else:
        raise ValueError("live_human_frame must have 3 or 4 channels")

    # Apply the affine transformation to the clothing image
    h, w = live_human_rgba.shape[:2]
    fore_aligned = cv2.warpAffine(
        cloth_transparent,
        affine_matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0, 0),
    )

    # Compose the final output by blending the live human frame with the aligned clothing
    composite = Compute.alpha_blend(live_human_rgba, fore_aligned)

    return composite


# For testing purposes, you can run this script directly
if __name__ == "__main__":
    # The paths to the input video files
    unity_joints = "./input/unity-joints.mp4"
    unity_clothes = "./input/unity-vid.mp4"
    kinect_joints = "./input/kinect-joints.mp4"
    live_human = "./input/live-vid.mp4"

    # Load only the first frame of each video
    unity_joints_frame = cv2.VideoCapture(unity_joints).read()[1]
    unity_clothes_frame = cv2.VideoCapture(unity_clothes).read()[1]
    kinect_joints_frame = cv2.VideoCapture(kinect_joints).read()[1]
    live_human_frame = cv2.VideoCapture(live_human).read()[1]

    # Process the frames
    output_frame = process_frame(
        unity_joints_frame, unity_clothes_frame, kinect_joints_frame, live_human_frame
    )

    # Display the output frame using matplotlib
    plt.figure(figsize=(8, 8))
    plt.imshow(output_frame)
    plt.axis("off")
    plt.title("Processed Output Frame")
    plt.show()
