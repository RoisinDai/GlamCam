import cv2
import numpy as np
from matplotlib import pyplot as plt
from typing import Any, Dict, List
from JointMapping import joint_color, JointType


def segment_joints(frame: np.ndarray) -> np.ndarray:
    """
    Segment green dots (joints) from the input frame.
    Args:
        frame (np.ndarray): Input image frame in BGR format.
    Returns:
        np.ndarray: A binary mask where the green dots are white (255) and the rest is black (0).
    """

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Joint color
    joint_rgb = np.array([87, 239, 89])  # (R, G, B)

    # Set margin for thresholding (tune as needed)
    margin = 50
    lower = np.clip(joint_rgb - margin, 0, 255)
    upper = np.clip(joint_rgb + margin, 0, 255)

    # Create mask: pixels close to joint_rgb
    mask = np.all((frame_rgb >= lower) & (frame_rgb <= upper), axis=-1)

    return mask


def _get_joints_coord(mask: np.ndarray) -> list[tuple[int, int]]:
    """
    Extract coordinates of green dots from the mask.
    Args:
        mask (np.ndarray): Binary mask of green dots.
    Returns:
        list[tuple[int, int]]: List of (x, y) coordinates of the green dots.
    """

    # Find contours in the mask
    mask = (mask.astype(np.uint8)) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dot_centers = []
    min_area = 1  # Minimum area to filter out noise

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                dot_centers.append((cx, cy))

    # Sort by y, then x for consistency
    dot_centers_sorted = sorted(dot_centers, key=lambda p: (p[1], p[0]))

    return dot_centers_sorted


def _get_joints_coord(kinect_joints_coords: dict[str, Any]) -> list[tuple[int, int]]:
    """
    Get joint coordinates from Kinect data.
    Args:
        kinect_joints_coords (dict): Dictionary containing joint coordinates for one body.
    Returns:
        list[tuple[int, int]]: List of (x, y) coordinates of the joints.
    """

    if not isinstance(kinect_joints_coords, dict):
        raise ValueError("kinect_joints_coords must be a dictionary")

    kinect_coords = []
    for joint in list(kinect_joints_coords.values()):
        try:
            x = float(joint["X"])
            y = float(joint["Y"])
            if np.isfinite(x) and np.isfinite(y):
                kinect_coords.append((x, y))
        except (KeyError, ValueError, TypeError):
            continue

    return kinect_coords


def get_joints_coord(
    kinect_joints_coords: dict[str, Any],
) -> Dict[JointType, tuple[float, float]]:
    """
    Get joint coordinates from Kinect data, returning a dictionary mapping joint_type to (x, y).
    Args:
        kinect_joints_coords (dict): Dictionary containing joint coordinates for one body.
    Returns:
        dict[JointType, tuple[float, float]]: Dictionary of joint_type to (x, y) for each joint.
    """

    if not isinstance(kinect_joints_coords, dict):
        raise ValueError("kinect_joints_coords must be a dictionary")

    kinect_coords = {}
    for joint_name, joint in kinect_joints_coords.items():
        try:
            x = float(joint["X"])
            y = float(joint["Y"])
            if np.isfinite(x) and np.isfinite(y):
                # Convert joint_name string to JointType enum
                joint_type = JointType[joint_name]
                kinect_coords[joint_type] = (x, y)
        except (KeyError, ValueError, TypeError):
            continue

    return kinect_coords
