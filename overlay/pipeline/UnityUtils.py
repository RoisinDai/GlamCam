from __future__ import annotations

import cv2
import numpy as np
from JointMapping import joint_color, JointType


def _segment_joints(frame: np.ndarray) -> np.ndarray:
    """
    Segment red dots from a frame and return a mask of the segmented red dots.
    Args:
        frame (np.ndarray): Input image frame in BGR format.
    Returns:
        the mask of segmented red dots (np.ndarray):
        A binary mask where red dots are white (255) and the rest is black (0).
    """

    # Convert BGR (OpenCV) to RGB (matplotlib)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # If your image is RGB (from matplotlib or elsewhere), convert to BGR for OpenCV
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    # Convert BGR to HSV
    img_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Tune HSV ranges for bright red
    # Red hue is around 0 and 180 (OpenCV: 0-179). Bright red: high S and V.
    # [H, S, V] (lower hue, higher sat/val)
    lower_red1 = np.array([0, 120, 100])
    upper_red1 = np.array([10, 255, 255])
    # [H, S, V] (upper hue, higher sat/val)
    lower_red2 = np.array([170, 120, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for the two red ranges and combine
    mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Optional: Clean up small noise with morphological operations
    kernel = np.ones((5, 5), np.uint8)  # Slightly larger kernel for dots
    red_mask_clean = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask_clean = cv2.morphologyEx(red_mask_clean, cv2.MORPH_CLOSE, kernel)

    return red_mask_clean


def get_dots_mapping(
    frame: np.ndarray, joints: list[tuple[int, int]]
) -> list[JointType | None]:
    """
    Map the segmented dots to the closest joint type based on color.

    Args:
        frame (np.ndarray): Input image frame in BGR format.
        joints (list[tuple[int, int]]): List of (x, y) coordinates of the segmented joints.

    Returns:
        list[JointType | None]: The joint type corresponding to each dot color, or None if no match found.
    """

    mapping: list[JointType | None] = []
    for x, y in joints:
        # Make sure (x, y) are in bounds, which should be true always.
        if 0 <= y < frame.shape[0] and 0 <= x < frame.shape[1]:
            b, g, r = frame[y, x]  # OpenCV uses BGR format
            rgb = (r, g, b)
            joint_type = find_closest_joint(rgb)
            mapping.append(joint_type)
        else:
            mapping.append(None)

    # Ensure the mapping is consistent with the number of joints
    assert len(mapping) == len(joints), "Mapping length does not match number of joints"
    return mapping


def find_closest_joint(rgb: tuple[float, float, float]) -> JointType | None:
    """
    Args:
        rgb: A tuple or list of (R, G, B) in 0-255 or 0-1 range.

    Returns:
        closest_joint: The joint whose color is closest to rgb.
    """
    # If rgb is in 0-255 range, normalize to 0-1
    rgb = np.array(rgb, dtype=float)
    if np.max(rgb) > 1.0:
        rgb = rgb / 255.0

    min_dist = float("inf")
    closest_joint = None
    for joint, color in joint_color.items():
        dist = np.linalg.norm(rgb - np.array(color))
        if dist < min_dist:
            min_dist = dist
            closest_joint = joint
    return closest_joint


def segment_joints(frame: np.ndarray) -> list[tuple[float, float]]:
    """
    Segment non-green dots from a frame with a pure green background.
    Args:
        frame (np.ndarray): Input image frame in BGR format.
    Returns:
        list[tuple[int, int]]: List of (x, y) coordinates of the segmented joints.
    """
    # Convert BGR to HSV for easier color masking
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Pure green in HSV (OpenCV): H about 60, S and V high
    # Allow some tolerance for green background
    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])

    # Mask for green background
    green_mask = cv2.inRange(img_hsv, lower_green, upper_green)

    # Invert mask to get non-green areas (i.e., the dots)
    dots_mask = cv2.bitwise_not(green_mask)

    # Clean up small noise with morphological operations
    kernel = np.ones((5, 5), np.uint8)
    dots_mask_clean = cv2.morphologyEx(dots_mask, cv2.MORPH_OPEN, kernel)
    dots_mask_clean = cv2.morphologyEx(dots_mask_clean, cv2.MORPH_CLOSE, kernel)

    return get_joints_coord(dots_mask_clean)


def get_joints_coord(red_mask_clean: np.ndarray) -> list[tuple[int, int]]:
    """
    Extract coordinates of red dots from the cleaned mask.
    Args:
        red_mask_clean (np.ndarray): Cleaned binary mask of red dots.
    Returns:
        list[tuple[int, int]]: List of (x, y) coordinates of the red dots.
    """

    # Find contours in the mask
    contours, _ = cv2.findContours(
        red_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    dot_centers = []
    min_area = 10  # Minimum area to filter out noise

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


def segment_clothes(frame: np.ndarray) -> np.ndarray:
    """
    Segment the green background from a frame and return a mask of the segmented area.
    Args:
        frame (np.ndarray): Input image frame in BGR format.
    Returns:
        np.ndarray: A binary mask where the green background is white (255) and the rest is black (0).
    """

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2HSV)

    # Mask for green background
    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])
    bg_mask = cv2.inRange(hsv, lower_green, upper_green)

    # Invert background mask: 255 where not background
    fg_mask = cv2.bitwise_not(bg_mask)

    return fg_mask


def apply_mask_to_image(frame: np.ndarray, fg_mask: np.ndarray) -> np.ndarray:
    """
    Apply a foreground mask to an image to obtain a transparent image.
    Args:
        frame2 (np.ndarray): The original image in BGR format.
        fg_mask (np.ndarray): The foreground mask where the foreground is white (255) and the background is black (0).
    Returns:
        np.ndarray: The masked image with transparency applied.
    """

    # Read the original image using OpenCV
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Mask the original image with the foreground mask
    alpha = fg_mask
    img_rgb_masked = cv2.bitwise_and(img_rgb, img_rgb, mask=alpha)
    img_rgba = np.dstack([img_rgb_masked, alpha])

    return img_rgba
