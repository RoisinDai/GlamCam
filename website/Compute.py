import numpy as np
from scipy.spatial import cKDTree  # type: ignore
from JointMapping import JointType


def compute_rms_radius(points):
    """
    Compute the RMS radius of a set of points.
    Args:
        points (np.ndarray): Array of shape (N, 2) representing N points in 2D space.
    Returns:
        float: The RMS radius of the points.
    """

    centroid = np.mean(points, axis=0)
    return np.sqrt(np.mean(np.sum((points - centroid) ** 2, axis=1)))


def rigid_icp_2d_accum(src, dst, max_iter=50, tolerance=1e-6, verbose=False):
    """
    Perform a rigid ICP (Iterative Closest Point) alignment of two sets of 2D points.
    Args:
        src (np.ndarray): Source points of shape (N, 2).
        dst (np.ndarray): Destination points of shape (M, 2).
        max_iter (int): Maximum number of iterations.
        tolerance (float): Convergence tolerance.
        verbose (bool): If True, print progress.
    Returns:
        tuple:
            - R (np.ndarray): Rotation matrix of shape (2, 2).
            - t (np.ndarray): Translation vector of shape (2,).
            - src_trans (np.ndarray): Transformed source points after alignment.
    """

    src_trans = src.copy()
    errors = []
    prev_error = None

    # Initialize accumulated rotation and translation
    R_accum = np.eye(2)
    t_accum = np.zeros(2)

    for i in range(max_iter):
        tree = cKDTree(dst)
        _, indices = tree.query(src_trans)
        dst_matched = dst[indices]

        centroid_src = np.mean(src_trans, axis=0)
        centroid_dst = np.mean(dst_matched, axis=0)
        src_centered = src_trans - centroid_src
        dst_centered = dst_matched - centroid_dst

        H = src_centered.T @ dst_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_dst - R @ centroid_src

        # Apply transform
        src_trans = (R @ src_trans.T).T + t

        # Accumulate transformation
        t_accum = R @ t_accum + t
        R_accum = R @ R_accum

        mean_error = np.mean(np.linalg.norm(src_trans - dst_matched, axis=1))
        errors.append(mean_error)
        if verbose:
            print(f"Iter {i+1}: error={mean_error:.4f}")
        if prev_error is not None and abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return R_accum, t_accum, src_trans  # Accumulated R, t


def icp_with_prescaling(src, dst, max_iter=50, tolerance=1e-6, verbose=False):
    """
    Perform ICP with an initial scaling step.
    Args:
        src (np.ndarray): Source points of shape (N, 2).
        dst (np.ndarray): Destination points of shape (M, 2).
        max_iter (int): Maximum number of iterations.
        tolerance (float): Convergence tolerance.
        verbose (bool): If True, print progress.
    Returns:
        tuple:
            - scale (float): Scaling factor applied to the source points.
            - R (np.ndarray): Rotation matrix of shape (2, 2).
            - t (np.ndarray): Translation vector of shape (2,).
            - src_aligned (np.ndarray): Transformed source points after alignment.
    """

    # Step 1: Estimate global scaling
    rms_src = compute_rms_radius(src)
    rms_dst = compute_rms_radius(dst)
    scale = rms_dst / rms_src if rms_src > 1e-8 else 1.0

    # Step 2: Scale src
    src_scaled = src * scale

    # Step 3: Rigid ICP (rotation + translation, no scaling)
    R, t, src_aligned = rigid_icp_2d_accum(
        src_scaled, dst, max_iter=max_iter, tolerance=tolerance, verbose=verbose
    )

    return scale, R, t, src_aligned


def run_icp(
    unity_coords: dict[JointType, tuple[float, float]],
    kinect_coords: dict[JointType, tuple[float, float]],
    common_joints: list[JointType],
) -> np.ndarray:
    """
    Run ICP to align Unity coordinates with Kinect coordinates.
    Args:
      unity_coords (dict[JointType, tuple[float, float]]): Dictionary of (x, y) from Unity.
      kinect_coords (dict[JointType, tuple[float, float]]): Dictionary of (x, y) from Kinect.
    Returns:
      np.ndarray: The 2x3 affine transformation matrix (scale * R | t).
    """
    # Establish one-to-one correspondence, sorted by JointType value
    unity_points = np.array([unity_coords[jt] for jt in common_joints])
    kinect_points = np.array([kinect_coords[jt] for jt in common_joints])
    assert len(unity_points) == len(kinect_points), "Mismatched joint counts"

    # Perform ICP to align Kinect coordinates to Unity coordinates
    scale, R, t, _ = icp_with_prescaling(unity_points, kinect_points)

    # Create the affine transformation matrix
    M = scale * R
    affine_matrix = np.hstack([M, t.reshape(2, 1)])

    return affine_matrix


def alpha_blend(bg: np.ndarray, fg: np.ndarray) -> np.ndarray:
    """
    Perform alpha blending of two images.
    Args:
        bg (np.ndarray): Background image in RGBA format.
        fg (np.ndarray): Foreground image in RGBA format.
    Returns:
        np.ndarray: Blended image in RGBA format.
    """

    bg = bg.astype(float) / 255.0
    fg = fg.astype(float) / 255.0

    out_alpha = fg[..., 3] + bg[..., 3] * (1 - fg[..., 3])
    mask = out_alpha > 0
    out_rgb = np.zeros_like(fg[..., :3])
    out_rgb[mask] = (
        fg[..., :3] * fg[..., 3:4] + bg[..., :3] * bg[..., 3:4] * (1 - fg[..., 3:4])
    )[mask] / out_alpha[mask, None]
    out = np.zeros_like(fg)
    out[..., :3] = out_rgb
    out[..., 3] = out_alpha

    return (out * 255).astype(np.uint8)
