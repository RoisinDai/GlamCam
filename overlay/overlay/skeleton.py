import numpy as np
import matplotlib.pyplot as plt

from enum import Enum
from collections.abc import Mapping

class Skeleton:
    class JointType(Enum):
        """ Enum for joint types in a skeleton.
            Each joint corresponds to a specific body part."""
        HEAD = 0
        NECK = 1
        SHOULDER_LEFT = 2
        SHOULDER_RIGHT = 3
        ELBOW_LEFT = 4
        ELBOW_RIGHT = 5
        WRIST_LEFT = 6
        WRIST_RIGHT = 7
        HIP_LEFT = 8
        HIP_RIGHT = 9
        KNEE_LEFT = 10
        KNEE_RIGHT = 11
        ANKLE_LEFT = 12
        ANKLE_RIGHT = 13
        TORSO_UP = 14
        TORSO_MID = 15
        TORSO_DOWN = 16
        FEET_LEFT = 17
        FEET_RIGHT = 18

    # Define how the joints are connected
    # This is a simple stickman-like structure
    # Total of 19 joints and 18 connections (a TREE!)
    connections: list[tuple[JointType, JointType]] = [
      # Head and torso
      (JointType.HEAD,         JointType.NECK),
      (JointType.NECK,         JointType.TORSO_UP),
      (JointType.TORSO_UP,     JointType.TORSO_MID),
      (JointType.TORSO_MID,    JointType.TORSO_DOWN),

      # Shoulders and arms
      (JointType.TORSO_UP,     JointType.SHOULDER_LEFT),
      (JointType.TORSO_UP,     JointType.SHOULDER_RIGHT),
      (JointType.SHOULDER_LEFT,JointType.ELBOW_LEFT),
      (JointType.SHOULDER_RIGHT,JointType.ELBOW_RIGHT),
      (JointType.ELBOW_LEFT,   JointType.WRIST_LEFT),
      (JointType.ELBOW_RIGHT,  JointType.WRIST_RIGHT),

      # Hips and legs
      (JointType.TORSO_DOWN,   JointType.HIP_LEFT),
      (JointType.TORSO_DOWN,   JointType.HIP_RIGHT),
      (JointType.HIP_LEFT,     JointType.KNEE_LEFT),
      (JointType.HIP_RIGHT,    JointType.KNEE_RIGHT),
      (JointType.KNEE_LEFT,    JointType.ANKLE_LEFT),
      (JointType.KNEE_RIGHT,   JointType.ANKLE_RIGHT),
      (JointType.ANKLE_LEFT,   JointType.FEET_LEFT),
      (JointType.ANKLE_RIGHT,  JointType.FEET_RIGHT),
    ]

    def __init__(
            self,
            # The joints coordinates as a dictionary
            joints : Mapping[JointType, tuple[float, float]],
            # Do you want to normalize the coordinates?
            normalize : bool = True,
            # If you want to normalize, what is the reference point?
            reference_point : JointType = JointType.TORSO_MID):
        """
        joints: Nx2 numpy array or list of (x, y) coordinates
        connections: list of (idx1, idx2) tuples, indices into joints array
        """
        self.joints = dict()

        if normalize:
            assert reference_point in joints, "Reference point must be in joints if normalization is enabled."
            origin = np.array(joints[reference_point])
            self.joints = {jt: (np.array(coord) - origin) for jt, coord in joints.items()}
        else:
            self.joints = joints

    def draw(self, ax=None, joint_color='red', bone_color='blue', joint_size=40, bone_width=2):
        if ax is None:
            _, ax = plt.subplots()

        # Draw joints
        coords = np.array(list(self.joints.values()))
        ax.scatter(coords[:,0], coords[:,1], c=joint_color, s=joint_size, zorder=2)

        # Draw bones
        for idx1, idx2 in self.connections:
            # Get the joints coordinates
            j1 = self.joints[idx1]
            j2 = self.joints[idx2]
            x = [j1[0], j2[0]]
            y = [j1[1], j2[1]]
            ax.plot(x, y, c=bone_color, linewidth=bone_width, zorder=1)

        ax.set_aspect('equal')
        plt.show()


def create_sample_joints():
    """Create a dictionary of sample joint coordinates, using the JointType enum."""
    jt = Skeleton.JointType
    # Just placing joints in a rough stickman shape in 2D
    joints = {
        jt.HEAD: (0, 10),
        jt.NECK: (0, 8),
        jt.SHOULDER_LEFT: (-2, 8),
        jt.SHOULDER_RIGHT: (2, 8),
        jt.ELBOW_LEFT: (-3, 6),
        jt.ELBOW_RIGHT: (3, 6),
        jt.WRIST_LEFT: (-4, 4),
        jt.WRIST_RIGHT: (4, 4),
        jt.HIP_LEFT: (-1, 4),
        jt.HIP_RIGHT: (1, 4),
        jt.KNEE_LEFT: (-1, 2),
        jt.KNEE_RIGHT: (1, 2),
        jt.ANKLE_LEFT: (-1, 0),
        jt.ANKLE_RIGHT: (1, 0),
        jt.TORSO_UP: (0, 7),
        jt.TORSO_MID: (0, 6),
        jt.TORSO_DOWN: (0, 4),
        jt.FEET_LEFT: (-1, -1),
        jt.FEET_RIGHT: (1, -1),
    }
    return joints


# Example usage:
def test_skeleton():
    """Test function to create and draw a sample skeleton."""
    ske = Skeleton(create_sample_joints())
    ske.draw()
