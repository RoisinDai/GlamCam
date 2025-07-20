from enum import IntEnum


class JointType(IntEnum):
    SpineBase = 0
    SpineMid = 1
    Neck = 2
    Head = 3
    ShoulderLeft = 4
    ElbowLeft = 5
    WristLeft = 6
    HandLeft = 7
    ShoulderRight = 8
    ElbowRight = 9
    WristRight = 10
    HandRight = 11
    HipLeft = 12
    KneeLeft = 13
    AnkleLeft = 14
    FootLeft = 15
    HipRight = 16
    KneeRight = 17
    AnkleRight = 18
    FootRight = 19
    SpineShoulder = 20
    HandTipLeft = 21
    ThumbLeft = 22
    HandTipRight = 23
    ThumbRight = 24


joint_color = {
    JointType.SpineBase: (0.482, 0.408, 0.933),  # #7b68ee - Medium Slate Blue
    JointType.SpineMid: (0.933, 0.510, 0.933),  # #ee82ee - Violet
    JointType.Neck: (1.000, 0.753, 0.796),  # #ffc0cb - Pink
    JointType.Head: (1.000, 0.078, 0.576),  # #ff1493 - Deep Pink
    JointType.ShoulderLeft: (0.000, 1.000, 1.000),  # #00ffff - Cyan
    JointType.ElbowLeft: (0.000, 0.000, 1.000),  # #0000ff - Blue
    JointType.WristLeft: (0.863, 0.078, 0.235),  # #dc143c - Crimson
    JointType.HandLeft: (0.541, 0.169, 0.886),  # #8a2be2 - Blue Violet
    JointType.ShoulderRight: (0.859, 0.439, 0.576),  # #db7093 - Pale Violet Red
    JointType.ElbowRight: (0.545, 0.000, 0.545),  # #8b008b - Dark Magenta
    JointType.WristRight: (1.000, 0.000, 1.000),  # #ff00ff - Magenta
    JointType.HandRight: (1.000, 0.271, 0.000),  # #ff4500 - Orange Red
    JointType.HipLeft: (0.941, 1.000, 1.000),  # #f0ffff - Azure
    JointType.KneeLeft: (0.000, 0.545, 0.545),  # #008b8b - Dark Cyan
    JointType.AnkleLeft: (0.282, 0.239, 0.545),  # #483d8b - Dark Slate Blue
    JointType.FootLeft: (1.000, 0.431, 0.780),  # #ff6ec7 - Hot Pink
    JointType.HipRight: (1.000, 0.361, 0.361),  # #FF5C5C - Coral Red
    JointType.KneeRight: (0.275, 0.541, 1.000),  # #468AFF - Sky Blue
    JointType.AnkleRight: (1.000, 0.541, 0.396),  # #FF8A65 - Light Orange
    JointType.FootRight: (0.910, 0.451, 1.000),  # #E873FF - Light Purple
    JointType.SpineShoulder: (0.400, 0.267, 1.000),  # #6644FF - Royal Purple
    JointType.HandTipLeft: (1.000, 0.200, 1.000),  # #FF33FF - Bright Magenta
    JointType.ThumbLeft: (0.600, 0.200, 1.000),  # #9933FF - Purple
    JointType.HandTipRight: (1.000, 0.302, 0.000),  # #ff4d00 - Orange Red
    JointType.ThumbRight: (0.310, 0.318, 0.694),  # #4f51b1 - Dark Blue
}
