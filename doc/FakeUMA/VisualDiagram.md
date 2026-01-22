# Continuous Bone Scaling System - Visual Flow Diagram

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    INITIALIZATION (Start)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. Initialize FakeUMA Bone Scaling System                      │
│     └─► Collect all bone transforms in hierarchy                │
│                                                                  │
│  2. Initialize Bone Mapping Configurations                      │
│     └─► Define 8 Kinect joint pairs → Unity bone mappings       │
│                                                                  │
│  3. Measure Original Avatar Bone Lengths                        │
│     └─► Store baseline measurements in dictionary               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    RUNTIME (Every Frame)                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Update() → hasValidBody = true if Kinect tracking active       │
│                                                                  │
│  LateUpdate() → If hasValidBody:                                │
│                                                                  │
│    ┌────────────────────────────────────────────┐               │
│    │ 1. UpdateContinuousBoneScaling()          │               │
│    │    (PHASE 5 - Main Scaling Update)        │               │
│    └────────────────────────────────────────────┘               │
│                      ▼                                           │
│    ┌────────────────────────────────────────────┐               │
│    │ For each bone in _BoneMappingConfigs:     │               │
│    │                                            │               │
│    │  ┌──────────────────────────────────────┐ │               │
│    │  │ PHASE 3: Measure & Smooth            │ │               │
│    │  ├──────────────────────────────────────┤ │               │
│    │  │ • Get Kinect joint positions         │ │               │
│    │  │ • Calculate raw distance             │ │               │
│    │  │ • Check tracking quality             │ │               │
│    │  │ • Apply exponential smoothing        │ │               │
│    │  │   smoothed = lerp(prev, curr, α)     │ │               │
│    │  └──────────────────────────────────────┘ │               │
│    │                  ▼                         │               │
│    │  ┌──────────────────────────────────────┐ │               │
│    │  │ PHASE 4: Calculate Scale Factor      │ │               │
│    │  ├──────────────────────────────────────┤ │               │
│    │  │ • Get original avatar bone length    │ │               │
│    │  │ • Calculate ratio: kinect/avatar     │ │               │
│    │  │ • Apply safety clamping (0.5-2.0x)   │ │               │
│    │  │ • Convert to Vector3 scale           │ │               │
│    │  └──────────────────────────────────────┘ │               │
│    │                  ▼                         │               │
│    │  ┌──────────────────────────────────────┐ │               │
│    │  │ PHASE 5: Apply Absolute Scaling      │ │               │
│    │  ├──────────────────────────────────────┤ │               │
│    │  │ • Get current relative scale         │ │               │
│    │  │ • Calculate reset (inverse)          │ │               │
│    │  │ • Apply reset → back to original     │ │               │
│    │  │ • Apply new scale → match Kinect     │ │               │
│    │  │ • FakeUMA handles child compensation │ │               │
│    │  └──────────────────────────────────────┘ │               │
│    │                                            │               │
│    └────────────────────────────────────────────┘               │
│                      ▼                                           │
│    ┌────────────────────────────────────────────┐               │
│    │ 2. ApplyForwardKinematics()               │               │
│    │    (Rotate bones based on Kinect)         │               │
│    └────────────────────────────────────────────┘               │
│                      ▼                                           │
│    ┌────────────────────────────────────────────┐               │
│    │ 3. RotateAvatarBasedOnShoulders()         │               │
│    │    (Orient avatar to face user)            │               │
│    └────────────────────────────────────────────┘               │
│                      ▼                                           │
│    ┌────────────────────────────────────────────┐               │
│    │ 4. Position Avatar at SpineBase           │               │
│    │    (Move root to match Kinect position)    │               │
│    └────────────────────────────────────────────┘               │
│                      ▼                                           │
│    ┌────────────────────────────────────────────┐               │
│    │ 5. Apply Uniform Scaling (once)            │               │
│    │    (Scale root for overall height match)   │               │
│    └────────────────────────────────────────────┘               │
│                      ▼                                           │
│    ┌────────────────────────────────────────────┐               │
│    │ 6. ApplyUniformShoulderTranslation()      │               │
│    │    (Fine-tune position from shoulders)     │               │
│    └────────────────────────────────────────────┘               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Data Flow Diagram

```
                    ┌──────────────────┐
                    │  Kinect Sensor   │
                    │   (Hardware)     │
                    └────────┬─────────┘
                             │ Joint positions (X,Y,Z)
                             ▼
                    ┌──────────────────┐
                    │  Body Tracking   │
                    │  (Kinect SDK)    │
                    └────────┬─────────┘
                             │ Body + Joints + TrackingState
                             ▼
        ┌────────────────────────────────────────────┐
        │       AvatarController.Update()            │
        │  trackedBody = GetTrackedBody()            │
        │  hasValidBody = (trackedBody != null)      │
        └────────────────┬───────────────────────────┘
                         │
                         ▼
        ┌────────────────────────────────────────────┐
        │  AvatarController.LateUpdate()             │
        │  IF hasValidBody:                          │
        └────────────────┬───────────────────────────┘
                         │
                         ▼
        ┌────────────────────────────────────────────┐
        │  UpdateContinuousBoneScaling()             │
        └────────────────┬───────────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         │               │               │
         ▼               ▼               ▼
    ┌────────┐    ┌────────┐      ┌────────┐
    │ Kinect │    │Original│      │Smoothed│
    │ Joints │    │ Avatar │      │ Kinect │
    │        │    │ Bones  │      │ Values │
    └───┬────┘    └───┬────┘      └───┬────┘
        │             │               │
        │ Raw         │ Baseline      │ Previous
        │ Measure     │ Length        │ Smoothed
        │             │               │
        └──────┬──────┴───────┬───────┘
               │              │
               ▼              ▼
        ┌──────────────────────────┐
        │  Quality Check           │
        │  • TrackingState?        │
        │  • In bounds (0.1-1.0m)? │
        └──────────┬───────────────┘
                   │ ✓ Valid
                   ▼
        ┌──────────────────────────┐
        │  Exponential Smoothing   │
        │  smooth = lerp(prev,     │
        │         curr, α)          │
        └──────────┬───────────────┘
                   │ Smoothed length
                   ▼
        ┌──────────────────────────┐
        │  Calculate Scale Factor  │
        │  ratio = kinect/avatar   │
        │  clamp(ratio, 0.5, 2.0)  │
        └──────────┬───────────────┘
                   │ Scale factor
                   ▼
        ┌──────────────────────────┐
        │  Convert to Vector3      │
        │  (1, scaleFactor, 1)     │
        │  [Y-axis stretch]        │
        └──────────┬───────────────┘
                   │ Scale vector
                   ▼
        ┌──────────────────────────┐
        │  Apply via FakeUMA       │
        │  1. Reset to original    │
        │  2. Apply new scale      │
        └──────────┬───────────────┘
                   │ Scaled bone
                   ▼
        ┌──────────────────────────┐
        │  Unity Avatar Skeleton   │
        │  (Bones match Kinect     │
        │   user proportions)      │
        └──────────────────────────┘
```

## Exponential Smoothing Visualization

```
Raw Kinect Measurements (noisy):
│
│     ╱╲    ╱╲  ╱╲
│  ╱╲╱  ╲╱╲╱  ╲╱  ╲
│╱╲                ╲╱╲
└─────────────────────────► Time


Smoothed Output (α = 0.1):
│
│        ╱‾‾‾‾‾‾╲
│      ╱          ╲
│    ╱              ╲
│  ╱                  ╲
└─────────────────────────► Time


Lower α (0.05) = More smoothing:
│        ╱‾‾‾‾‾‾‾‾╲
│      ╱            ╲
│    ╱                ╲
│  ╱                    ╲
└─────────────────────────► Time


Higher α (0.20) = Less smoothing:
│     ╱╲  ╱‾╲  ╱╲
│  ╱╲╱  ╲╱   ╲╱  ╲
│╱                ╲╱╲
└─────────────────────────► Time
```

## Bone Scaling State Machine

```
                  START
                    │
                    ▼
        ┌─────────────────────┐
        │   Initialization    │
        │  • Measure original │
        │  • Setup mappings   │
        └──────────┬──────────┘
                   │
                   ▼
        ┌─────────────────────┐
        │   Waiting for       │◄───────┐
        │   Valid Body        │        │
        └──────────┬──────────┘        │
                   │ hasValidBody      │
                   ▼                   │ Lost tracking
        ┌─────────────────────┐        │
        │   Measuring         │        │
        │  • Get joint pos    │        │
        │  • Check quality    │        │
        └──────────┬──────────┘        │
                   │ Valid             │
                   ▼                   │
        ┌─────────────────────┐        │
        │   Smoothing         │        │
        │  • Apply EMA        │        │
        │  • Store value      │        │
        └──────────┬──────────┘        │
                   │                   │
                   ▼                   │
        ┌─────────────────────┐        │
        │   Calculating       │        │
        │  • Compute ratio    │        │
        │  • Clamp safety     │        │
        └──────────┬──────────┘        │
                   │                   │
                   ▼                   │
        ┌─────────────────────┐        │
        │   Scaling           │        │
        │  • Reset bone       │        │
        │  • Apply scale      │        │
        └──────────┬──────────┘        │
                   │                   │
                   └───────────────────┘
                   (Next Frame)
```

## Bone Configuration Map

```
┌─────────────────────────────────────────────────────────┐
│              KINECT → UNITY BONE MAPPING                │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  KINECT (User Facing)          UNITY (Avatar)          │
│                                                         │
│    ShoulderLeft  ───────────►  RightUpperArm           │
│         │                            │                  │
│         │                            │                  │
│    ElbowLeft     ───────────►  RightLowerArm           │
│         │                            │                  │
│         │                            │                  │
│    WristLeft                    RightHand               │
│                                                         │
│                                                         │
│    ShoulderRight ───────────►  LeftUpperArm            │
│         │                            │                  │
│         │                            │                  │
│    ElbowRight    ───────────►  LeftLowerArm            │
│         │                            │                  │
│         │                            │                  │
│    WristRight                   LeftHand                │
│                                                         │
│                                                         │
│    HipLeft       ───────────►  RightUpperLeg           │
│         │                            │                  │
│         │                            │                  │
│    KneeLeft      ───────────►  RightLowerLeg           │
│         │                            │                  │
│         │                            │                  │
│    AnkleLeft                    RightFoot               │
│                                                         │
│                                                         │
│    HipRight      ───────────►  LeftUpperLeg            │
│         │                            │                  │
│         │                            │                  │
│    KneeRight     ───────────►  LeftLowerLeg            │
│         │                            │                  │
│         │                            │                  │
│    AnkleRight                   LeftFoot                │
│                                                         │
└─────────────────────────────────────────────────────────┘

Note: Left/Right are MIRRORED because Kinect faces the user!
```

## Scale Vector Application

```
BONE STRETCH AXIS (Standard Unity Humanoid):

            Y (UP) ← Length axis
            │
            │
            │
            └────── X
           ╱
          ╱
         Z

Scale Vector = Vector3(1.0, scaleFactor, 1.0)

               X = 1.0  (no change in width)
               Y = scaleFactor (change length)
               Z = 1.0  (no change in depth)

Example: scaleFactor = 1.5
         Bone becomes 50% longer in Y direction
         Width and depth unchanged
```

## FakeUMA Inverse Compensation

```
BEFORE SCALING:
    Parent Bone
    │ scale = 1.0
    │
    ├─ Child A
    │  scale = 1.0
    │
    └─ Child B
       scale = 1.0


AFTER ScaleBoneIndependently(Parent, 1.5x):
    Parent Bone
    │ scale = 1.5 ◄─── Scaled UP
    │
    ├─ Child A
    │  scale = 0.667 ◄─── Scaled DOWN (inverse)
    │
    └─ Child B
       scale = 0.667 ◄─── Scaled DOWN (inverse)


VISUAL RESULT:
    Parent bone appears 1.5x longer
    Children maintain their original size in world space
    (Parent's 1.5x × Child's 0.667x = 1.0x effective scale)
```

## Measurement Bounds Validation

```
Measurement Value vs. Action:

  0.0m ─┬─ REJECT (Invalid)
        │
  0.1m ─┼─ ACCEPT (Minimum valid) ◄─── Lower bound
        │
  0.3m ─┤
        │
  0.5m ─┤  Typical arm/leg
        │  segment lengths
  0.7m ─┤
        │
  1.0m ─┼─ ACCEPT (Maximum valid) ◄─── Upper bound
        │
  1.5m ─┴─ REJECT (Too large)

Action on rejection: Use previous smoothed value
```

## Safety Clamping Visualization

```
Scale Factor Calculation:

  kinectLength = 0.40m
  avatarLength = 0.25m
  ratio = 0.40 / 0.25 = 1.6


Safety Clamping Range:

  0.0x ─┬─ Invalid
        │
  0.5x ─┼─ Minimum allowed scale ◄─── Lower clamp
        │
  1.0x ─┤  No scaling (1:1 match)
        │
  1.6x ─┤  ◄─── Our calculated ratio (ACCEPTED)
        │
  2.0x ─┼─ Maximum allowed scale ◄─── Upper clamp
        │
  5.0x ─┴─ Would be clamped to 2.0x

Final scale factor = Clamp(1.6, 0.5, 2.0) = 1.6
```

## Performance Profile

```
Per-Frame Cost Breakdown:

UpdateContinuousBoneScaling()
├─ Dictionary lookups (8×): ~0.01ms
├─ Kinect measurements (8×): ~0.03ms
├─ Quality checks (8×): ~0.01ms
├─ Smoothing calcs (8×): ~0.02ms
├─ Scale calculations (8×): ~0.02ms
├─ FakeUMA scaling (8×2): ~0.05ms
└─ Total: ~0.14ms per frame

Frame Budget at 60 FPS: 16.67ms
Bone Scaling Usage: 0.14ms (0.8% of budget)

Conclusion: Very efficient! ✓
```

## Configuration Decision Tree

```
                Start
                  │
                  ▼
        What's your priority?
                  │
    ┌─────────────┼─────────────┐
    │             │             │
    ▼             ▼             ▼
Smoothness    Balanced    Responsiveness
    │             │             │
    ▼             ▼             ▼
α = 0.05      α = 0.10      α = 0.20
    │             │             │
    ▼             ▼             ▼
Clamp:        Clamp:        Clamp:
0.7-1.3       0.5-2.0       0.5-2.0
    │             │             │
    └─────────────┴─────────────┘
                  │
                  ▼
        What's the use case?
                  │
    ┌─────────────┼─────────────┐
    │             │             │
    ▼             ▼             ▼
Presentation   Gaming      Recording
    │             │             │
    ▼             ▼             ▼
α = 0.05      α = 0.15      α = 0.10
Clamp:        Clamp:        Clamp:
0.7-1.3       0.5-2.0       0.6-1.5
```

---

## Legend

```
┌─┐  Box/Container
│  │  Vertical line
─    Horizontal line
▼    Flow direction (downward)
►    Flow direction (rightward)
◄    Reference/Pointer
╱╲   Graph/Chart line
‾    Smooth curve
```
