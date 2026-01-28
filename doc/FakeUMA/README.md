# Smoothed Continuous Bone Scaling System

## Overview

This system dynamically adjusts avatar bone lengths in real-time based on Kinect body tracking measurements. It uses exponential smoothing to eliminate jitter while maintaining responsiveness to the user's actual body proportions.

## Key Features

- **Real-time scaling**: Bones are scaled every frame based on live Kinect measurements
- **Exponential smoothing**: Reduces jitter and noise in tracking data
- **Absolute scaling**: Prevents accumulation errors by resetting to original scale each frame
- **Quality validation**: Ignores poor tracking data to maintain stability
- **Configurable responsiveness**: Adjustable smoothing factor via Unity Inspector

## Architecture

### Phase 1: Data Structures

- `_OriginalAvatarBoneLengths`: Baseline bone measurements (set once during initialization)
- `_SmoothedKinectBoneLengths`: Exponentially smoothed Kinect measurements (updated every frame)
- `boneSmoothingFactor`: Controls smoothing intensity (0.01-1.0, default 0.1)
- `_BoneMappingConfigs`: Maps Kinect joint pairs to Unity humanoid bones

### Phase 2: Initialization

1. **InitializeBoneMappingConfigs()**: Sets up mapping between Kinect joints and Unity bones

   - Upper/lower arms (bilateral)
   - Upper/lower legs (bilateral)
   - Optional: spine segments, neck
   - Note: Kinect left → Unity right due to mirroring

2. **MeasureOriginalAvatarBoneLengths()**: Captures baseline bone lengths
   - Measures distance from bone to first child
   - Validates measurements (must be > 0)
   - Logs all measurements for debugging

### Phase 3: Measurement & Smoothing

1. **MeasureRawKinectBoneLength()**: Gets raw distance between Kinect joints

   - Converts joints to Unity world space
   - Returns Euclidean distance in meters

2. **MeasureSmoothedKinectBoneLength()**: Applies exponential smoothing

   - Formula: `smoothed = lerp(previous, current, smoothingFactor)`
   - First measurement: stores raw value directly
   - Subsequent measurements: applies exponential moving average

3. **Quality Validation**:
   - Checks joint tracking state (must be "Tracked", not "Inferred")
   - Validates measurements are within 0.1m to 1.0m range
   - Falls back to previous smoothed value on failure

### Phase 4: Scale Calculation

1. **CalculateBoneScaleFactor()**: Computes scale ratio

   - Formula: `scaleFactor = kinectLength / avatarLength`
   - Safety clamping: 0.5x to 2.0x to prevent extreme scaling
   - Returns 1.0 if no valid measurement exists

2. **GetScaleVectorForBone()**: Converts scalar to Vector3
   - Most bones: `Vector3(1, scaleFactor, 1)` (Y-axis stretch)
   - Maintains other axes at 1.0 for proper bone deformation

### Phase 5: Integration

1. **UpdateContinuousBoneScaling()**: Main update method (called from LateUpdate)

   - Iterates through all configured bones
   - Measures smoothed Kinect length
   - Calculates scale factor
   - Applies absolute scaling via FakeUMA
   - Resets to original scale first to prevent accumulation

2. **Execution Order** (in LateUpdate):
   1. Update bone scaling (lengths match Kinect user)
   2. Apply forward kinematics (rotations from Kinect)
   3. Apply root transformations (position, rotation)

## Configuration

### Smoothing Factor Tuning

Access via Unity Inspector: `boneSmoothingFactor`

- **0.05**: Maximum smoothness, minimal jitter, slower response
- **0.10**: Balanced (recommended default)
- **0.20**: More responsive, slight jitter possible
- **Higher values**: Nearly real-time but may show tracking noise

### Adding New Bones

Edit `InitializeBoneMappingConfigs()` to add entries:

```csharp
new BoneMappingConfig(
    Kinect.JointType.StartJoint,
    Kinect.JointType.EndJoint,
    HumanBodyBones.UnityBone,
    new Vector3(1f, 0f, 1f) // Y-axis stretch
)
```

**Important**: Remember Kinect left/right is mirrored to Unity!

- Kinect ShoulderLeft → Unity RightUpperArm
- Kinect ShoulderRight → Unity LeftUpperArm

## Technical Details

### Exponential Smoothing Formula

```
newValue = alpha * currentMeasurement + (1 - alpha) * previousValue
```

Where `alpha` is the `boneSmoothingFactor`.

Unity's `Mathf.Lerp(a, b, t)` implements this as:

```
result = a + (b - a) * t
```

### Absolute Scaling Implementation

To prevent accumulation errors, we:

1. Calculate inverse of current relative scale
2. Apply inverse to reset bone to original
3. Apply new scale factor

```csharp
Vector3 resetScale = Vector3.one / currentRelativeScale;
fakeUMA.ScaleBoneIndependently(bone, resetScale);
fakeUMA.ScaleBoneIndependently(bone, newScaleVector);
```

### Coordinate System Notes

- **Kinect**: Right-handed, Z-forward, Y-up
- **Unity**: Left-handed, Z-forward, Y-up
- **Mirroring**: Left/right sides are swapped in the mapping
- **Bone stretch axis**: Typically Y-axis (bone grows upward from joint)

## Troubleshooting

### Bones Not Scaling

- Check that `boneSmoothingFactor` > 0
- Verify Kinect tracking state is "Tracked" (watch for green joints)
- Ensure original avatar measurements succeeded (check console logs)
- Confirm bone mapping includes the bone you're testing

### Jittery Motion

- Decrease `boneSmoothingFactor` (try 0.05)
- Check Kinect placement and lighting
- Verify user is within optimal tracking range (1.5-3.5 meters)

### Slow Response

- Increase `boneSmoothingFactor` (try 0.15-0.20)
- Check for measurement validation failures (bad tracking)

### Extreme Scaling

- Verify safety clamping is active (0.5x to 2.0x)
- Check that original avatar measurements are reasonable
- Confirm Kinect measurements are in meters (not mm or cm)

### Bones Accumulating Scale

- Ensure absolute scaling is working (reset + apply pattern)
- Check FakeUMA's `GetBoneScaleFactor()` returns correct values
- Verify no other code is modifying bone scales

## Performance Considerations

- **Per-frame cost**: ~8 bone measurements + 8 scale operations
- **Memory**: ~3 dictionaries with 8-12 entries each
- **Optimization**: Dictionary lookups are O(1), no iterations over full skeleton
- **Scalability**: Add more bones by expanding `_BoneMappingConfigs`

## Future Enhancements

1. **Adaptive smoothing**: Increase smoothing when stationary, decrease during motion
2. **Per-bone smoothing**: Different smoothing factors for arms vs. legs
3. **Confidence weighting**: Reduce smoothing factor when tracking confidence is high
4. **Temporal prediction**: Predict next frame's measurement to reduce latency
5. **Bone thickness scaling**: Scale cross-sectional area based on body mass index

## Integration with Existing Systems

- **FakeUMA**: Provides the underlying scaling mechanism with inverse compensation
- **Forward Kinematics**: Applied after scaling for proper bone rotation
- **Uniform Scaling**: Root transform scaling still applied for overall height matching
- **IK System**: If enabled, runs after all scaling and FK operations

## Debug Logging

Key log messages to watch:

- `"Initialized X bone mapping configurations"`: Confirms setup
- `"Measuring Original Avatar Bone Lengths"`: Shows baseline measurements
- `"Bone: X: Y meters"`: Individual bone measurements
- `"Measurement Complete: X successful, Y failed"`: Validation summary

Enable detailed per-frame logging by adding Debug.Log statements in `UpdateContinuousBoneScaling()`.
