# Continuous Bone Scaling - Quick Reference

## Inspector Configuration

### Bone Smoothing Factor

**Location**: AvatarController component → Bone Smoothing Factor slider
**Range**: 0.01 to 1.0
**Presets**:

- **0.05** - Ultra smooth (slow response, zero jitter)
- **0.10** - Balanced (recommended)
- **0.15** - Responsive (slight smoothing)
- **0.20** - Near real-time (minimal smoothing)

## Testing Checklist

### Initial Setup

- [ ] Unity scene loaded with avatar and Kinect
- [ ] AvatarController component attached to avatar
- [ ] Kinect sensor connected and tracking
- [ ] Console shows: "Initialized X bone mapping configurations"
- [ ] Console shows: "Measurement Complete: 8 successful, 0 failed"

### Runtime Verification

- [ ] Avatar moves with user in real-time
- [ ] Arm lengths adjust to match user's arms
- [ ] Leg lengths adjust to match user's legs
- [ ] No visible jitter or shaking
- [ ] Scaling is smooth and natural
- [ ] Arms don't detach or distort when reaching
- [ ] Legs maintain proper proportions when walking

### Troubleshooting Steps

#### Issue: No scaling happens

1. Check console for "Initialized X bone mapping" message
2. Verify Kinect is tracking (green skeleton on sensor)
3. Ensure `boneSmoothingFactor` > 0 in Inspector
4. Check that Update() and LateUpdate() are being called

#### Issue: Extreme scaling (arms too long/short)

1. Verify original avatar measurements in console logs
2. Check Kinect distance (should be 1.5-3.5 meters)
3. Look for measurement validation failures
4. Adjust safety clamping range if needed (in code: 0.5f to 2.0f)

#### Issue: Jittery or shaky bones

1. Decrease `boneSmoothingFactor` to 0.05
2. Improve Kinect lighting conditions
3. Clear space around user (remove obstacles)
4. Check for clothing that obscures joints

#### Issue: Slow to respond to changes

1. Increase `boneSmoothingFactor` to 0.15-0.20
2. Check for measurement validation rejecting good data
3. Verify tracking state is "Tracked" not "Inferred"

#### Issue: Bones accumulate scale over time

1. Check that FakeUMA initialization succeeded
2. Verify reset-then-apply pattern in UpdateContinuousBoneScaling()
3. Look for other scripts modifying bone scales

## Code Modification Guide

### Adding a New Bone to Scale

**Step 1**: Find the Kinect joint names

```csharp
// Example: Adding hand bones
Kinect.JointType.WristLeft
Kinect.JointType.HandLeft
```

**Step 2**: Identify the Unity humanoid bone

```csharp
// Remember: Kinect left = Unity right!
HumanBodyBones.RightHand
```

**Step 3**: Add to InitializeBoneMappingConfigs()

```csharp
new BoneMappingConfig(
    Kinect.JointType.WristLeft,      // Start joint
    Kinect.JointType.HandLeft,       // End joint
    HumanBodyBones.RightHand,        // Unity bone (mirrored!)
    new Vector3(1f, 0f, 1f)          // Y-axis stretch
),
```

**Step 4**: Test and verify in console logs

### Changing Measurement Bounds

**Location**: `MeasureSmoothedKinectBoneLength()` method

```csharp
// Current: 0.1m to 1.0m range for limbs
if (rawMeasurement < 0.1f || rawMeasurement > 1.0f)

// For spine segments (shorter):
if (rawMeasurement < 0.05f || rawMeasurement > 0.5f)

// For full arms/legs (longer):
if (rawMeasurement < 0.2f || rawMeasurement > 1.2f)
```

### Adjusting Safety Clamping

**Location**: `CalculateBoneScaleFactor()` method

```csharp
// Current: 0.5x to 2.0x range
scaleFactor = Mathf.Clamp(scaleFactor, 0.5f, 2.0f);

// More conservative (less variation):
scaleFactor = Mathf.Clamp(scaleFactor, 0.7f, 1.3f);

// More permissive (wider range):
scaleFactor = Mathf.Clamp(scaleFactor, 0.3f, 3.0f);
```

### Enabling Spine/Neck Scaling

**Location**: `InitializeBoneMappingConfigs()` method

Uncomment these lines:

```csharp
// Spine segments
new BoneMappingConfig(Kinect.JointType.SpineBase, Kinect.JointType.SpineMid, HumanBodyBones.Hips, yAxis),
new BoneMappingConfig(Kinect.JointType.SpineMid, Kinect.JointType.SpineShoulder, HumanBodyBones.Spine, yAxis),
new BoneMappingConfig(Kinect.JointType.SpineShoulder, Kinect.JointType.Neck, HumanBodyBones.Chest, yAxis),

// Neck
new BoneMappingConfig(Kinect.JointType.Neck, Kinect.JointType.Head, HumanBodyBones.Neck, yAxis),
```

**Note**: Spine scaling may need different measurement bounds (shorter segments).

### Per-Bone Smoothing Factors

**Current**: All bones use the same `boneSmoothingFactor`

**To customize**: Modify `BoneMappingConfig` class to include a smoothing factor field:

```csharp
class BoneMappingConfig
{
    // ...existing fields...
    public float smoothingFactor = 0.1f;  // Add this

    public BoneMappingConfig(Kinect.JointType start, Kinect.JointType end,
                            HumanBodyBones bone, Vector3 axis, float smoothing = 0.1f)
    {
        // ...existing code...
        smoothingFactor = smoothing;
    }
}
```

Then use `config.smoothingFactor` instead of `boneSmoothingFactor` in the Lerp call.

## Performance Monitoring

### Expected Frame Times

- **8 bones**: ~0.1-0.2ms per frame
- **16 bones**: ~0.2-0.4ms per frame
- **32 bones**: ~0.4-0.8ms per frame

### Profiling in Unity

1. Open Window → Analysis → Profiler
2. Look for "UpdateContinuousBoneScaling" in Timeline
3. Check CPU time and memory allocations
4. Should see minimal GC allocations (dictionaries pre-allocated)

## Common Configuration Scenarios

### Scenario 1: Maximum Smoothness (Presentation Mode)

```
boneSmoothingFactor = 0.03
Safety clamping = 0.7f to 1.3f
Measurement bounds = 0.15f to 0.9f
```

### Scenario 2: Maximum Responsiveness (Gaming)

```
boneSmoothingFactor = 0.25
Safety clamping = 0.5f to 2.0f
Measurement bounds = 0.05f to 1.2f
```

### Scenario 3: Balanced (General Use)

```
boneSmoothingFactor = 0.10
Safety clamping = 0.5f to 2.0f
Measurement bounds = 0.1f to 1.0f
```

### Scenario 4: Children (Smaller Bodies)

```
boneSmoothingFactor = 0.10
Safety clamping = 0.3f to 1.5f
Measurement bounds = 0.05f to 0.8f
```

### Scenario 5: Athletes (Larger Bodies)

```
boneSmoothingFactor = 0.10
Safety clamping = 0.8f to 2.5f
Measurement bounds = 0.15f to 1.2f
```

## Console Log Analysis

### Successful Initialization

```
Initialized 8 bone mapping configurations.
=== Measuring Original Avatar Bone Lengths ===
  RightUpperArm: 0.2543 meters
  LeftUpperArm: 0.2541 meters
  RightLowerArm: 0.2318 meters
  LeftLowerArm: 0.2315 meters
  RightUpperLeg: 0.4012 meters
  LeftUpperLeg: 0.4015 meters
  RightLowerLeg: 0.4201 meters
  LeftLowerLeg: 0.4198 meters
=== Measurement Complete: 8 successful, 0 failed ===
```

### Warning Signs

```
"Bone X not found in animator. Skipping."
→ Check that avatar has proper humanoid rig

"Invalid bone length (0.0000) for X. Skipping."
→ Bone has no children or measurement failed

"Invalid avatar bone length: X. Cannot calculate scale factor."
→ Original measurement was invalid or zero
```

## Integration Notes

### Works With

- ✅ FakeUMA bone scaling system
- ✅ Forward kinematics (applied after scaling)
- ✅ Root transform positioning/rotation
- ✅ Uniform scaling (applied to root)

### May Conflict With

- ⚠️ Other bone scaling scripts (disable them)
- ⚠️ Animation systems that modify bone scales
- ⚠️ IK solvers that expect fixed bone lengths (adjust IK after scaling)

### Execution Order

1. UpdateContinuousBoneScaling() - scales bones
2. ApplyForwardKinematics() - rotates bones
3. RotateAvatarBasedOnShoulders() - rotates root
4. Position root at spine base
5. Apply uniform scaling to root
6. Apply shoulder translation correction

## Debug Visualization (Optional Addition)

To visualize bone measurements in the scene, add this to `UpdateContinuousBoneScaling()`:

```csharp
// Inside the foreach loop, after getting bone transform
if (boneTransform != null && boneTransform.childCount > 0)
{
    Transform child = boneTransform.GetChild(0);
    Debug.DrawLine(boneTransform.position, child.position, Color.green);

    // Show scale factor as text (requires UnityEditor)
    #if UNITY_EDITOR
    UnityEditor.Handles.Label(boneTransform.position, $"{scaleFactor:F2}x");
    #endif
}
```

This will draw green lines for bones and show their scale factors in the Scene view.
