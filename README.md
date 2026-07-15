# XR Steering Controller Anchor Calibration

Unity package that estimates a rigid **ModelView** (tracking space → CG handle space) from a partial hand-tracking arc, a known CG handle pose, and HMD **head position**.

Requires a **VR-HMD tracking space** where **`Vector3.up` is true world up** (gravity / IMU horizontal plane). That constraint is used to pin handle roll.

**Current version: 0.4.0**

## Install

### Git URL (UPM)

Open **Window > Package Manager > + > Add package from git URL...** and enter:

```
https://github.com/neon-izm/XrSteeringControllerAnchorCalibration.git?path=Packages/com.neon-izm.xr-steering-controller-anchor-calibration
```

Or add to `Packages/manifest.json`:

```json
{
  "dependencies": {
    "com.neon-izm.xr-steering-controller-anchor-calibration": "https://github.com/neon-izm/XrSteeringControllerAnchorCalibration.git?path=Packages/com.neon-izm.xr-steering-controller-anchor-calibration"
  }
}
```

Pin a tag or commit by appending `#v0.4.0` or `#<commit-hash>`.

## Requirements

- Unity 6 (6000.3+ recommended)
- VR-HMD / OpenXR-style tracking where world up is gravity-aligned (`Vector3.up`)
- `com.unity.test-framework` (for Edit Mode tests)

## Quick start

### API

```csharp
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

// Known CG handle pose (position + rotation). Radius is unknown.
// Forward = vehicle front direction.
var targetHandle = new CgHandlePose(cgHandleTransform.position, cgHandleTransform.rotation);

// HMD world-space tracking points along the handle arc
IReadOnlyList<Vector3> worldPoints = trackedPoints;

// HeadPose: Calibrate uses Position only (front/back).
// Rotation is unused; pass HMD rotation for convenience (e.g. FromTransform).
var headPose = HeadPose.FromTransform(hmdTransform);

CalibrationResult result = AnchorCalibration.Calibrate(worldPoints, targetHandle, headPose);
Matrix4x4 modelView = result.ModelView;

Vector3 cgPoint = AnchorCalibration.WorldToCg(worldPoint, modelView);
```

### Assumptions

- CG handle **radius is unknown**; output is a rigid ModelView (scale = 1).
- **Front / back** is resolved from **head position** only (driver-seat side vs `targetHandle.Forward`).
- **Roll** uses VR-HMD world up: `OrientCircleWithWorldUp` (`Vector3.up`), then `RemoveHandleLocalRoll` vs the known CG handle. **Head pitch/roll are not used.**
- Robust fitting: **MSAC** (+ Taubin refine).
- CG handle forward should point vehicle front so Front/Back scoring is consistent.

### Pipeline

1. MSAC 3D circle fit
2. Orient circle with world up → roll = 0 on the circle plane
3. Two front/back ModelView candidates (in-plane normal flip)
4. Pick the candidate where mapped head is on the driver-seat side
5. Remove handle-local roll relative to `targetHandle`

### Applying to XR Origin (XRI `XRRig`)

`Calibrate` returns **ModelView only**. Keep CG content scene-fixed; move **XR Origin** with `RigCalibrationOffset` + `PoseConstraints` (default horizon: `RemoveRoll`).

| API | Purpose |
|-----|---------|
| `OrientCircleWithWorldUp` / `RemoveHandleLocalRoll` | Calibration-time roll (handle / circle) |
| `RigCalibrationOffset` | ModelView → XROrigin rig offset (content stays fixed) |
| `PoseConstraints.RemoveRoll` | World roll removal on the tracking origin after apply |
| `PoseConstraints.ComputeTrackingOriginPose` | Horizon options + pivot at CG handle world position |

Typical hierarchy:

```
Scene
├── XR Origin                 ← XROrigin (XRI Starter Assets / XRRig)
│   └── Camera Offset
│       └── Main Camera
└── VehicleRoot               ← scene-fixed CG root
    └── SteeringWheel         ← CgHandlePose + horizon pivot
```

Copy-paste apply helper:

```csharp
using Unity.XR.CoreUtils;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

public sealed class SteeringCalibrationApply : MonoBehaviour
{
    [SerializeField] XROrigin xrOrigin;
    [SerializeField] Transform vehicleRoot;
    [SerializeField] Transform cgHandle;

    Vector3? sessionOriginPosition;
    Quaternion? sessionOriginRotation;

    public void Apply(CalibrationResult result)
    {
        CaptureSessionOriginIfNeeded();

        RigCalibrationOffset.ApplyCalibrationToTrackingOrigin(
            result.ModelView,
            xrOrigin.transform,
            vehicleRoot,
            cgHandle,
            CalibrationOptions.Default);
    }

    public void ResetCalibrationOffset()
    {
        if (!sessionOriginPosition.HasValue || !sessionOriginRotation.HasValue)
        {
            return;
        }

        xrOrigin.transform.SetPositionAndRotation(
            sessionOriginPosition.Value,
            sessionOriginRotation.Value);
    }

    void CaptureSessionOriginIfNeeded()
    {
        if (sessionOriginPosition.HasValue)
        {
            return;
        }

        sessionOriginPosition = xrOrigin.transform.position;
        sessionOriginRotation = xrOrigin.transform.rotation;
    }
}
```

`TrackingHorizonConstraint`: `None`, `RemoveRoll` (recommended), `RemovePitch`, `RemovePitchAndRoll`.

## Sample

Import **Calibration Sample** from Package Manager, or use `Assets/CalibrationSample/` in this repo.

- Scene: `Assets/CalibrationSample/CalibrationSample.unity`
- `SteeringAnchorCalibrationSample`: Generate → Run Calibration → Clear
- Gizmos + Inspector metrics (front/back score, handle-local roll)

## Package layout

```
Packages/com.neon-izm.xr-steering-controller-anchor-calibration/
├── package.json
├── Runtime/          # AnchorCalibration, PoseConstraints, RigCalibrationOffset, circle fitting
├── Tests/Editor/     # Edit Mode tests
└── Samples~/         # Calibration Sample
```

## Tests

**Window > General > Test Runner** (Edit Mode). For package tests in a consumer project, add to `Packages/manifest.json`:

```json
{
  "testables": [
    "com.neon-izm.xr-steering-controller-anchor-calibration"
  ]
}
```

## License

MIT License. See [LICENSE](LICENSE).
