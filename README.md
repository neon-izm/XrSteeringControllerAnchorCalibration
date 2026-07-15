# XR Steering Controller Anchor Calibration

Unity package that estimates a rigid **ModelView** (HMD tracking space → CG handle space) from a partial hand-tracking arc, a known CG handle pose, and HMD head **position**.

Designed for **VR-HMD** tracking spaces where **`Vector3.up` is true world up** (gravity / IMU). That fact pins handle roll without using head pitch/roll.

Version **0.5.0**

## Install

**Window > Package Manager > + > Add package from git URL...**

```
https://github.com/neon-izm/XrSteeringControllerAnchorCalibration.git?path=Packages/com.neon-izm.xr-steering-controller-anchor-calibration
```

Or in `Packages/manifest.json`:

```json
{
  "dependencies": {
    "com.neon-izm.xr-steering-controller-anchor-calibration": "https://github.com/neon-izm/XrSteeringControllerAnchorCalibration.git?path=Packages/com.neon-izm.xr-steering-controller-anchor-calibration#v0.5.0"
  }
}
```

## Requirements

- Unity 6 (6000.3+ recommended)
- VR-HMD / OpenXR-style tracking with gravity-aligned world up
- Edit Mode tests: `com.unity.test-framework`

## Calibrate

```csharp
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

var targetHandle = new CgHandlePose(cgHandle.position, cgHandle.rotation);
// Forward = vehicle front.

IReadOnlyList<Vector3> worldPoints = trackedArcPoints;
var headPose = HeadPose.FromTransform(hmdTransform); // Position used for front/back only

CalibrationResult result = AnchorCalibration.Calibrate(worldPoints, targetHandle, headPose);
Matrix4x4 modelView = result.ModelView;

Vector3 cgPoint = AnchorCalibration.WorldToCg(worldPoint, modelView);
```

What `Calibrate` does:

1. MSAC 3D circle fit (+ Taubin refine)
2. Orient the circle with world up (`Vector3.up`) so in-plane roll is fixed
3. Build two front/back candidates and pick the one where the mapped head is on the driver-seat side
4. Remove remaining twist about the CG handle forward

Notes:

- CG handle **radius is unknown**; ModelView is rigid (scale = 1)
- Head **rotation is unused** by `Calibrate`
- Point CG handle forward toward vehicle front so front/back scoring is consistent

## Apply the result (XRI)

`Calibrate` only returns ModelView. Apply it by keeping **XR Origin** level and moving the CG content:

```csharp
RigCalibrationOffset.ApplyContentRootAlignment(
    result.ModelView,
    vehicleRoot,   // content root (usually cgHandle.parent)
    cgHandle,
    out ContentRootAlignmentResult align);
```

This places the content from ModelView, then twists about the handle hub so `handle.up` matches world up on the wheel plane.

Typical scene:

```
Scene
├── XR Origin
│   └── Camera Offset
│       └── Main Camera
└── VehicleRoot          ← content root
    └── SteeringWheel    ← CgHandlePose
```

## Sample

Import **Calibration Sample** from Package Manager, or open `Assets/CalibrationSample/CalibrationSample.unity` in this repo.

Inspector flow: Generate Sample Points → Run Calibration → Clear.

## Layout

```
Packages/com.neon-izm.xr-steering-controller-anchor-calibration/
├── Runtime/       # AnchorCalibration, RigCalibrationOffset, circle fitting
├── Tests/Editor/  # Edit Mode tests (+ device session fixtures)
└── Samples~/      # Calibration Sample
```

## Tests

**Window > General > Test Runner** (Edit Mode).

To run package tests from a consumer project:

```json
{
  "testables": [
    "com.neon-izm.xr-steering-controller-anchor-calibration"
  ]
}
```

## License

MIT License. See [LICENSE](LICENSE).
