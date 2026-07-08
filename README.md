# XR Steering Controller Anchor Calibration

Unity package for calibrating the rigid **ModelView** transform between HMD hand-tracking space and CG handle space, using a partial arc of tracked points and a known CG handle pose.

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

Pin a tag or commit by appending `#v0.1.0` or `#<commit-hash>`.

## Requirements

- Unity 6 (6000.3+ recommended)
- `com.unity.test-framework` (included in consumer projects that run Edit Mode tests)

## Quick start

### API

```csharp
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

// Known CG handle pose (position + rotation, roll=0). Radius is unknown.
var targetHandle = new CgHandlePose(cgHandleTransform.position, cgHandleTransform.rotation);

// HMD world-space tracking points along the handle arc
IReadOnlyList<Vector3> worldPoints = trackedPoints;

// Calibrate: output is ModelView only (rigid, scale = 1)
CalibrationResult result = AnchorCalibration.Calibrate(worldPoints, targetHandle);
Matrix4x4 modelView = result.ModelView;

// Map tracking points into CG space
Vector3 cgPoint = AnchorCalibration.WorldToCg(worldPoint, modelView);
```

### Assumptions

- Handle rotation has **roll = 0** (forward = circle plane normal).
- CG handle **radius is unknown**; calibration returns a rigid ModelView (no scale).
- Normal sign (forward vs back) is resolved using the CG handle forward hemisphere (pitch-up mount). Fully horizontal axes are ambiguous and not handled.
- Robust fitting uses **MSAC** for outlier resistance.

## Sample

After installing the package, open **Package Manager**, select **XR Steering Controller Anchor Calibration**, and import the **Calibration Sample** under Samples.

The sample provides:

- `SteeringAnchorCalibrationSample` component
- Inspector buttons: **Generate Sample Points** → **Run Calibration**
- Scene View gizmos (CG handle axes, fitted circles, mapped inliers)

## Package layout

```
Packages/com.neon-izm.xr-steering-controller-anchor-calibration/
├── package.json
├── Runtime/          # Core library
├── Editor/           # Sample inspector & scene setup utilities
├── Tests/Editor/     # Edit Mode tests
└── Samples~/         # Optional sample scene (import from Package Manager)
```

## Tests

Edit Mode tests live in the package `Tests/Editor` folder. Run them from **Window > General > Test Runner** (Edit Mode).

To run package tests in your project, add the package name to `testables` in `Packages/manifest.json`:

```json
{
  "testables": [
    "com.neon-izm.xr-steering-controller-anchor-calibration"
  ]
}
```

### Local development (this repository)

This repo is a Unity development project that embeds the package locally:

```json
"com.neon-izm.xr-steering-controller-anchor-calibration": "file:com.neon-izm.xr-steering-controller-anchor-calibration"
```

## License

MIT License. See [LICENSE](LICENSE).
