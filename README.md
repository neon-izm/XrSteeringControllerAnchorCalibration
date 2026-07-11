# XR Steering Controller Anchor Calibration

Unity package for calibrating the rigid **ModelView** transform between HMD hand-tracking space and CG handle space, using a partial arc of tracked points, a known CG handle pose, and HMD head pose.

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

Pin a tag or commit by appending `#v0.2.0` or `#<commit-hash>`.

## Requirements

- Unity 6 (6000.3+ recommended)
- `com.unity.test-framework` (included in consumer projects that run Edit Mode tests)

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

// Representative head pose during the arc (position + rotation from HMD)
var headPose = new HeadPose(hmdTransform.position, hmdTransform.rotation);

// Calibrate: output is ModelView only (rigid, scale = 1)
CalibrationResult result = AnchorCalibration.Calibrate(worldPoints, targetHandle, headPose);
Matrix4x4 modelView = result.ModelView;

// Map tracking points into CG space
Vector3 cgPoint = AnchorCalibration.WorldToCg(worldPoint, modelView);
```

### Assumptions

- CG handle **radius is unknown**; calibration returns a rigid ModelView (no scale).
- **Front / back** ambiguity is resolved from **head position**: the solution where the mapped head is on the driver-seat side (behind the handle relative to `targetHandle.Forward`) is selected.
- **Roll** around the handle axis is removed in CG space after alignment (`RemoveHandleLocalRoll`), relative to the known CG handle pose. **HMD roll is not assumed to be zero** and is not used as the roll reference.
- Head rotation is used only as a weak hint for arc phase on the fitted circle plane; front/back selection uses head **position** only.
- Robust fitting uses **MSAC** for outlier resistance.
- CG handle forward should point toward the vehicle front so Front/Back scoring is consistent.

### Pipeline (summary)

1. MSAC 3D circle fit from tracking points
2. Orient fitted circle using head pose hint (in-plane phase)
3. Build two front/back ModelView candidates (normal flip in plane)
4. Select candidate where mapped head is on the driver-seat side
5. Remove handle-local roll relative to `targetHandle`

## Sample

After installing the package, open **Package Manager**, select **XR Steering Controller Anchor Calibration**, and import the **Calibration Sample** under Samples. That imports the sample scripts and scene into your project.

This development repository keeps the same sample under `Assets/CalibrationSample/` for local work:

- Scene: `Assets/CalibrationSample/CalibrationSample.unity`
- Runtime scripts: `Assets/CalibrationSample/Runtime/Scripts/`
- Editor scripts: `Assets/CalibrationSample/Editor/Scripts/`

The sample provides:

- `SteeringAnchorCalibrationSample` component (`XrSteeringControllerAnchorCalibration.Sample`) with **CgHandle**, **Head**, and arc center references
- Inspector: **Generate Sample Points** → **Run Calibration** → **Clear**
- Scene View gizmos: CG handle axes, head, user-forward arrow, fitted/mapped circles, inliers
- After calibration: red **Head (Mapped)** clone at the ModelView-mapped head pose (Editor)
- Inspector metrics: front/back score, handle-local roll, driver-seat vs bonnet verdict

## Package layout

```
Packages/com.neon-izm.xr-steering-controller-anchor-calibration/
├── package.json
├── Runtime/          # Core library (AnchorCalibration, circle fitting)
├── Tests/Editor/     # Edit Mode tests
└── Samples~/         # Optional sample (import from Package Manager)

Assets/CalibrationSample/   # Local dev copy of the sample (not shipped in the UPM package)
├── CalibrationSample.unity
├── Runtime/Scripts/
├── Runtime/SampleCar/
└── Editor/Scripts/
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
