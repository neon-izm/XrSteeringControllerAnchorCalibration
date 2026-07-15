# XR Steering Controller Anchor Calibration

HMD のハンドトラッキング軌跡（部分円弧）、既知の CG ハンドル姿勢、HMD の**頭位置**から、トラッキング空間 → CG 空間への剛体 **ModelView** を求める Unity パッケージです。

**VR-HMD** を前提とし、トラッキング空間では **`Vector3.up` が真上**（重力 / IMU の水平面）であることを使ってハンドル roll を固定します。

UPM 構成は [uOSC](https://github.com/hecomi/uOSC) を参考にしています。

**現行バージョン: 0.4.0**

## インストール

### Git URL（UPM）

**Window > Package Manager > + > Add package from git URL...** に以下を入力:

```
https://github.com/neon-izm/XrSteeringControllerAnchorCalibration.git?path=Packages/com.neon-izm.xr-steering-controller-anchor-calibration
```

または `Packages/manifest.json` に追加:

```json
{
  "dependencies": {
    "com.neon-izm.xr-steering-controller-anchor-calibration": "https://github.com/neon-izm/XrSteeringControllerAnchorCalibration.git?path=Packages/com.neon-izm.xr-steering-controller-anchor-calibration"
  }
}
```

タグやコミットを固定する場合は末尾に `#v0.4.0` や `#<commit-hash>` を付けます。

## 動作要件

- Unity 6（6000.3 以降推奨）
- `Vector3.up` が重力基準の真上である VR-HMD / OpenXR 系トラッキング
- Edit Mode テスト実行時は `com.unity.test-framework`

## 使い方

### API

```csharp
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

// 既知の CG ハンドル姿勢（位置・回転）。半径は未知。
// Forward = 車の進行方向（Front）。
var targetHandle = new CgHandlePose(cgHandleTransform.position, cgHandleTransform.rotation);

IReadOnlyList<Vector3> worldPoints = trackedPoints;

// HeadPose: Calibrate が使うのは Position のみ（前後判定）。
// Rotation は未使用。FromTransform で渡してよい。
var headPose = HeadPose.FromTransform(hmdTransform);

CalibrationResult result = AnchorCalibration.Calibrate(worldPoints, targetHandle, headPose);
Matrix4x4 modelView = result.ModelView;

Vector3 cgPoint = AnchorCalibration.WorldToCg(worldPoint, modelView);
```

### 前提

- CG ハンドルの **半径は未知**。出力は剛体 ModelView（scale=1）のみ。
- **前後**は頭の **位置**だけで解決（運転席側 / `targetHandle.Forward`）。
- **roll** は VR-HMD のワールド up（`OrientCircleWithWorldUp`）で決め、続けて `RemoveHandleLocalRoll`。**頭の pitch/roll は使わない**。
- 外れ値耐性: **MSAC**（Taubin refine）。
- CG ハンドル forward は車の Front 向き（Front/Back スコアが一貫する前提）。

### パイプライン

1. MSAC で 3D 円フィット
2. ワールド up で円平面内位相を決め roll=0
3. 前後 2 候補（平面内法線反転）
4. 写像後の頭が運転席側の候補を採用
5. CG ハンドル軸まわりの局所 roll を除去

### XR Origin への適用（XRI の `XRRig`）

`Calibrate` は **ModelView のみ**返します。CG はシーン固定のまま、**XR Origin** を `RigCalibrationOffset` + `PoseConstraints`（デフォルト水平制約: `RemoveRoll`）で動かします。

| API | 目的 |
|-----|------|
| `OrientCircleWithWorldUp` / `RemoveHandleLocalRoll` | キャリブ時のハンドル / 円の roll |
| `RigCalibrationOffset` | ModelView → XROrigin の rig offset（CG は固定） |
| `PoseConstraints.RemoveRoll` | 適用後のトラッキング原点のワールド roll 除去 |
| `PoseConstraints.ComputeTrackingOriginPose` | 水平オプション + CG ハンドル位置を pivot |

典型的な階層:

```
Scene
├── XR Origin                 ← XROrigin（XRI Starter Assets / XRRig）
│   └── Camera Offset
│       └── Main Camera
└── VehicleRoot               ← CG ルート（シーン固定）
    └── SteeringWheel         ← CgHandlePose / 水平制約の pivot
```

コピペ用:

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

`TrackingHorizonConstraint`: `None`, `RemoveRoll`（推奨）, `RemovePitch`, `RemovePitchAndRoll`。

## サンプル

Package Manager から **Calibration Sample** を Importするか、本リポジトリの `Assets/CalibrationSample/` を使います。

- シーン: `Assets/CalibrationSample/CalibrationSample.unity`
- `SteeringAnchorCalibrationSample`: Generate → Run Calibration → Clear
- ギズモと Inspector 指標（前後スコア、ハンドル局所 roll）

## パッケージ構成

```
Packages/com.neon-izm.xr-steering-controller-anchor-calibration/
├── package.json
├── Runtime/          # AnchorCalibration, PoseConstraints, RigCalibrationOffset, 円フィット
├── Tests/Editor/     # Edit Mode テスト
└── Samples~/         # Calibration Sample
```

## テスト

**Window > General > Test Runner**（Edit Mode）。他プロジェクトでパッケージテストを走らせる場合:

```json
{
  "testables": [
    "com.neon-izm.xr-steering-controller-anchor-calibration"
  ]
}
```

## ライセンス

MIT License。詳細は [LICENSE](LICENSE) を参照してください。
