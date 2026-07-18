# XR Steering Controller Anchor Calibration

HMD のハンドトラッキング軌跡（部分円弧）、既知の CG ハンドル姿勢、HMD の頭の**位置**から、トラッキング空間 → CG 空間への剛体 **ModelView** を求める Unity パッケージです。

**VR-HMD** 向けで、トラッキング空間の **`Vector3.up` が真上**（重力 / IMU）であることを前提にハンドル roll を固定します。頭の pitch/roll は使いません。

バージョン **0.6.0**

UPM 構成は [uOSC](https://github.com/hecomi/uOSC) を参考にしています。

## インストール

**Window > Package Manager > + > Add package from git URL...**

```
https://github.com/neon-izm/XrSteeringControllerAnchorCalibration.git?path=Packages/com.neon-izm.xr-steering-controller-anchor-calibration
```

または `Packages/manifest.json`:

```json
{
  "dependencies": {
    "com.neon-izm.xr-steering-controller-anchor-calibration": "https://github.com/neon-izm/XrSteeringControllerAnchorCalibration.git?path=Packages/com.neon-izm.xr-steering-controller-anchor-calibration#v0.6.0"
  }
}
```

## 動作要件

- Unity 6（6000.3 以降推奨）
- `Vector3.up` が重力基準の真上である VR-HMD / OpenXR 系トラッキング
- Edit Mode テスト時は `com.unity.test-framework`

## キャリブレーション

```csharp
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

var targetHandle = new CgHandlePose(cgHandle.position, cgHandle.rotation);
// Forward = 車の進行方向（Front）

IReadOnlyList<Vector3> worldPoints = trackedArcPoints;
var headPose = HeadPose.FromTransform(hmdTransform); // 使うのは Position（前後判定）のみ

CalibrationResult result = AnchorCalibration.Calibrate(worldPoints, targetHandle, headPose);
Matrix4x4 modelView = result.ModelView;

Vector3 cgPoint = AnchorCalibration.WorldToCg(worldPoint, modelView);
```

`Calibrate` の流れ:

1. MSAC で 3D 円フィット（Taubin refine）
2. ワールド up（`Vector3.up`）で円平面内の位相を決め roll を固定
3. 前後 2 候補から、写像後の頭が運転席側になる方を採用
4. CG ハンドル軸まわりの残り twist を除去

補足:

- CG ハンドルの **半径は未知**。ModelView は剛体（scale=1）
- 頭の **回転は未使用**
- CG ハンドルの forward は車の Front 向きにしてください（前後スコアが一貫します）

## 結果の適用（XRI）

`Calibrate` は ModelView だけ返します。**XR Origin は水平のまま**、CG コンテンツを動かして適用します。

```csharp
RigCalibrationOffset.ApplyContentRootAlignment(
    result.ModelView,
    vehicleRoot,   // コンテンツルート（通常は cgHandle.parent）
    cgHandle,
    out ContentRootAlignmentResult align);

// 任意: handle.forward のワールド pitch も水平化（車の前傾・後傾を消す）
RigCalibrationOffset.ApplyContentRootAlignment(
    result.ModelView,
    vehicleRoot,
    cgHandle,
    ContentPitchMode.LevelToHorizon,
    out align);
```

ModelView でコンテンツを置き、ハンドル hub まわりにツイストして `handle.up` をワールド up の平面投影に揃えます。pitch の既定は `PreserveFromCalibration`（CG の pitch を真として残す）です。HMD 上で車が前傾・後傾して見えないようにしたい場合は `ContentPitchMode.LevelToHorizon` を渡してください。

典型的なシーン:

```
Scene
├── XR Origin
│   └── Camera Offset
│       └── Main Camera
└── VehicleRoot          ← コンテンツルート
    └── SteeringWheel    ← CgHandlePose
```

## サンプル

Package Manager から **Calibration Sample** を Importするか、本リポジトリの `Assets/CalibrationSample/CalibrationSample.unity` を開いてください。

Inspector: Generate Sample Points → Run Calibration → Clear

## 構成

```
Packages/com.neon-izm.xr-steering-controller-anchor-calibration/
├── Runtime/       # AnchorCalibration, RigCalibrationOffset, 円フィット
├── Tests/Editor/  # Edit Mode テスト（実機セッション fixtures 含む）
└── Samples~/      # Calibration Sample
```

## テスト

**Window > General > Test Runner**（Edit Mode）

他プロジェクトでパッケージテストを走らせる場合:

```json
{
  "testables": [
    "com.neon-izm.xr-steering-controller-anchor-calibration"
  ]
}
```

## ライセンス

MIT License。詳細は [LICENSE](LICENSE) を参照してください。
