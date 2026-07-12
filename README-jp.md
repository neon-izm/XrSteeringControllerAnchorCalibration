# XR Steering Controller Anchor Calibration

HMD のハンドトラッキング軌跡（部分円弧）、既知の CG ハンドル姿勢、HMD 頭姿勢から、トラッキング空間 → CG 空間への剛体 **ModelView** 行列を求める Unity パッケージです。

UPM 配布の構成は [uOSC](https://github.com/hecomi/uOSC) を参考にしています。

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

タグやコミットを固定する場合は末尾に `#v0.2.0` や `#<commit-hash>` を付けます。

## 動作要件

- Unity 6（6000.3 以降推奨）
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

// HMD 世界座標の軌跡点
IReadOnlyList<Vector3> worldPoints = trackedPoints;

// 軌跡取得時の代表頭姿勢（HMD の位置・回転）
var headPose = new HeadPose(hmdTransform.position, hmdTransform.rotation);

// キャリブレーション結果は ModelView のみ（剛体・scale=1）
CalibrationResult result = AnchorCalibration.Calibrate(worldPoints, targetHandle, headPose);
Matrix4x4 modelView = result.ModelView;

// トラッキング点を CG 空間へ変換
Vector3 cgPoint = AnchorCalibration.WorldToCg(worldPoint, modelView);
```

### 前提

- CG ハンドルの **半径は未知**。キャリブレーション出力は剛体 ModelView のみ（スケールなし）。
- **前後の不定性**は **頭の位置**で解決する。写像後の頭がハンドル後方（運転席側、`targetHandle.Forward` の Back 側）になる候補を採用する。
- **ハンドル軸まわりの roll** は、既知 CG ハンドル姿勢を基準にキャリブ後処理で除去する（`RemoveHandleLocalRoll`）。**HMD の roll を 0 とみなさない**し、roll の基準にも使わない。
- 頭の **回転**はフィット円の平面内位相の弱いヒントにのみ使う。前後選択は頭の **位置**のみ。
- 外れ値耐性のため **MSAC** による 3D 円フィッティングを使用。
- CG ハンドルの forward は車の Front 方向を向いている前提（Front/Back スコアの符号が一貫する）。

### パイプライン（概要）

1. 軌跡点から MSAC で 3D 円フィット
2. 頭姿勢ヒントで円の平面内位相を決定
3. 法線反転を含む前後 2 候補の ModelView を生成
4. 写像後の頭が運転席側になる候補を選択
5. CG ハンドル軸まわりの余分な roll を除去

### 水平維持（XR Origin 適用時）

`Calibrate` の出力は **ModelView のみ**で、ワールド水平制約は含みません。ModelView を XR トラッキング原点へ適用したあと、`PoseConstraints` で水平を維持しつつ CG ハンドルのワールド位置を pivot として固定してください。

| API | 目的 |
|-----|------|
| `RemoveHandleLocalRoll` | キャリブ時に **CG ハンドル forward 軸**まわりの twist を除去 |
| `RigCalibrationOffset` | `ModelView` から rig offset を計算し **XROrigin** に適用（CG コンテンツはシーン固定） |
| `PoseConstraints.RemoveRoll` | ModelView 適用後、原点 forward を保ち **ワールド roll** を除去 |
| `PoseConstraints.ComputeTrackingOriginPose` | `CalibrationOptions`（デフォルト `RemoveRoll`）を pivot 付きで適用 |

#### 典型的なシーン構成（XR Interaction Toolkit の `XRRig` + CG）

車両 / ステアリングホイールは **シーン固定**のまま、キャリブ後に動かすのは **XR Origin のみ**:

```
Scene
├── XR Origin                 ← XROrigin コンポーネント（Samples / XRI Starter Assets）
│   └── Camera Offset
│       └── Main Camera
└── VehicleRoot               ← CG コンテンツのルート（シーン固定）
    └── SteeringWheel         ← 既知 CG ハンドルの Transform
```

Inspector で割り当て:

- `xrOrigin` → `XROrigin` の付いたルート（prefab 内の `XR Origin`）
- `vehicleRoot` → ステアリングモデルの親
- `cgHandle` → ステアリングホイール Transform（`Calibrate` の `CgHandlePose` と水平制約 pivot の両方に使用）

#### コピペ用サンプル

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

`RigCalibrationOffset.ApplyCalibrationToTrackingOrigin` の内部処理:

1. `ModelView` と `vehicleRoot` / `cgHandle` から `rigOffset` を計算
2. `proposedOrigin = rigOffset * 現在の原点`
3. `PoseConstraints.ComputeTrackingOriginPose(..., pivot = cgHandle.position, RemoveRoll)`

ステップ 2–3 を分けて確認・カスタマイズする場合:

```csharp
Matrix4x4 modelView = result.ModelView;
var origin = xrOrigin.transform;

RigCalibrationOffset.TryComputeProposedOriginPose(
    modelView,
    origin,
    vehicleRoot,
    cgHandle,
    out var proposedPosition,
    out var proposedRotation);

PoseConstraints.ComputeTrackingOriginPose(
    proposedPosition,
    proposedRotation,
    cgHandle.position,
    CalibrationOptions.Default,
    out var originPosition,
    out var originRotation);

origin.SetPositionAndRotation(originPosition, originRotation);
```

`TrackingHorizonConstraint`: `None`, `RemoveRoll`（HMD 向け推奨）, `RemovePitch`, `RemovePitchAndRoll`。pivot 補正は `RemoveHandleLocalRoll` と同型で、回転変更後も原点から pivot への局所オフセットが保たれます。

## サンプル

パッケージ導入後、**Package Manager** で **XR Steering Controller Anchor Calibration** を選び、Samples から **Calibration Sample** を Import してください。

本リポジトリでは `Assets/CalibrationSample/CalibrationSample.unity` でも試せます。

サンプル内容:

- `SteeringAnchorCalibrationSample`（**CgHandle** / **Head** / 弧中心の参照）
- Inspector: **Generate Sample Points** → **Run Calibration** → **Clear**
- Scene View ギズモ（ハンドル軸、頭、userForward 矢印、フィット円、マップ結果）
- キャリブ後: ModelView 写像位置に赤い **Head (Mapped)** を表示（Editor）
- Inspector: 前後スコア、ハンドル局所 roll、運転席側 / ボンネット側判定

## パッケージ構成

```
Packages/com.neon-izm.xr-steering-controller-anchor-calibration/
├── package.json
├── Runtime/          # コアライブラリ（AnchorCalibration, PoseConstraints, RigCalibrationOffset 等）
├── Tests/Editor/     # Edit Mode テスト
└── Samples~/         # 任意インポートのサンプルシーン
```

## テスト

`Tests/Editor` の Edit Mode テストを **Window > General > Test Runner** から実行できます。

他プロジェクトでパッケージ内テストを走らせる場合は、`Packages/manifest.json` の `testables` にパッケージ名を追加してください:

```json
{
  "testables": [
    "com.neon-izm.xr-steering-controller-anchor-calibration"
  ]
}
```

### ローカル開発（本リポジトリ）

開発用 Unity プロジェクトでは embedded パッケージとして参照しています:

```json
"com.neon-izm.xr-steering-controller-anchor-calibration": "file:com.neon-izm.xr-steering-controller-anchor-calibration"
```

## ライセンス

MIT License。詳細は [LICENSE](LICENSE) を参照してください。
