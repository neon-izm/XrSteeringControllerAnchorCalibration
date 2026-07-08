# XR Steering Controller Anchor Calibration

HMD のハンドトラッキング軌跡（部分円弧）と、既知の CG ハンドル姿勢から、トラッキング空間 → CG 空間への剛体 **ModelView** 行列を求める Unity パッケージです。

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

タグやコミットを固定する場合は末尾に `#v0.1.0` や `#<commit-hash>` を付けます。


## 動作要件

- Unity 6（6000.3 以降推奨）
- Edit Mode テスト実行時は `com.unity.test-framework`

## 使い方

### API

```csharp
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

// 既知の CG ハンドル姿勢（位置・回転、roll=0）。半径は未知。
var targetHandle = new CgHandlePose(cgHandleTransform.position, cgHandleTransform.rotation);

// HMD 世界座標の軌跡点
IReadOnlyList<Vector3> worldPoints = trackedPoints;

// キャリブレーション結果は ModelView のみ（剛体・scale=1）
CalibrationResult result = AnchorCalibration.Calibrate(worldPoints, targetHandle);
Matrix4x4 modelView = result.ModelView;

// トラッキング点を CG 空間へ変換
Vector3 cgPoint = AnchorCalibration.WorldToCg(worldPoint, modelView);
```

### 前提

- ハンドル姿勢は **roll = 0**（forward = 円面法線）。
- CG ハンドルの **半径は未知**。キャリブレーション出力は剛体 ModelView のみ（スケールなし）。
- 法線の向き（forward / back）は CG ハンドル forward の半球で解決（上向きマウント想定）。完全水平軸は未対応。
- 外れ値耐性のため **MSAC** による 3D 円フィッティングを使用。

## サンプル

パッケージ導入後、**Package Manager** で **XR Steering Controller Anchor Calibration** を選び、Samples から **Calibration Sample** を Import してください。

サンプル内容:

- `SteeringAnchorCalibrationSample` コンポーネント
- Inspector: **Generate Sample Points** → **Run Calibration**
- Scene View ギズモ（CG ハンドル軸、フィット円、マップ済み inlier など）

## パッケージ構成

```
Packages/com.neon-izm.xr-steering-controller-anchor-calibration/
├── package.json
├── Runtime/          # コアライブラリ
├── Editor/           # サンプル用 Inspector 等
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
