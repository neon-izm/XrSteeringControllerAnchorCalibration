using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    /// <summary>
    /// 3D空間上の円。ハンドル型コントローラの形状近似に使用する。
    /// 姿勢の forward = 円面法線。
    /// </summary>
    public readonly struct Circle3D
    {
        public readonly Vector3 Center;
        public readonly Quaternion Rot;
        public readonly float Radius;

        public Vector3 Position => Center;
        public Quaternion Rotation => Rot;
        public Vector3 Normal => Rot * Vector3.forward;

        public Circle3D(Vector3 center, Quaternion rot, float radius)
        {
            Center = center;
            Rot = rot;
            Radius = radius;
        }

        public static Circle3D FromPose(Vector3 position, Quaternion rotation, float radius)
        {
            return new Circle3D(position, rotation, radius);
        }

        public static Circle3D FromPose(Vector3 position, Vector3 forward, float radius)
        {
            return new Circle3D(position, CircleFitting3D.RotationFromNormal(forward), radius);
        }
    }

    /// <summary>
    /// MSAC 円フィッティングの結果。
    /// </summary>
    public readonly struct CircleFitResult
    {
        public readonly Circle3D Circle;
        public readonly int[] InlierIndices;

        public CircleFitResult(Circle3D circle, int[] inlierIndices)
        {
            Circle = circle;
            InlierIndices = inlierIndices;
        }
    }

    /// <summary>
    /// CG ハンドルモデルの既知姿勢。半径は未知のため含めない。
    /// Forward = 車の進行方向（Front）。
    /// </summary>
    public readonly struct CgHandlePose
    {
        public readonly Vector3 Position;
        public readonly Quaternion Rotation;

        public Vector3 Forward => Rotation * Vector3.forward;

        public CgHandlePose(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = rotation;
        }

        public static CgHandlePose FromTransform(Transform transform)
        {
            return new CgHandlePose(transform.position, transform.rotation);
        }
    }

    /// <summary>
    /// HMD / 頭の世界座標姿勢。前後解決には Position のみ使用。
    /// Rotation はロール解決に使わない（VR-HMD では Vector3.up が真上）。
    /// </summary>
    public readonly struct HeadPose
    {
        public readonly Vector3 Position;
        public readonly Quaternion Rotation;

        public HeadPose(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = rotation;
        }

        public static HeadPose FromTransform(Transform transform)
        {
            return new HeadPose(transform.position, transform.rotation);
        }
    }

    /// <summary>
    /// キャリブレーション結果。世界座標 → CG 空間への ModelView のみを返す。
    /// </summary>
    public readonly struct CalibrationResult
    {
        /// <summary>
        /// 世界座標 → CG 空間への剛体 ModelView 行列 (scale=1)。
        /// cgPoint = ModelView.MultiplyPoint3x4(worldPoint)
        /// </summary>
        public readonly Matrix4x4 ModelView;

        public CalibrationResult(Matrix4x4 modelView)
        {
            ModelView = modelView;
        }
    }

    /// <summary>
    /// ModelView をトラッキング原点へ適用する際の水平・姿勢制約。
    /// <see cref="AnchorCalibration.RemoveHandleLocalRoll"/> とは独立。
    /// </summary>
    public enum TrackingHorizonConstraint
    {
        /// <summary>追加の水平制約なし。</summary>
        None = 0,

        /// <summary>
        /// 原点 forward を維持し、ワールド up 基準で roll のみ除去。HMD 適用の推奨。
        /// </summary>
        RemoveRoll = 1,

        /// <summary>ヨーを維持し、ピッチを水平化（roll は維持）。</summary>
        RemovePitch = 2,

        /// <summary>ヨーのみ残す（ピッチ・ロール除去）。</summary>
        RemovePitchAndRoll = 3,
    }

    /// <summary>
    /// トラッキング原点適用時のオプション。ModelView 計算自体には影響しない。
    /// </summary>
    public readonly struct CalibrationOptions
    {
        public TrackingHorizonConstraint HorizonConstraint { get; }
        public Vector3 ReferenceUp { get; }

        public CalibrationOptions(TrackingHorizonConstraint horizonConstraint, Vector3 referenceUp)
        {
            HorizonConstraint = horizonConstraint;
            ReferenceUp = referenceUp;
        }

        public static CalibrationOptions Default => new CalibrationOptions(
            TrackingHorizonConstraint.RemoveRoll,
            Vector3.up);

        public static CalibrationOptions None => new CalibrationOptions(
            TrackingHorizonConstraint.None,
            Vector3.up);
    }
}
