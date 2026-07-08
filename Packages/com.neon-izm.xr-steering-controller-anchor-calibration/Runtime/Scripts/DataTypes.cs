using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    /// <summary>
    /// 3D空間上の円。ハンドル型コントローラの形状近似に使用する。
    /// 姿勢は roll=0 制約付き（回転軸 = forward = 円面法線）。
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
            Rot = CircleFitting3D.EnforceZeroRoll(rot);
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
    /// </summary>
    public readonly struct CgHandlePose
    {
        public readonly Vector3 Position;
        public readonly Quaternion Rotation;

        public Vector3 Forward => Rotation * Vector3.forward;

        public CgHandlePose(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = CircleFitting3D.EnforceZeroRoll(rotation);
        }

        public static CgHandlePose FromTransform(Transform transform)
        {
            return new CgHandlePose(transform.position, transform.rotation);
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
}
