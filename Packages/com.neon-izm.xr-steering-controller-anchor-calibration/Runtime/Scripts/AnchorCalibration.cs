using System;
using System.Collections.Generic;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    public static class AnchorCalibration
    {
        /// <summary>
        /// HMD 世界座標の軌跡から円を推定し、既知の CG ハンドル姿勢へ合わせる ModelView を求める。
        /// CG ハンドルの半径は未知。剛体変換 (scale=1) のみ返す。
        /// </summary>
        public static CalibrationResult Calibrate(
            IReadOnlyList<Vector3> worldPoints,
            CgHandlePose targetHandle,
            float threshold = CircleFitting3D.DefaultThreshold,
            int maxIterations = 0,
            float successProbability = 0.999f,
            System.Random random = null)
        {
            if (worldPoints == null)
            {
                throw new ArgumentNullException(nameof(worldPoints));
            }

            var fit = CircleFitting3D.FitCircleMsac(
                worldPoints,
                threshold,
                maxIterations,
                successProbability,
                random);

            var modelView = ComputeModelView(fit.Circle, targetHandle);
            return new CalibrationResult(modelView);
        }

        /// <summary>
        /// 世界座標で推定した円を、既知の CG ハンドル姿勢へ一致させる剛体 ModelView (scale=1) を求める。
        /// 法線の向きは CG ターゲットの forward と同じ半球へ揃える。
        /// </summary>
        public static Matrix4x4 ComputeModelView(Circle3D estimatedWorld, CgHandlePose targetHandle)
        {
            var aligned = AlignNormalToTargetForward(estimatedWorld, targetHandle.Forward);
            var rotation = targetHandle.Rotation * Quaternion.Inverse(aligned.Rotation);
            var translation = targetHandle.Position - rotation * aligned.Position;
            return Matrix4x4.TRS(translation, rotation, Vector3.one);
        }

        /// <summary>
        /// フィットした円の法線を、CG ターゲット forward と同じ半球へ揃える。
        /// 完全水平などで dot≈0 の場合はそのまま返す。
        /// </summary>
        public static Circle3D AlignNormalToTargetForward(Circle3D circle, Vector3 targetForward)
        {
            var reference = targetForward.normalized;
            if (reference.sqrMagnitude < 1e-8f)
            {
                return circle;
            }

            if (Vector3.Dot(circle.Normal, reference) < 0f)
            {
                return new Circle3D(
                    circle.Center,
                    CircleFitting3D.RotationFromNormal(-circle.Normal),
                    circle.Radius);
            }

            return circle;
        }

        /// <summary>
        /// キャリブレーション済み ModelView で世界座標点を CG 空間へ変換する。
        /// </summary>
        public static Vector3 WorldToCg(Vector3 worldPoint, Matrix4x4 modelView)
        {
            return modelView.MultiplyPoint3x4(worldPoint);
        }

        /// <summary>
        /// キャリブレーション済み ModelView で世界座標点群を CG 空間へ変換する。
        /// </summary>
        public static Vector3[] TransformWorldPointsToCg(IReadOnlyList<Vector3> worldPoints, Matrix4x4 modelView)
        {
            if (worldPoints == null)
            {
                throw new ArgumentNullException(nameof(worldPoints));
            }

            var cgPoints = new Vector3[worldPoints.Count];
            for (var i = 0; i < worldPoints.Count; i++)
            {
                cgPoints[i] = WorldToCg(worldPoints[i], modelView);
            }

            return cgPoints;
        }

        /// <summary>
        /// 半径スケールも含む変換。物理半径と CG 半径の両方が既知の場合に使用。
        /// </summary>
        public static Matrix4x4 ComputeModelViewWithScale(Circle3D estimatedWorld, Circle3D targetCg)
        {
            var aligned = AlignNormalToTargetForward(estimatedWorld, targetCg.Normal);
            var scale = targetCg.Radius / aligned.Radius;
            var rotation = targetCg.Rotation * Quaternion.Inverse(aligned.Rotation);
            var translation = targetCg.Position - rotation * (aligned.Position * scale);
            return Matrix4x4.TRS(translation, rotation, Vector3.one * scale);
        }

        public static Circle3D TransformCircleRigid(Matrix4x4 modelView, Circle3D circleWorld)
        {
            var center = modelView.MultiplyPoint3x4(circleWorld.Position);
            var forward = modelView.MultiplyVector(circleWorld.Rotation * Vector3.forward);
            var up = modelView.MultiplyVector(circleWorld.Rotation * Vector3.up);
            var rotation = Quaternion.LookRotation(forward, up);
            return new Circle3D(center, rotation, circleWorld.Radius);
        }

        public static Circle3D TransformCircle(Matrix4x4 transform, Circle3D circle)
        {
            var scale = transform.lossyScale.x;
            var center = transform.MultiplyPoint3x4(circle.Position);
            var rotation = transform.rotation * circle.Rotation;
            return new Circle3D(center, rotation, circle.Radius * scale);
        }
    }
}
