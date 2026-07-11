using System;
using System.Collections.Generic;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    public static class AnchorCalibration
    {
        const float MinVectorSqrMagnitude = 1e-8f;

        /// <summary>
        /// HMD 世界座標の軌跡と頭姿勢から円を推定し、既知の CG ハンドル姿勢へ合わせる ModelView を求める。
        /// 前後は頭位置で選択し、ハンドル局所 roll を除去する。
        /// </summary>
        public static CalibrationResult Calibrate(
            IReadOnlyList<Vector3> worldPoints,
            CgHandlePose targetHandle,
            HeadPose headPose,
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

            var modelView = ComputeCalibratedModelView(fit.Circle, targetHandle, headPose);
            return new CalibrationResult(modelView);
        }

        /// <summary>
        /// 頭 → 手（軌跡代表点）の前向きベクトルを推定する。
        /// </summary>
        public static Vector3 EstimateUserForward(Vector3 headPosition, IReadOnlyList<Vector3> handPoints)
        {
            if (handPoints == null || handPoints.Count == 0)
            {
                throw new ArgumentException("At least one hand point is required.", nameof(handPoints));
            }

            var centroid = Vector3.zero;
            for (var i = 0; i < handPoints.Count; i++)
            {
                centroid += handPoints[i];
            }

            centroid /= handPoints.Count;
            var forward = centroid - headPosition;
            if (forward.sqrMagnitude < MinVectorSqrMagnitude)
            {
                throw new InvalidOperationException("Head position coincides with hand centroid.");
            }

            return forward.normalized;
        }

        /// <summary>
        /// 頭姿勢ヒントでフィット円の平面内位相を決定する。
        /// </summary>
        public static Circle3D OrientCircleWithHeadHint(Circle3D rawFit, HeadPose headPose)
        {
            var normal = rawFit.Normal.normalized;
            var headUp = headPose.Rotation * Vector3.up;
            var upOnPlane = Vector3.ProjectOnPlane(headUp, normal);

            if (upOnPlane.sqrMagnitude < MinVectorSqrMagnitude)
            {
                var headForward = headPose.Rotation * Vector3.forward;
                upOnPlane = Vector3.ProjectOnPlane(headForward, normal);
            }

            if (upOnPlane.sqrMagnitude < MinVectorSqrMagnitude)
            {
                upOnPlane = Vector3.ProjectOnPlane(Vector3.up, normal);
            }

            upOnPlane.Normalize();
            return new Circle3D(rawFit.Center, Quaternion.LookRotation(normal, upOnPlane), rawFit.Radius);
        }

        /// <summary>
        /// 円面内で法線を反転した候補（前後 2 解のうちの一方）を返す。
        /// </summary>
        public static Circle3D FlipCircleNormalInPlane(Circle3D circle)
        {
            var flippedRotation = circle.Rotation * Quaternion.AngleAxis(180f, Vector3.up);
            return new Circle3D(circle.Center, flippedRotation, circle.Radius);
        }

        /// <summary>
        /// 世界座標で推定した円を、既知の CG ハンドル姿勢へ一致させる剛体 ModelView (scale=1) を求める。
        /// </summary>
        public static Matrix4x4 ComputeModelView(Circle3D estimatedWorld, CgHandlePose targetHandle)
        {
            var rotation = targetHandle.Rotation * Quaternion.Inverse(estimatedWorld.Rotation);
            var translation = targetHandle.Position - rotation * estimatedWorld.Position;
            return Matrix4x4.TRS(translation, rotation, Vector3.one);
        }

        /// <summary>
        /// 頭位置スコア。大きいほど頭がハンドル後方（運転席側）にある。
        /// score = dot(normalize(handle - mappedHead), handleForward)
        /// </summary>
        public static float ScoreDriverSeat(Matrix4x4 modelView, HeadPose headPose, CgHandlePose targetHandle)
        {
            var mappedHead = WorldToCg(headPose.Position, modelView);
            var toHandle = targetHandle.Position - mappedHead;
            if (toHandle.sqrMagnitude < MinVectorSqrMagnitude)
            {
                return 0f;
            }

            return Vector3.Dot(toHandle.normalized, targetHandle.Forward);
        }

        /// <summary>
        /// 前後 2 候補の ModelView から、頭が運転席側になる方を選ぶ。
        /// </summary>
        public static Matrix4x4 SelectFrontBackCandidate(
            Matrix4x4 modelViewA,
            Matrix4x4 modelViewB,
            HeadPose headPose,
            CgHandlePose targetHandle)
        {
            var scoreA = ScoreDriverSeat(modelViewA, headPose, targetHandle);
            var scoreB = ScoreDriverSeat(modelViewB, headPose, targetHandle);
            return scoreA >= scoreB ? modelViewA : modelViewB;
        }

        /// <summary>
        /// 頭が運転席側（ハンドル Back）に写っているか。
        /// </summary>
        public static bool IsOnDriverSeatSide(Matrix4x4 modelView, HeadPose headPose, CgHandlePose targetHandle)
        {
            return ScoreDriverSeat(modelView, headPose, targetHandle) > 0f;
        }

        /// <summary>
        /// CG ハンドル軸まわりの余分な roll を除去する。
        /// </summary>
        public static Matrix4x4 RemoveHandleLocalRoll(
            Matrix4x4 modelView,
            Circle3D estimatedWorld,
            CgHandlePose targetHandle)
        {
            var rotation = modelView.rotation;
            var translation = (Vector3)modelView.GetColumn(3);
            var mappedRotation = rotation * estimatedWorld.Rotation;
            var twist = GetSignedTwistAngle(targetHandle.Rotation, mappedRotation, targetHandle.Forward);
            var correction = Quaternion.AngleAxis(-twist, targetHandle.Forward);

            var correctedRotation = correction * rotation;
            var correctedTranslation = correction * (translation - targetHandle.Position) + targetHandle.Position;
            return Matrix4x4.TRS(correctedTranslation, correctedRotation, Vector3.one);
        }

        /// <summary>
        /// ハンドル局所座標系での相対 roll 角（度）。
        /// </summary>
        public static float GetHandleLocalRollDeg(
            Matrix4x4 modelView,
            Circle3D estimatedWorld,
            CgHandlePose targetHandle)
        {
            var mappedRotation = modelView.rotation * estimatedWorld.Rotation;
            return GetSignedTwistAngle(targetHandle.Rotation, mappedRotation, targetHandle.Forward);
        }

        /// <summary>
        /// 円フィット → 頭ヒント位相 → 前後選択 → roll 除去までの一連処理。
        /// </summary>
        public static Matrix4x4 ComputeCalibratedModelView(
            Circle3D rawFit,
            CgHandlePose targetHandle,
            HeadPose headPose,
            bool forceFlippedCandidate = false)
        {
            return ComputeCalibratedModelView(rawFit, targetHandle, headPose, forceFlippedCandidate, out _);
        }

        /// <summary>
        /// 円フィット → 頭ヒント位相 → 前後選択 → roll 除去までの一連処理。
        /// orientedCircleUsed は採用候補の姿勢付き円。
        /// </summary>
        public static Matrix4x4 ComputeCalibratedModelView(
            Circle3D rawFit,
            CgHandlePose targetHandle,
            HeadPose headPose,
            bool forceFlippedCandidate,
            out Circle3D orientedCircleUsed)
        {
            var oriented = OrientCircleWithHeadHint(rawFit, headPose);
            var flipped = FlipCircleNormalInPlane(oriented);

            var modelViewA = RemoveHandleLocalRoll(ComputeModelView(oriented, targetHandle), oriented, targetHandle);
            var modelViewB = RemoveHandleLocalRoll(ComputeModelView(flipped, targetHandle), flipped, targetHandle);

            if (forceFlippedCandidate)
            {
                orientedCircleUsed = flipped;
                return modelViewB;
            }

            var scoreA = ScoreDriverSeat(modelViewA, headPose, targetHandle);
            if (scoreA >= ScoreDriverSeat(modelViewB, headPose, targetHandle))
            {
                orientedCircleUsed = oriented;
                return modelViewA;
            }

            orientedCircleUsed = flipped;
            return modelViewB;
        }

        [Obsolete("前後解決には使わない。AlignNormalToTargetForward は半球合わせのみで運転席側を保証しない。")]
        public static Circle3D AlignNormalToTargetForward(Circle3D circle, Vector3 targetForward)
        {
            var reference = targetForward.normalized;
            if (reference.sqrMagnitude < MinVectorSqrMagnitude)
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
            var scale = targetCg.Radius / estimatedWorld.Radius;
            var rotation = targetCg.Rotation * Quaternion.Inverse(estimatedWorld.Rotation);
            var translation = targetCg.Position - rotation * (estimatedWorld.Position * scale);
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

        static float GetSignedTwistAngle(Quaternion from, Quaternion to, Vector3 axis)
        {
            var delta = Quaternion.Inverse(from) * to;
            delta.ToAngleAxis(out var angle, out var deltaAxis);
            if (angle > 180f)
            {
                angle -= 360f;
            }

            if (deltaAxis.sqrMagnitude < MinVectorSqrMagnitude)
            {
                return 0f;
            }

            if (Vector3.Dot(deltaAxis.normalized, axis.normalized) < 0f)
            {
                angle = -angle;
            }

            return angle;
        }
    }
}
