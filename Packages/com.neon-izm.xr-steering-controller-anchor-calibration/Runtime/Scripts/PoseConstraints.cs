using System;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    /// <summary>
    /// トラッキング原点（XR Origin）へ ModelView を適用する際の姿勢制約。
    /// <see cref="AnchorCalibration.RemoveHandleLocalRoll"/>（CG ハンドル軸 twist 除去）とは独立。
    /// </summary>
    public static class PoseConstraints
    {
        const float MinVectorSqrMagnitude = 1e-8f;

        /// <summary>
        /// forward を維持し、<paramref name="referenceUp"/> 基準で roll のみ除去する。
        /// </summary>
        public static Quaternion RemoveRoll(Quaternion rotation, Vector3 referenceUp)
        {
            var forward = rotation * Vector3.forward;
            if (forward.sqrMagnitude < MinVectorSqrMagnitude)
            {
                return rotation;
            }

            var up = referenceUp.sqrMagnitude < MinVectorSqrMagnitude ? Vector3.up : referenceUp.normalized;
            return Quaternion.LookRotation(forward.normalized, up);
        }

        /// <summary>
        /// ピッチを水平化し、元の roll を維持する（flat forward + rolled up）。
        /// </summary>
        public static Quaternion RemovePitch(Quaternion rotation, Vector3 referenceUp)
        {
            var up = referenceUp.sqrMagnitude < MinVectorSqrMagnitude ? Vector3.up : referenceUp.normalized;
            var forward = rotation * Vector3.forward;
            var flatForward = Vector3.ProjectOnPlane(forward, up);
            if (flatForward.sqrMagnitude < MinVectorSqrMagnitude)
            {
                var right = rotation * Vector3.right;
                flatForward = Vector3.Cross(up, Vector3.ProjectOnPlane(right, up));
                if (flatForward.sqrMagnitude < MinVectorSqrMagnitude)
                {
                    return rotation;
                }
            }

            flatForward.Normalize();

            var rolledUp = Vector3.ProjectOnPlane(rotation * Vector3.up, flatForward);
            if (rolledUp.sqrMagnitude < MinVectorSqrMagnitude)
            {
                return Quaternion.LookRotation(flatForward, up);
            }

            return Quaternion.LookRotation(flatForward, rolledUp.normalized);
        }

        /// <summary>
        /// ヨーのみ残す（ピッチ・ロール除去）。
        /// </summary>
        public static Quaternion RemovePitchAndRoll(Quaternion rotation, Vector3 referenceUp)
        {
            var up = referenceUp.sqrMagnitude < MinVectorSqrMagnitude ? Vector3.up : referenceUp.normalized;
            var forward = rotation * Vector3.forward;
            var flatForward = Vector3.ProjectOnPlane(forward, up);
            if (flatForward.sqrMagnitude < MinVectorSqrMagnitude)
            {
                var right = rotation * Vector3.right;
                flatForward = Vector3.Cross(up, Vector3.ProjectOnPlane(right, up));
                if (flatForward.sqrMagnitude < MinVectorSqrMagnitude)
                {
                    return Quaternion.LookRotation(Vector3.forward, up);
                }
            }

            return Quaternion.LookRotation(flatForward.normalized, up);
        }

        /// <summary>
        /// 回転変更後も <paramref name="worldPoint"/> がワールドで不変になる位置を再計算する。
        /// </summary>
        public static Vector3 ComputePositionPreservingWorldPoint(
            Vector3 position,
            Quaternion rotation,
            Quaternion constrainedRotation,
            Vector3 worldPoint)
        {
            var localOffset = Quaternion.Inverse(rotation) * (worldPoint - position);
            return worldPoint - constrainedRotation * localOffset;
        }

        /// <summary>
        /// 姿勢制約を適用し、pivot のワールド位置を保存する。
        /// </summary>
        public static void ConstrainPosePreservingWorldPoint(
            Vector3 position,
            Quaternion rotation,
            TrackingHorizonConstraint constraint,
            Vector3 referenceUp,
            Vector3 worldPivot,
            out Vector3 constrainedPosition,
            out Quaternion constrainedRotation)
        {
            constrainedRotation = ApplyConstraint(rotation, constraint, referenceUp);
            constrainedPosition = ComputePositionPreservingWorldPoint(
                position,
                rotation,
                constrainedRotation,
                worldPivot);
        }

        /// <summary>
        /// 候補のトラッキング原点姿勢に水平制約を適用する（HMD / XR Origin 適用の推奨ヘルパー）。
        /// </summary>
        public static void ComputeTrackingOriginPose(
            Vector3 proposedOriginPosition,
            Quaternion proposedOriginRotation,
            Vector3 pivotWorldPosition,
            CalibrationOptions options,
            out Vector3 originPosition,
            out Quaternion originRotation)
        {
            ConstrainPosePreservingWorldPoint(
                proposedOriginPosition,
                proposedOriginRotation,
                options.HorizonConstraint,
                options.ReferenceUp,
                pivotWorldPosition,
                out originPosition,
                out originRotation);
        }

        static Quaternion ApplyConstraint(
            Quaternion rotation,
            TrackingHorizonConstraint constraint,
            Vector3 referenceUp)
        {
            return constraint switch
            {
                TrackingHorizonConstraint.None => rotation,
                TrackingHorizonConstraint.RemoveRoll => RemoveRoll(rotation, referenceUp),
                TrackingHorizonConstraint.RemovePitch => RemovePitch(rotation, referenceUp),
                TrackingHorizonConstraint.RemovePitchAndRoll => RemovePitchAndRoll(rotation, referenceUp),
                _ => throw new ArgumentOutOfRangeException(nameof(constraint), constraint, null),
            };
        }
    }
}
