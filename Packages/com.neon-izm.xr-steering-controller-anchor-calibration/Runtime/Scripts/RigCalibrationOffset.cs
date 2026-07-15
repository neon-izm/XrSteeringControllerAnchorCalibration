using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    /// <summary>
    /// Result of <see cref="RigCalibrationOffset.ApplyContentRootAlignment"/>.
    /// </summary>
    public readonly struct ContentRootAlignmentResult
    {
        /// <summary>
        /// Signed degrees applied about the handle hub (forward) so handle.up matches
        /// world up projected onto the wheel plane. Zero when already aligned
        /// or the projection is degenerate (e.g. nearly flat wheel).
        /// </summary>
        public readonly float HubTwistAppliedDeg;

        public ContentRootAlignmentResult(float hubTwistAppliedDeg)
        {
            HubTwistAppliedDeg = hubTwistAppliedDeg;
        }
    }

    /// <summary>
    /// ModelView を CG コンテンツへ適用する。
    /// XR Origin は動かさず、コンテンツルートを整合したうえで handle.up をワールド up に揃える。
    /// </summary>
    public static class RigCalibrationOffset
    {
        /// <summary>
        /// Tracking origin は動かさず、コンテンツルートを ModelView 世界姿勢へ合わせたあと、
        /// ハンドル hub（forward）まわりにツイストして handle.up をワールド up 平面投影に揃える。
        /// </summary>
        public static bool ApplyContentRootAlignment(
            Matrix4x4 modelView,
            Transform contentRoot,
            Transform cgHandle,
            out ContentRootAlignmentResult result)
        {
            result = default;
            if (contentRoot == null || cgHandle == null)
            {
                return false;
            }

            if (!TryComputeContentRootAlignment(
                    modelView,
                    cgHandle.position,
                    cgHandle.rotation,
                    cgHandle.localPosition,
                    cgHandle.localRotation,
                    out var alignedRootPos,
                    out var alignedRootRot))
            {
                return false;
            }

            contentRoot.SetPositionAndRotation(alignedRootPos, alignedRootRot);
            var hubTwistDeg = TwistContentAboutHandleToMatchWorldUp(contentRoot, cgHandle);
            result = new ContentRootAlignmentResult(hubTwistDeg);
            return true;
        }

        static bool TryComputeContentRootAlignment(
            Matrix4x4 modelView,
            Vector3 cgHandleWorldPosition,
            Quaternion cgHandleWorldRotation,
            Vector3 cgHandleLocalPosition,
            Quaternion cgHandleLocalRotation,
            out Vector3 alignedRootPosition,
            out Quaternion alignedRootRotation)
        {
            var cgToWorld = modelView.inverse;
            var alignedHandlePosition = cgToWorld.MultiplyPoint3x4(cgHandleWorldPosition);
            var alignedHandleRotation = cgToWorld.rotation * cgHandleWorldRotation;
            alignedRootRotation = alignedHandleRotation * Quaternion.Inverse(cgHandleLocalRotation);
            alignedRootPosition = alignedHandlePosition - alignedRootRotation * cgHandleLocalPosition;
            return true;
        }

        static float TwistContentAboutHandleToMatchWorldUp(Transform contentRoot, Transform cgHandle)
        {
            var twistDeg = ComputeHubTwistDeg(cgHandle.forward, cgHandle.up);
            if (Mathf.Abs(twistDeg) < 1e-3f)
            {
                return 0f;
            }

            contentRoot.RotateAround(cgHandle.position, cgHandle.forward.normalized, twistDeg);
            return twistDeg;
        }

        static float ComputeHubTwistDeg(Vector3 handleForward, Vector3 handleUp)
        {
            if (handleForward.sqrMagnitude < 1e-8f)
            {
                return 0f;
            }

            var axis = handleForward.normalized;
            var currentUp = Vector3.ProjectOnPlane(handleUp, axis);
            var desiredUp = Vector3.ProjectOnPlane(Vector3.up, axis);
            if (currentUp.sqrMagnitude < 1e-6f || desiredUp.sqrMagnitude < 1e-6f)
            {
                return 0f;
            }

            return Vector3.SignedAngle(currentUp.normalized, desiredUp.normalized, axis);
        }
    }
}
