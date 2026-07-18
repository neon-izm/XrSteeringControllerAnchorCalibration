using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    /// <summary>
    /// Whether <see cref="RigCalibrationOffset.ApplyContentRootAlignment"/> corrects
    /// world pitch after roll hub-twist, or keeps the calibrated pitch.
    /// </summary>
    public enum ContentPitchMode
    {
        /// <summary>
        /// Keep pitch from calibration (CG handle pitch treated as truth).
        /// The car may appear nose-up / nose-down in world if the physical wheel was tilted.
        /// </summary>
        PreserveFromCalibration = 0,

        /// <summary>
        /// Rotate about handle.right so handle.forward is horizontal (zero world pitch).
        /// </summary>
        LevelToHorizon = 1,
    }

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

        /// <summary>
        /// Signed degrees applied about handle.right to zero world pitch of handle.forward.
        /// Zero when <see cref="ContentPitchMode.PreserveFromCalibration"/> or already level.
        /// </summary>
        public readonly float PitchCorrectionAppliedDeg;

        public ContentRootAlignmentResult(float hubTwistAppliedDeg, float pitchCorrectionAppliedDeg = 0f)
        {
            HubTwistAppliedDeg = hubTwistAppliedDeg;
            PitchCorrectionAppliedDeg = pitchCorrectionAppliedDeg;
        }
    }

    /// <summary>
    /// ModelView を CG コンテンツへ適用する。
    /// XR Origin は動かさず、コンテンツルートを整合したうえで handle.up をワールド up に揃える。
    /// 任意で handle.forward のワールド pitch を水平化する。
    /// </summary>
    public static class RigCalibrationOffset
    {
        const float MinAxisSqrMagnitude = 1e-8f;
        const float AngleEpsilonDeg = 1e-3f;

        /// <summary>
        /// Tracking origin は動かさず、コンテンツルートを ModelView 世界姿勢へ合わせたあと、
        /// ハンドル hub（forward）まわりにツイストして handle.up をワールド up 平面投影に揃える。
        /// Pitch はキャリブ結果のまま（<see cref="ContentPitchMode.PreserveFromCalibration"/>）。
        /// </summary>
        public static bool ApplyContentRootAlignment(
            Matrix4x4 modelView,
            Transform contentRoot,
            Transform cgHandle,
            out ContentRootAlignmentResult result)
        {
            return ApplyContentRootAlignment(
                modelView,
                contentRoot,
                cgHandle,
                ContentPitchMode.PreserveFromCalibration,
                out result);
        }

        /// <summary>
        /// Tracking origin は動かさず、コンテンツルートを ModelView 世界姿勢へ合わせたあと、
        /// roll（hub twist）と任意の pitch 水平化を適用する。
        /// </summary>
        public static bool ApplyContentRootAlignment(
            Matrix4x4 modelView,
            Transform contentRoot,
            Transform cgHandle,
            ContentPitchMode pitchMode,
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
            var pitchCorrectionDeg = 0f;
            if (pitchMode == ContentPitchMode.LevelToHorizon)
            {
                pitchCorrectionDeg = LevelHandleForwardToHorizon(contentRoot, cgHandle);
            }

            result = new ContentRootAlignmentResult(hubTwistDeg, pitchCorrectionDeg);
            return true;
        }

        /// <summary>
        /// World pitch of a handle forward vector in degrees.
        /// Positive when the nose points above the horizon (dot with <see cref="Vector3.up"/> &gt; 0).
        /// </summary>
        public static float GetHandleWorldPitchDeg(Vector3 handleForward)
        {
            if (handleForward.sqrMagnitude < MinAxisSqrMagnitude)
            {
                return 0f;
            }

            var s = Mathf.Clamp(Vector3.Dot(handleForward.normalized, Vector3.up), -1f, 1f);
            return Mathf.Asin(s) * Mathf.Rad2Deg;
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
            if (Mathf.Abs(twistDeg) < AngleEpsilonDeg)
            {
                return 0f;
            }

            contentRoot.RotateAround(cgHandle.position, cgHandle.forward.normalized, twistDeg);
            return twistDeg;
        }

        static float LevelHandleForwardToHorizon(Transform contentRoot, Transform cgHandle)
        {
            var right = cgHandle.right;
            if (right.sqrMagnitude < MinAxisSqrMagnitude)
            {
                return 0f;
            }

            var forward = cgHandle.forward;
            if (forward.sqrMagnitude < MinAxisSqrMagnitude)
            {
                return 0f;
            }

            forward.Normalize();
            right.Normalize();

            var horizontal = Vector3.ProjectOnPlane(forward, Vector3.up);
            if (horizontal.sqrMagnitude < 1e-6f)
            {
                return 0f;
            }

            horizontal.Normalize();
            var correctionDeg = Vector3.SignedAngle(forward, horizontal, right);
            if (Mathf.Abs(correctionDeg) < AngleEpsilonDeg)
            {
                return 0f;
            }

            contentRoot.RotateAround(cgHandle.position, right, correctionDeg);
            return correctionDeg;
        }

        static float ComputeHubTwistDeg(Vector3 handleForward, Vector3 handleUp)
        {
            if (handleForward.sqrMagnitude < MinAxisSqrMagnitude)
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
