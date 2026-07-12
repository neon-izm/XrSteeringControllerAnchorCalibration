using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    /// <summary>
    /// ModelView を XR Interaction Toolkit の XROrigin（トラッキング原点）へ適用するための rig offset 計算。
    /// CG コンテンツ（車両ルート）はシーン固定のまま、原点だけ動かしてハンドル位置を整合させる。
    /// </summary>
    public static class RigCalibrationOffset
    {
        /// <summary>
        /// ModelView 整合後にコンテンツルートが置かれるべきワールド姿勢を求める。
        /// </summary>
        public static bool TryComputeContentRootAlignment(
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

        /// <summary>
        /// 現在のコンテンツルート姿勢から、トラッキング原点に掛ける rig offset を求める。
        /// </summary>
        public static bool TryComputeRigOffset(
            Matrix4x4 modelView,
            Vector3 contentRootPosition,
            Quaternion contentRootRotation,
            Vector3 cgHandleWorldPosition,
            Quaternion cgHandleWorldRotation,
            Vector3 cgHandleLocalPosition,
            Quaternion cgHandleLocalRotation,
            out Vector3 rigOffsetPosition,
            out Quaternion rigOffsetRotation)
        {
            if (!TryComputeContentRootAlignment(
                    modelView,
                    cgHandleWorldPosition,
                    cgHandleWorldRotation,
                    cgHandleLocalPosition,
                    cgHandleLocalRotation,
                    out var alignedRootPosition,
                    out var alignedRootRotation))
            {
                rigOffsetPosition = default;
                rigOffsetRotation = default;
                return false;
            }

            var contentDelta = Matrix4x4.TRS(alignedRootPosition, alignedRootRotation, Vector3.one)
                * Matrix4x4.TRS(contentRootPosition, contentRootRotation, Vector3.one).inverse;
            var rigOffset = contentDelta.inverse;
            rigOffsetPosition = rigOffset.GetColumn(3);
            rigOffsetRotation = rigOffset.rotation;
            return true;
        }

        public static bool TryComputeRigOffset(
            Matrix4x4 modelView,
            Transform contentRoot,
            Transform cgHandle,
            out Vector3 rigOffsetPosition,
            out Quaternion rigOffsetRotation)
        {
            return TryComputeRigOffset(
                modelView,
                contentRoot.position,
                contentRoot.rotation,
                cgHandle.position,
                cgHandle.rotation,
                cgHandle.localPosition,
                cgHandle.localRotation,
                out rigOffsetPosition,
                out rigOffsetRotation);
        }

        /// <summary>
        /// rig offset を現在のトラッキング原点に適用した候補姿勢（水平制約前）を求める。
        /// </summary>
        public static bool TryComputeProposedOriginPose(
            Matrix4x4 modelView,
            Transform trackingOrigin,
            Transform contentRoot,
            Transform cgHandle,
            out Vector3 proposedPosition,
            out Quaternion proposedRotation)
        {
            if (!TryComputeRigOffset(modelView, contentRoot, cgHandle, out var rigOffsetPosition, out var rigOffsetRotation))
            {
                proposedPosition = default;
                proposedRotation = default;
                return false;
            }

            var originMatrix = Matrix4x4.TRS(trackingOrigin.position, trackingOrigin.rotation, Vector3.one);
            var rigOffsetMatrix = Matrix4x4.TRS(rigOffsetPosition, rigOffsetRotation, Vector3.one);
            var proposedMatrix = rigOffsetMatrix * originMatrix;
            proposedPosition = proposedMatrix.GetColumn(3);
            proposedRotation = proposedMatrix.rotation;
            return true;
        }

        /// <summary>
        /// ModelView をトラッキング原点へ適用し、水平制約（デフォルト RemoveRoll）まで行う。
        /// </summary>
        public static bool ApplyCalibrationToTrackingOrigin(
            Matrix4x4 modelView,
            Transform trackingOrigin,
            Transform contentRoot,
            Transform cgHandle,
            CalibrationOptions options)
        {
            if (!TryComputeProposedOriginPose(
                    modelView,
                    trackingOrigin,
                    contentRoot,
                    cgHandle,
                    out var proposedPosition,
                    out var proposedRotation))
            {
                return false;
            }

            PoseConstraints.ComputeTrackingOriginPose(
                proposedPosition,
                proposedRotation,
                cgHandle.position,
                options,
                out var originPosition,
                out var originRotation);

            trackingOrigin.SetPositionAndRotation(originPosition, originRotation);
            return true;
        }
    }
}
