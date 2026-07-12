using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public class RigCalibrationOffsetTests
    {
        const float Tolerance = 1e-3f;

        [Test]
        public void ApplyCalibrationToTrackingOrigin_MovesOrigin_PreservesCgHandleWorldPosition()
        {
            var contentRoot = new GameObject("ContentRoot").transform;
            var cgHandle = new GameObject("CgHandle").transform;
            var trackingOrigin = new GameObject("TrackingOrigin").transform;
            cgHandle.SetParent(contentRoot, false);
            cgHandle.localPosition = new Vector3(0.2f, 0.9f, 0.45f);
            cgHandle.localRotation = Quaternion.Euler(10f, 25f, 0f);
            contentRoot.SetPositionAndRotation(new Vector3(1f, 0f, 2f), Quaternion.Euler(0f, 30f, 0f));
            trackingOrigin.SetPositionAndRotation(new Vector3(0f, 1.6f, 0f), Quaternion.Euler(5f, -15f, 2f));

            var targetHandle = CgHandlePose.FromTransform(cgHandle);
            var modelView = BuildModelViewMatchingHandle(
                targetHandle,
                new Vector3(0.35f, 1.05f, 0.55f),
                Quaternion.Euler(8f, -40f, 3f));

            try
            {
                var originBefore = trackingOrigin.position;
                var handleWorldBefore = cgHandle.position;
                Assert.IsTrue(RigCalibrationOffset.ApplyCalibrationToTrackingOrigin(
                    modelView,
                    trackingOrigin,
                    contentRoot,
                    cgHandle,
                    CalibrationOptions.Default));

                Assert.That(Vector3.Distance(trackingOrigin.position, originBefore), Is.GreaterThan(1e-3f));
                Assert.That(Vector3.Distance(cgHandle.position, handleWorldBefore), Is.LessThan(Tolerance));
            }
            finally
            {
                Object.DestroyImmediate(contentRoot.gameObject);
                Object.DestroyImmediate(trackingOrigin.gameObject);
            }
        }

        [Test]
        public void TryComputeProposedOriginPose_PreservesCgHandleWorldPosition_WhenConstraintIsNone()
        {
            var contentRoot = new GameObject("ContentRoot").transform;
            var cgHandle = new GameObject("CgHandle").transform;
            var trackingOrigin = new GameObject("TrackingOrigin").transform;
            cgHandle.SetParent(contentRoot, false);
            cgHandle.localPosition = new Vector3(0f, 0.8f, 0.4f);
            contentRoot.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            trackingOrigin.SetPositionAndRotation(new Vector3(0f, 1.5f, 0f), Quaternion.Euler(0f, 20f, 0f));

            var targetHandle = CgHandlePose.FromTransform(cgHandle);
            var modelView = BuildModelViewMatchingHandle(
                targetHandle,
                new Vector3(0.1f, 1.2f, 0.5f),
                Quaternion.Euler(0f, -10f, 0f));

            try
            {
                var handleWorldBefore = cgHandle.position;
                Assert.IsTrue(RigCalibrationOffset.TryComputeProposedOriginPose(
                    modelView,
                    trackingOrigin,
                    contentRoot,
                    cgHandle,
                    out var proposedPosition,
                    out var proposedRotation));

                trackingOrigin.SetPositionAndRotation(proposedPosition, proposedRotation);
                Assert.That(Vector3.Distance(cgHandle.position, handleWorldBefore), Is.LessThan(Tolerance));
            }
            finally
            {
                Object.DestroyImmediate(contentRoot.gameObject);
                Object.DestroyImmediate(trackingOrigin.gameObject);
            }
        }

        static Matrix4x4 BuildModelViewMatchingHandle(
            CgHandlePose targetHandle,
            Vector3 trackedHandleWorld,
            Quaternion trackedHandleRotation)
        {
            var estimatedWorld = Circle3D.FromPose(trackedHandleWorld, trackedHandleRotation, 0.2f);
            return AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);
        }
    }
}
