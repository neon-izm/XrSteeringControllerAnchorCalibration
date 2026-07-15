using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public class RigCalibrationOffsetTests
    {
        const float Tolerance = 1e-3f;

        [Test]
        public void ApplyContentRootAlignment_MovesContent_LeavesOrigin_HandleMatchesEstimated()
        {
            var contentRoot = new GameObject("ContentRoot").transform;
            var cgHandle = new GameObject("CgHandle").transform;
            var trackingOrigin = new GameObject("TrackingOrigin").transform;
            cgHandle.SetParent(contentRoot, false);
            cgHandle.localPosition = new Vector3(0.1f, 0.85f, 0.4f);
            cgHandle.localRotation = Quaternion.Euler(35f, 10f, 20f);
            contentRoot.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            trackingOrigin.SetPositionAndRotation(new Vector3(0f, 1.4f, 0f), Quaternion.identity);

            var targetHandle = CgHandlePose.FromTransform(cgHandle);
            var estimatedWorldPos = new Vector3(0.2f, 1.1f, 0.55f);
            var estimatedRot = Quaternion.LookRotation(
                (Quaternion.Euler(35f, 10f, 0f) * Vector3.forward).normalized,
                Vector3.up);
            var modelView = BuildModelViewMatchingHandle(targetHandle, estimatedWorldPos, estimatedRot);

            try
            {
                var originBefore = trackingOrigin.position;
                var originRotBefore = trackingOrigin.rotation;

                Assert.IsTrue(RigCalibrationOffset.ApplyContentRootAlignment(
                    modelView,
                    contentRoot,
                    cgHandle,
                    out var result));

                Assert.That(Vector3.Distance(trackingOrigin.position, originBefore), Is.LessThan(Tolerance));
                Assert.That(Quaternion.Angle(trackingOrigin.rotation, originRotBefore), Is.LessThan(0.1f));
                Assert.That(Vector3.Distance(cgHandle.position, estimatedWorldPos), Is.LessThan(0.02f));
                Assert.That(Mathf.Abs(result.HubTwistAppliedDeg), Is.GreaterThanOrEqualTo(0f));

                var axis = cgHandle.forward.normalized;
                var handleUpOnPlane = Vector3.ProjectOnPlane(cgHandle.up, axis).normalized;
                var worldUpOnPlane = Vector3.ProjectOnPlane(Vector3.up, axis).normalized;
                Assert.That(Vector3.Angle(handleUpOnPlane, worldUpOnPlane), Is.LessThan(1f));
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
