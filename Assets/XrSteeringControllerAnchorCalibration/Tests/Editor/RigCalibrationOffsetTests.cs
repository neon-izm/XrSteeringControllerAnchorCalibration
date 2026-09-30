using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public class RigCalibrationOffsetTests
    {
        const float Tolerance = 1e-3f;
        const float PitchToleranceDeg = 1e-2f;

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
                Assert.That(result.PitchCorrectionAppliedDeg, Is.EqualTo(0f).Within(PitchToleranceDeg));

                AssertRollAligned(cgHandle);
            }
            finally
            {
                Object.DestroyImmediate(contentRoot.gameObject);
                Object.DestroyImmediate(trackingOrigin.gameObject);
            }
        }

        [Test]
        public void ApplyContentRootAlignment_PreservePitch_KeepsWorldPitch()
        {
            using var scene = new HandleSceneFixture(cgLocalEuler: new Vector3(20f, 5f, 15f));
            var estimatedWorldPos = new Vector3(0.15f, 1.05f, 0.5f);
            // Forward pitched ~25° above horizon, then roll-aligned up.
            var pitchedForward = (Quaternion.Euler(-25f, 20f, 0f) * Vector3.forward).normalized;
            var estimatedRot = Quaternion.LookRotation(pitchedForward, Vector3.up);
            var modelView = BuildModelViewMatchingHandle(
                CgHandlePose.FromTransform(scene.CgHandle),
                estimatedWorldPos,
                estimatedRot);

            Assert.IsTrue(RigCalibrationOffset.ApplyContentRootAlignment(
                modelView,
                scene.ContentRoot,
                scene.CgHandle,
                ContentPitchMode.PreserveFromCalibration,
                out var result));

            Assert.That(result.PitchCorrectionAppliedDeg, Is.EqualTo(0f).Within(PitchToleranceDeg));
            Assert.That(
                Mathf.Abs(RigCalibrationOffset.GetHandleWorldPitchDeg(scene.CgHandle.forward)),
                Is.GreaterThan(10f));
            AssertRollAligned(scene.CgHandle);
        }

        [Test]
        public void ApplyContentRootAlignment_LevelToHorizon_ZerosWorldPitch()
        {
            using var scene = new HandleSceneFixture(cgLocalEuler: new Vector3(20f, 5f, 15f));
            var estimatedWorldPos = new Vector3(0.15f, 1.05f, 0.5f);
            var pitchedForward = (Quaternion.Euler(-25f, 20f, 0f) * Vector3.forward).normalized;
            var estimatedRot = Quaternion.LookRotation(pitchedForward, Vector3.up);
            var modelView = BuildModelViewMatchingHandle(
                CgHandlePose.FromTransform(scene.CgHandle),
                estimatedWorldPos,
                estimatedRot);

            Assert.IsTrue(RigCalibrationOffset.ApplyContentRootAlignment(
                modelView,
                scene.ContentRoot,
                scene.CgHandle,
                ContentPitchMode.LevelToHorizon,
                out var result));

            Assert.That(Mathf.Abs(result.PitchCorrectionAppliedDeg), Is.GreaterThan(10f));
            Assert.That(
                Mathf.Abs(RigCalibrationOffset.GetHandleWorldPitchDeg(scene.CgHandle.forward)),
                Is.LessThan(PitchToleranceDeg));
            AssertRollAligned(scene.CgHandle);
        }

        [Test]
        public void ApplyContentRootAlignment_DefaultOverload_MatchesPreserve()
        {
            using var sceneA = new HandleSceneFixture(cgLocalEuler: new Vector3(15f, 8f, 12f));
            using var sceneB = new HandleSceneFixture(cgLocalEuler: new Vector3(15f, 8f, 12f));
            var estimatedWorldPos = new Vector3(0.12f, 1.0f, 0.45f);
            var pitchedForward = (Quaternion.Euler(-18f, 12f, 0f) * Vector3.forward).normalized;
            var estimatedRot = Quaternion.LookRotation(pitchedForward, Vector3.up);
            var modelView = BuildModelViewMatchingHandle(
                CgHandlePose.FromTransform(sceneA.CgHandle),
                estimatedWorldPos,
                estimatedRot);

            Assert.IsTrue(RigCalibrationOffset.ApplyContentRootAlignment(
                modelView, sceneA.ContentRoot, sceneA.CgHandle, out var defaultResult));
            Assert.IsTrue(RigCalibrationOffset.ApplyContentRootAlignment(
                modelView,
                sceneB.ContentRoot,
                sceneB.CgHandle,
                ContentPitchMode.PreserveFromCalibration,
                out var preserveResult));

            Assert.That(defaultResult.PitchCorrectionAppliedDeg, Is.EqualTo(0f).Within(PitchToleranceDeg));
            Assert.That(
                defaultResult.PitchCorrectionAppliedDeg,
                Is.EqualTo(preserveResult.PitchCorrectionAppliedDeg).Within(PitchToleranceDeg));
            Assert.That(
                RigCalibrationOffset.GetHandleWorldPitchDeg(sceneA.CgHandle.forward),
                Is.EqualTo(RigCalibrationOffset.GetHandleWorldPitchDeg(sceneB.CgHandle.forward))
                    .Within(PitchToleranceDeg));
        }

        [Test]
        public void GetHandleWorldPitchDeg_MatchesAsinOfUpDot()
        {
            var forward = (Quaternion.Euler(-30f, 0f, 0f) * Vector3.forward).normalized;
            var expected = Mathf.Asin(Mathf.Clamp(Vector3.Dot(forward, Vector3.up), -1f, 1f)) * Mathf.Rad2Deg;
            Assert.That(RigCalibrationOffset.GetHandleWorldPitchDeg(forward), Is.EqualTo(expected).Within(1e-4f));
        }

        static void AssertRollAligned(Transform cgHandle)
        {
            var axis = cgHandle.forward.normalized;
            var handleUpOnPlane = Vector3.ProjectOnPlane(cgHandle.up, axis).normalized;
            var worldUpOnPlane = Vector3.ProjectOnPlane(Vector3.up, axis).normalized;
            Assert.That(Vector3.Angle(handleUpOnPlane, worldUpOnPlane), Is.LessThan(1f));
        }

        static Matrix4x4 BuildModelViewMatchingHandle(
            CgHandlePose targetHandle,
            Vector3 trackedHandleWorld,
            Quaternion trackedHandleRotation)
        {
            var estimatedWorld = Circle3D.FromPose(trackedHandleWorld, trackedHandleRotation, 0.2f);
            return AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);
        }

        sealed class HandleSceneFixture : System.IDisposable
        {
            public readonly Transform ContentRoot;
            public readonly Transform CgHandle;

            public HandleSceneFixture(Vector3 cgLocalEuler)
            {
                ContentRoot = new GameObject("ContentRoot").transform;
                CgHandle = new GameObject("CgHandle").transform;
                CgHandle.SetParent(ContentRoot, false);
                CgHandle.localPosition = new Vector3(0.1f, 0.85f, 0.4f);
                CgHandle.localRotation = Quaternion.Euler(cgLocalEuler);
                ContentRoot.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            }

            public void Dispose()
            {
                Object.DestroyImmediate(ContentRoot.gameObject);
            }
        }
    }
}
