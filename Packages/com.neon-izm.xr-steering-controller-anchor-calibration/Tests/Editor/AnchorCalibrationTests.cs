using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public class AnchorCalibrationTests
    {
        private const float CenterTolerance = 0.001f;
        private const float AngleToleranceDeg = 1f;

        [Test]
        public void StoredModelView_RoundTrip_PreservesRigidTransform()
        {
            var estimatedWorld = Circle3D.FromPose(Vector3.one, Quaternion.Euler(12f, 34f, 0f), 0.14f);
            var targetHandle = new CgHandlePose(new Vector3(2f, 0.5f, -1f), Quaternion.Euler(-8f, 60f, 0f));
            var modelView = AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);

            var reconstructed = new Matrix4x4();
            reconstructed.SetRow(0, modelView.GetRow(0));
            reconstructed.SetRow(1, modelView.GetRow(1));
            reconstructed.SetRow(2, modelView.GetRow(2));
            reconstructed.SetRow(3, modelView.GetRow(3));

            var mappedOriginal = AnchorCalibration.TransformCircleRigid(modelView, estimatedWorld);
            var mappedReconstructed = AnchorCalibration.TransformCircleRigid(reconstructed, estimatedWorld);

            Assert.That(Vector3.Distance(mappedOriginal.Position, mappedReconstructed.Position), Is.LessThan(1e-4f));
            Assert.That(Vector3.Distance(mappedOriginal.Position, targetHandle.Position), Is.LessThan(CenterTolerance));
        }

        [Test]
        public void AlignNormalToTargetForward_FlipsOppositeHemisphere()
        {
            var targetHandle = new CgHandlePose(Vector3.zero, Quaternion.Euler(20f, 40f, 0f));
            var opposite = Circle3D.FromPose(Vector3.up, CircleFitting3D.RotationFromNormal(-targetHandle.Forward), 0.15f);

            var aligned = AnchorCalibration.AlignNormalToTargetForward(opposite, targetHandle.Forward);

            Assert.That(Vector3.Dot(aligned.Normal, targetHandle.Forward), Is.GreaterThan(0f));
            Assert.That(Vector3.Distance(aligned.Position, opposite.Position), Is.LessThan(1e-5f));
            Assert.That(aligned.Radius, Is.EqualTo(opposite.Radius).Within(1e-5f));
        }

        [Test]
        public void ComputeModelView_MapsEstimatedWorldCircleToCgTargetPose()
        {
            var estimatedWorld = Circle3D.FromPose(
                new Vector3(0.2f, 0.1f, -0.3f),
                Quaternion.Euler(20f, 35f, 0f),
                0.14f);

            var targetHandle = new CgHandlePose(
                new Vector3(1.1f, 0.8f, 0.4f),
                Quaternion.Euler(-15f, 70f, 0f));

            var modelView = AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);
            var mapped = AnchorCalibration.TransformCircleRigid(modelView, estimatedWorld);

            Assert.That(Vector3.Distance(mapped.Position, targetHandle.Position), Is.LessThan(CenterTolerance));
            Assert.That(Vector3.Angle(mapped.Normal, targetHandle.Forward), Is.LessThan(AngleToleranceDeg));
            Assert.That(mapped.Radius, Is.EqualTo(estimatedWorld.Radius).Within(1e-5f));
        }

        [Test]
        public void TransformWorldPointsToCg_AppliesCalibratedModelView()
        {
            var estimatedWorld = Circle3D.FromPose(Vector3.one, Quaternion.Euler(0f, 45f, 0f), 0.15f);
            var targetHandle = new CgHandlePose(new Vector3(2f, 0f, 1f), Quaternion.Euler(0f, 90f, 0f));
            var modelView = AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);

            var worldPoints = new[] { Vector3.zero, Vector3.right };
            var cgPoints = AnchorCalibration.TransformWorldPointsToCg(worldPoints, modelView);

            Assert.That(Vector3.Distance(cgPoints[0], AnchorCalibration.WorldToCg(worldPoints[0], modelView)), Is.LessThan(1e-5f));
            Assert.That(Vector3.Distance(cgPoints[1], AnchorCalibration.WorldToCg(worldPoints[1], modelView)), Is.LessThan(1e-5f));
        }

        [Test]
        public void Calibrate_ReturnsOnlyModelView()
        {
            var data = TestDataGenerator.GenerateArc(pointCount: 150, outlierRatio: 0.05f, seed: 99);
            var targetHandle = new CgHandlePose(
                new Vector3(0.5f, 1.2f, -0.2f),
                Quaternion.Euler(10f, 55f, 0f));

            var result = AnchorCalibration.Calibrate(data.Points, targetHandle, random: new System.Random(4));

            Assert.That(result.ModelView, Is.Not.EqualTo(Matrix4x4.zero));
        }

        [Test]
        public void Calibrate_WorldPointsAndKnownCgPose_ProducesAlignedModelView()
        {
            var data = TestDataGenerator.GenerateArc(pointCount: 150, outlierRatio: 0.05f, seed: 99);
            var targetHandle = new CgHandlePose(
                new Vector3(0.5f, 1.2f, -0.2f),
                Quaternion.Euler(10f, 55f, 0f));

            var fit = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(4));
            var result = AnchorCalibration.Calibrate(data.Points, targetHandle, random: new System.Random(4));
            var aligned = AnchorCalibration.AlignNormalToTargetForward(fit.Circle, targetHandle.Forward);
            var mapped = AnchorCalibration.TransformCircleRigid(result.ModelView, aligned);

            Assert.That(Vector3.Distance(mapped.Position, targetHandle.Position), Is.LessThan(0.01f));
            Assert.That(Vector3.Angle(mapped.Normal, targetHandle.Forward), Is.LessThan(3f));
        }

        [Test]
        public void EndToEnd_GenerateAndCalibrate_WithoutKnownCgRadius()
        {
            var data = TestDataGenerator.GenerateArc(pointCount: 150, outlierRatio: 0.05f, seed: 99);
            var targetHandle = new CgHandlePose(
                new Vector3(0.5f, 1.2f, -0.2f),
                Quaternion.Euler(10f, 55f, 0f));

            var fit = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(4));
            var result = AnchorCalibration.Calibrate(data.Points, targetHandle, random: new System.Random(4));
            var aligned = AnchorCalibration.AlignNormalToTargetForward(fit.Circle, targetHandle.Forward);
            var mapped = AnchorCalibration.TransformCircleRigid(result.ModelView, aligned);

            Assert.That(Vector3.Distance(mapped.Position, targetHandle.Position), Is.LessThan(0.01f));
            Assert.That(Vector3.Angle(mapped.Normal, targetHandle.Forward), Is.LessThan(3f));
            Assert.That(
                Mathf.Abs(mapped.Radius - data.GroundTruth.Radius),
                Is.LessThan(0.01f));
        }
    }
}
