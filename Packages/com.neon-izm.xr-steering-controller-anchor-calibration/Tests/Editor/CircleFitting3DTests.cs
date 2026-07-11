using System;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public class CircleFitting3DTests
    {
        private const float CenterTolerance = 0.005f;
        private const float RadiusTolerance = 0.003f;
        private const float AngleToleranceDeg = 2f;

        [Test]
        public void Circle3D_ExposesPositionAndRotation()
        {
            var rotation = Quaternion.Euler(30f, 45f, 90f);
            var circle = Circle3D.FromPose(new Vector3(1f, 2f, 3f), rotation, 0.15f);

            Assert.That(circle.Position, Is.EqualTo(circle.Center));
            Assert.That(circle.Rotation, Is.EqualTo(circle.Rot));
            Assert.That(circle.Rotation, Is.EqualTo(rotation));
        }

        [Test]
        public void CircleFrom3Points_PerfectTriangle_ExactCircle()
        {
            var center = new Vector3(1f, 2f, 3f);
            var rot = Quaternion.Euler(30f, 45f, 0f);
            var radius = 0.15f;
            var right = rot * Vector3.right;
            var up = rot * Vector3.up;

            var a = center + radius * right;
            var b = center + radius * up;
            var c = center - radius * right;

            var circle = CircleFitting3D.CircleFrom3Points(a, b, c);
            Assert.That(circle, Is.Not.Null);

            AssertCenter(circle.Value, center);
            AssertRadius(circle.Value, radius);
            AssertNormalAngle(circle.Value, rot * Vector3.forward);
        }

        [Test]
        public void CircleFitting_ShortArc45Deg_ExactResult()
        {
            var data = TestDataGenerator.GenerateArcAt(
                new Vector3(0.1f, 1f, 0.2f),
                Quaternion.Euler(25f, 40f, 0f),
                0.15f,
                pointCount: 120,
                gaussianSigma: 0f,
                arcHalfAngleDeg: 22.5f,
                seed: 42);

            var result = CircleFitting3D.FitCircleMsac(
                data.Points,
                threshold: 0.001f,
                maxIterations: 200,
                random: new System.Random(0));

            AssertCenter(result.Circle, data.GroundTruth.Center, tolerance: 0.002f);
            AssertRadius(result.Circle, data.GroundTruth.Radius, tolerance: 0.002f);
            AssertNormalAngle(result.Circle, data.GroundTruth.Normal);
        }

        [Test]
        public void CircleFitting_ShortArc45Deg_WithNoise_WithinTolerance()
        {
            var data = TestDataGenerator.GenerateArcAt(
                Vector3.zero,
                Quaternion.Euler(15f, 30f, 0f),
                0.15f,
                pointCount: 150,
                gaussianSigma: 0.002f,
                arcHalfAngleDeg: 22.5f,
                seed: 11);

            var result = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(2));

            AssertRadius(result.Circle, data.GroundTruth.Radius, tolerance: 0.008f);
        }

        [Test]
        public void CircleFitting_PerfectArc_ExactResult()
        {
            var data = TestDataGenerator.GenerateArc(pointCount: 120, gaussianSigma: 0f, seed: 42);
            var result = CircleFitting3D.FitCircleMsac(data.Points, threshold: 0.001f, maxIterations: 200, random: new System.Random(0));

            AssertCenter(result.Circle, data.GroundTruth.Center, tolerance: 0.001f);
            AssertRadius(result.Circle, data.GroundTruth.Radius, tolerance: 0.001f);
            AssertNormalAngle(result.Circle, data.GroundTruth.Normal);
        }

        [Test]
        public void CircleFitting_GaussianNoise_WithinTolerance()
        {
            var data = TestDataGenerator.GenerateArc(pointCount: 200, gaussianSigma: 0.002f, seed: 42);
            var result = CircleFitting3D.FitCircleMsac(
                data.Points,
                maxIterations: 300,
                random: new System.Random(42));

            AssertCenter(result.Circle, data.GroundTruth.Center, tolerance: 0.01f);
            AssertRadius(result.Circle, data.GroundTruth.Radius, tolerance: 0.005f);
            AssertNormalAngle(result.Circle, data.GroundTruth.Normal);
        }

        [Test]
        public void CircleFitting_5PercentOutliers_Converges()
        {
            var data = TestDataGenerator.GenerateArc(
                pointCount: 150,
                outlierRatio: 0.05f,
                gaussianSigma: 0.002f,
                seed: 11);

            var result = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(2));

            AssertCenter(result.Circle, data.GroundTruth.Center);
            AssertRadius(result.Circle, data.GroundTruth.Radius, tolerance: 0.004f);
            AssertNormalAngle(result.Circle, data.GroundTruth.Normal);
            Assert.That(result.InlierIndices.Length, Is.GreaterThan(120));
        }

        [Test]
        public void CircleFitting_15PercentOutliers_Converges()
        {
            var data = TestDataGenerator.GenerateArc(
                pointCount: 200,
                outlierRatio: 0.15f,
                gaussianSigma: 0.002f,
                seed: 19);

            var result = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(3));

            AssertCenter(result.Circle, data.GroundTruth.Center, tolerance: 0.01f);
            AssertRadius(result.Circle, data.GroundTruth.Radius, tolerance: 0.005f);
            AssertNormalAngle(result.Circle, data.GroundTruth.Normal);
            Assert.That(result.InlierIndices.Length, Is.GreaterThan(140));
        }

        private static void AssertCenter(Circle3D actual, Vector3 expected, float tolerance = CenterTolerance)
        {
            Assert.That(Vector3.Distance(actual.Center, expected), Is.LessThan(tolerance));
        }

        private static void AssertRadius(Circle3D actual, float expected, float tolerance = RadiusTolerance)
        {
            Assert.That(Mathf.Abs(actual.Radius - expected), Is.LessThan(tolerance));
        }

        private static void AssertNormalAngle(Circle3D actual, Vector3 expectedNormal, float toleranceDeg = AngleToleranceDeg)
        {
            var angle = Vector3.Angle(actual.Normal, expectedNormal);
            var complementary = Vector3.Angle(actual.Normal, -expectedNormal);
            Assert.That(Mathf.Min(angle, complementary), Is.LessThan(toleranceDeg));
        }
    }
}
