using System;
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public static class ArcPointGenerator
    {
        public readonly struct GeneratedArc
        {
            public readonly List<Vector3> Points;
            public readonly Circle3D GroundTruth;

            public GeneratedArc(List<Vector3> points, Circle3D groundTruth)
            {
                Points = points;
                GroundTruth = groundTruth;
            }
        }

        public static GeneratedArc Generate(
            Vector3 center,
            Quaternion rotation,
            float radius,
            ArcGenerationSettings settings)
        {
            if (settings.PointCount < 3)
            {
                throw new ArgumentOutOfRangeException(nameof(settings.PointCount));
            }

            var random = new System.Random(settings.RandomSeed);
            var rollZeroRotation = CircleFitting3D.EnforceZeroRoll(rotation);
            var right = rollZeroRotation * Vector3.right;
            var up = rollZeroRotation * Vector3.up;
            var halfAngleDeg = settings.ArcHalfAngleDeg > 0f ? settings.ArcHalfAngleDeg : 45f;

            var points = new List<Vector3>(settings.PointCount);
            for (var i = 0; i < settings.PointCount; i++)
            {
                var t = i / (float)(settings.PointCount - 1);
                var angle = Mathf.Lerp(-halfAngleDeg, halfAngleDeg, t) * Mathf.Deg2Rad;
                var point = center + radius * (Mathf.Cos(angle) * right + Mathf.Sin(angle) * up);
                point += SampleGaussianVector3(random, settings.GaussianSigmaMeters);
                points.Add(point);
            }

            if (settings.OutlierRatio > 0f)
            {
                var outlierCount = Mathf.Clamp(
                    Mathf.RoundToInt(settings.PointCount * settings.OutlierRatio),
                    0,
                    settings.PointCount);

                for (var i = 0; i < outlierCount; i++)
                {
                    var index = random.Next(settings.PointCount);
                    var direction = RandomUnitVector(random);
                    var magnitude = Mathf.Lerp(
                        settings.OutlierMinOffsetMeters,
                        settings.OutlierMaxOffsetMeters,
                        (float)random.NextDouble());
                    points[index] = center + direction * (radius + magnitude);
                }
            }

            return new GeneratedArc(points, new Circle3D(center, rollZeroRotation, radius));
        }

        public static GeneratedArc GenerateRandom(ArcGenerationSettings settings)
        {
            var random = new System.Random(settings.RandomSeed);
            var center = new Vector3(
                (float)random.NextDouble() * 2f - 1f,
                (float)random.NextDouble() * 1.5f,
                (float)random.NextDouble() * 2f - 1f);
            var rotation = Quaternion.Euler(
                (float)random.NextDouble() * 360f,
                (float)random.NextDouble() * 360f,
                0f);
            var radius = Mathf.Lerp(0.12f, 0.18f, (float)random.NextDouble());
            return Generate(center, rotation, radius, settings);
        }

        static Vector3 SampleGaussianVector3(System.Random random, float sigma)
        {
            return new Vector3(
                SampleGaussian(random) * sigma,
                SampleGaussian(random) * sigma,
                SampleGaussian(random) * sigma);
        }

        static float SampleGaussian(System.Random random)
        {
            var u1 = 1f - (float)random.NextDouble();
            var u2 = 1f - (float)random.NextDouble();
            return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
        }

        static Vector3 RandomUnitVector(System.Random random)
        {
            var z = (float)random.NextDouble() * 2f - 1f;
            var theta = (float)random.NextDouble() * 2f * Mathf.PI;
            var r = Mathf.Sqrt(1f - z * z);
            return new Vector3(r * Mathf.Cos(theta), r * Mathf.Sin(theta), z);
        }
    }
}
