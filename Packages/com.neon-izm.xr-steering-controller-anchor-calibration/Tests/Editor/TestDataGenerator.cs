using System;
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public static class TestDataGenerator
    {
        public readonly struct GeneratedArcData
        {
            public readonly List<Vector3> Points;
            public readonly Circle3D GroundTruth;

            public GeneratedArcData(List<Vector3> points, Circle3D groundTruth)
            {
                Points = points;
                GroundTruth = groundTruth;
            }
        }

        /// <summary>
        /// HMD 世界座標系の円弧点群と、その真値 Circle3D を生成する。
        /// </summary>
        public static GeneratedArcData GenerateArc(
            int pointCount = 120,
            float outlierRatio = 0f,
            float gaussianSigma = 0.002f,
            float outlierMinOffset = 0.01f,
            float outlierMaxOffset = 0.05f,
            int? seed = null)
        {
            var settings = new ArcPointGenerator.Settings
            {
                PointCount = pointCount,
                OutlierRatio = outlierRatio,
                GaussianSigmaMeters = gaussianSigma,
                OutlierMinOffsetMeters = outlierMinOffset,
                OutlierMaxOffsetMeters = outlierMaxOffset,
                RandomSeed = seed ?? 42,
            };

            var generated = ArcPointGenerator.GenerateRandom(settings);
            return new GeneratedArcData(generated.Points, generated.GroundTruth);
        }
    }
}
