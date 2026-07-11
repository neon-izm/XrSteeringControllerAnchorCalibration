using System;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    [Serializable]
    public struct ArcGenerationSettings
    {
        public int PointCount;
        public float OutlierRatio;
        public float GaussianSigmaMeters;
        public float OutlierMinOffsetMeters;
        public float OutlierMaxOffsetMeters;
        public int RandomSeed;
        public float ArcHalfAngleDeg;

        public static ArcGenerationSettings Default => new ArcGenerationSettings
        {
            PointCount = 120,
            OutlierRatio = 0.05f,
            GaussianSigmaMeters = 0.002f,
            OutlierMinOffsetMeters = 0.01f,
            OutlierMaxOffsetMeters = 0.05f,
            RandomSeed = 42,
            ArcHalfAngleDeg = 45f,
        };
    }
}
