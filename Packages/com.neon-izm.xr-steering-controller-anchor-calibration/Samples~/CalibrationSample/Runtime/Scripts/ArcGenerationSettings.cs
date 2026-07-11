using System;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration.Sample
{
    /// <summary>
    /// サンプル／テスト用の円弧点群生成パラメータ（データのみ。生成は Editor の ArcPointGenerator）。
    /// </summary>
    [Serializable]
    public struct ArcGenerationSettings
    {
        public int PointCount;
        public float OutlierRatio;
        public float GaussianSigmaMeters;
        public float OutlierMinOffsetMeters;
        public float OutlierMaxOffsetMeters;
        public int RandomSeed;
        /// <summary>弧の半開角（度）。合計弧長は 2 × ArcHalfAngleDeg。既定 45° → 合計 90°。</summary>
        [Tooltip("弧の半開角（度）。合計弧長 = 2 × この値。22.5 → 45°、45 → 90°。")]
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
