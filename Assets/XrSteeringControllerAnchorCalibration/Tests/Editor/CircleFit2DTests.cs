using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public class CircleFit2DTests
    {
        const float TrueRadius = 0.15f;

        /// <summary>合計 45° 弧（±22.5°）。</summary>
        const float ShortArcHalfAngleDeg = 22.5f;

        [Test]
        public void Fit2D_PerfectFullCircle_MatchesGroundTruth()
        {
            var (xs, ys, xc, yc) = SampleArc2D(TrueRadius, arcHalfAngleDeg: 180f, pointCount: 72, noiseSigma: 0f, seed: 1);

            var fit = CircleFit2D.Fit(xs, ys);

            AssertRadius(fit, TrueRadius, tolerance: 1e-4f);
            AssertCenter(fit, xc, yc, tolerance: 1e-4f);
        }

        [Test]
        public void Fit2D_ShortArc45Deg_Noiseless_WithinTolerance()
        {
            var (xs, ys, xc, yc) = SampleArc2D(
                TrueRadius,
                ShortArcHalfAngleDeg,
                pointCount: 120,
                noiseSigma: 0f,
                seed: 42,
                centerX: 0.12f,
                centerY: -0.08f);

            var fit = CircleFit2D.Fit(xs, ys);

            AssertRadius(fit, TrueRadius, tolerance: 0.002f);
            AssertCenter(fit, xc, yc, tolerance: 0.005f);
        }

        [Test]
        public void Fit2D_ShortArc45Deg_WithNoise_WithinTolerance()
        {
            var (xs, ys, _, _) = SampleArc2D(
                TrueRadius,
                ShortArcHalfAngleDeg,
                pointCount: 150,
                noiseSigma: 0.002f,
                seed: 11,
                centerX: 0.12f,
                centerY: -0.08f);

            var fit = CircleFit2D.Fit(xs, ys);

            AssertRadius(fit, TrueRadius, tolerance: 0.005f);
        }

        [Test]
        public void RefineWithInliers_ShortArc45Deg_WithinTolerance()
        {
            var data = TestDataGenerator.GenerateArcAt(
                Vector3.zero,
                Quaternion.identity,
                TrueRadius,
                pointCount: 120,
                gaussianSigma: 0f,
                arcHalfAngleDeg: ShortArcHalfAngleDeg,
                seed: 42);

            var fit = CircleFitting3D.FitCircleMsac(
                data.Points,
                threshold: 0.001f,
                maxIterations: 200,
                random: new System.Random(0));

            Assert.That(Mathf.Abs(fit.Circle.Radius - data.GroundTruth.Radius), Is.LessThan(0.002f));
            Assert.That(
                Vector3.Distance(fit.Circle.Center, data.GroundTruth.Center),
                Is.LessThan(0.002f));
        }

        static (float[] xs, float[] ys, float centerX, float centerY) SampleArc2D(
            float radius,
            float arcHalfAngleDeg,
            int pointCount,
            float noiseSigma,
            int seed,
            float centerX = 0.12f,
            float centerY = -0.08f)
        {
            var random = new System.Random(seed);
            var xs = new float[pointCount];
            var ys = new float[pointCount];

            for (var i = 0; i < pointCount; i++)
            {
                var t = i / (float)(pointCount - 1);
                var angle = Mathf.Lerp(-arcHalfAngleDeg, arcHalfAngleDeg, t) * Mathf.Deg2Rad;
                var x = centerX + radius * Mathf.Cos(angle);
                var y = centerY + radius * Mathf.Sin(angle);
                if (noiseSigma > 0f)
                {
                    x += SampleGaussian(random) * noiseSigma;
                    y += SampleGaussian(random) * noiseSigma;
                }

                xs[i] = x;
                ys[i] = y;
            }

            return (xs, ys, centerX, centerY);
        }

        static float SampleGaussian(System.Random random)
        {
            var u1 = 1f - (float)random.NextDouble();
            var u2 = 1f - (float)random.NextDouble();
            return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
        }

        static void AssertRadius((float xc, float yc, float radius) fit, float expected, float tolerance)
        {
            Assert.That(Mathf.Abs(fit.radius - expected), Is.LessThan(tolerance));
        }

        static void AssertCenter((float xc, float yc, float radius) fit, float expectedX, float expectedY, float tolerance)
        {
            Assert.That(Mathf.Abs(fit.xc - expectedX), Is.LessThan(tolerance));
            Assert.That(Mathf.Abs(fit.yc - expectedY), Is.LessThan(tolerance));
        }
    }
}
