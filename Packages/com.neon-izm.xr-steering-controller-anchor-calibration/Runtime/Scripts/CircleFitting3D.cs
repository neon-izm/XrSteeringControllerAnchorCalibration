using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    public static class CircleFitting3D
    {
        public const float DefaultThreshold = 0.005f;
        public const float CollinearityEpsilon = 1e-10f;

        /// <summary>
        /// Hard ceiling for adaptive MSAC growth when <c>maxIterations &lt;= 0</c>.
        /// Without this, a temporarily low inlier ratio can request ~1e5–1e6 iterations
        /// and stall Android HMDs for tens of seconds. 50000 preserves Quest device-fixture
        /// normals (seeded MSAC) while still cutting the uncapped ~250k worst case.
        /// </summary>
        public const int DefaultMaxAdaptiveIterations = 50000;

        const int ParallelInnerBatchSize = 32;

        public static Circle3D? CircleFrom3Points(Vector3 a, Vector3 b, Vector3 c)
        {
            if (!CircleFittingBurst.TryCircleFrom3Points(a, b, c, out var center, out var normal, out var radius))
            {
                return null;
            }

            return new Circle3D(center, RotationFromNormal(normal), radius);
        }

        public static float DistanceToCircle(Vector3 point, Circle3D circle)
        {
            var distSq = CircleFittingBurst.DistanceToCircleSq(point, circle.Center, circle.Normal, circle.Radius);
            return Mathf.Sqrt(distSq);
        }

        public static CircleFitResult FitCircleMsac(
            IReadOnlyList<Vector3> points,
            float threshold = DefaultThreshold,
            int maxIterations = 0,
            float successProbability = 0.999f,
            System.Random random = null)
        {
            if (points == null || points.Count < 3)
            {
                throw new ArgumentException("At least 3 points are required.", nameof(points));
            }

            random ??= new System.Random();
            var pointCount = points.Count;
            var thresholdSq = threshold * threshold;

            // maxIterations > 0 is a hard ceiling. Otherwise adapt up to DefaultMaxAdaptiveIterations.
            var iterationCap = maxIterations > 0 ? maxIterations : DefaultMaxAdaptiveIterations;
            var iterations = maxIterations > 0
                ? maxIterations
                : Mathf.Max(50, EstimateIterationCount(0.9f, successProbability));
            iterations = Mathf.Min(iterations, iterationCap);

            var nativePoints = new NativeArray<float3>(pointCount, Allocator.TempJob);
            var triples = new NativeArray<int3>(iterationCap, Allocator.TempJob);
            var scores = new NativeArray<float>(iterationCap, Allocator.TempJob);
            var inlierCounts = new NativeArray<int>(iterationCap, Allocator.TempJob);
            var centers = new NativeArray<float3>(iterationCap, Allocator.TempJob);
            var normals = new NativeArray<float3>(iterationCap, Allocator.TempJob);
            var radii = new NativeArray<float>(iterationCap, Allocator.TempJob);
            var inlierBuffer = new NativeArray<int>(pointCount, Allocator.TempJob);
            try
            {
                for (var i = 0; i < pointCount; i++)
                {
                    nativePoints[i] = points[i];
                }

                // Pre-sample with System.Random so parallel batches keep the same draw order as sequential MSAC.
                for (var i = 0; i < iterationCap; i++)
                {
                    triples[i] = new int3(random.Next(pointCount), random.Next(pointCount), random.Next(pointCount));
                    scores[i] = float.MaxValue;
                }

                Circle3D? bestCircle = null;
                var bestScore = float.MaxValue;
                var bestInliers = Array.Empty<int>();
                var scoredUpTo = 0;
                var candidateIndex = 0;

                while (candidateIndex < iterations)
                {
                    if (scoredUpTo < iterations)
                    {
                        ScheduleScoreCandidates(
                            nativePoints,
                            triples,
                            scores,
                            inlierCounts,
                            centers,
                            normals,
                            radii,
                            thresholdSq,
                            start: scoredUpTo,
                            count: iterations - scoredUpTo);
                        scoredUpTo = iterations;
                    }

                    var score = scores[candidateIndex];
                    if (!(score < bestScore))
                    {
                        candidateIndex++;
                        continue;
                    }

                    bestScore = score;
                    var center = centers[candidateIndex];
                    var normal = normals[candidateIndex];
                    var radius = radii[candidateIndex];
                    bestCircle = new Circle3D(center, RotationFromNormal(normal), radius);

                    ScheduleCollectInliers(
                        nativePoints, center, normal, radius, threshold, inlierBuffer, out var collected);
                    if (collected < 3)
                    {
                        candidateIndex++;
                        continue;
                    }

                    bestInliers = new int[collected];
                    for (var k = 0; k < collected; k++)
                    {
                        bestInliers[k] = inlierBuffer[k];
                    }

                    var inlierCount = inlierCounts[candidateIndex];
                    if (inlierCount > 0)
                    {
                        var inlierRatio = inlierCount / (float)pointCount;
                        var adaptiveIterations = EstimateIterationCount(inlierRatio, successProbability);
                        if (adaptiveIterations > iterations)
                        {
                            iterations = Mathf.Min(adaptiveIterations, iterationCap);
                        }
                    }

                    candidateIndex++;
                }

                if (!bestCircle.HasValue || bestInliers.Length < 3)
                {
                    throw new InvalidOperationException("MSAC failed to find a valid circle model.");
                }

                var refined = RefineWithInliers(points, bestInliers);
                return new CircleFitResult(refined, bestInliers);
            }
            finally
            {
                DisposeIfCreated(nativePoints);
                DisposeIfCreated(triples);
                DisposeIfCreated(scores);
                DisposeIfCreated(inlierCounts);
                DisposeIfCreated(centers);
                DisposeIfCreated(normals);
                DisposeIfCreated(radii);
                DisposeIfCreated(inlierBuffer);
            }
        }

        public static Circle3D RefineWithInliers(IReadOnlyList<Vector3> points, IReadOnlyList<int> inlierIndices)
        {
            if (inlierIndices == null || inlierIndices.Count < 3)
            {
                throw new ArgumentException("At least 3 inlier indices are required.", nameof(inlierIndices));
            }

            var inlierCount = inlierIndices.Count;
            var centroid = Vector3.zero;
            for (var i = 0; i < inlierCount; i++)
            {
                centroid += points[inlierIndices[i]];
            }

            centroid /= inlierCount;

            var cov = Matrix3x3.Zero;
            for (var i = 0; i < inlierCount; i++)
            {
                var p = points[inlierIndices[i]] - centroid;
                cov.M00 += p.x * p.x;
                cov.M01 += p.x * p.y;
                cov.M02 += p.x * p.z;
                cov.M11 += p.y * p.y;
                cov.M12 += p.y * p.z;
                cov.M22 += p.z * p.z;
            }

            cov.M10 = cov.M01;
            cov.M20 = cov.M02;
            cov.M21 = cov.M12;
            cov /= inlierCount;

            var (_, eigenvectors) = SymmetricEigenDecomposition3x3.Decompose(cov);
            var normal = eigenvectors[0].normalized;
            var u = eigenvectors[2].normalized;
            var v = Vector3.Cross(normal, u).normalized;

            var xs = new float[inlierCount];
            var ys = new float[inlierCount];
            for (var i = 0; i < inlierCount; i++)
            {
                var centered = points[inlierIndices[i]] - centroid;
                xs[i] = Vector3.Dot(centered, u);
                ys[i] = Vector3.Dot(centered, v);
            }

            var (xc, yc, radius) = CircleFit2D.Fit(xs, ys);
            var center3D = centroid + xc * u + yc * v;

            return new Circle3D(center3D, RotationFromNormal(normal), radius);
        }

        public static Quaternion RotationFromNormal(Vector3 normal)
        {
            var forward = normal.normalized;
            var up = Vector3.up;
            if (Mathf.Abs(Vector3.Dot(forward, up)) > 0.999f)
            {
                up = Vector3.right;
            }

            return Quaternion.LookRotation(forward, up);
        }

        static void ScheduleScoreCandidates(
            NativeArray<float3> points,
            NativeArray<int3> triples,
            NativeArray<float> scores,
            NativeArray<int> inlierCounts,
            NativeArray<float3> centers,
            NativeArray<float3> normals,
            NativeArray<float> radii,
            float thresholdSq,
            int start,
            int count)
        {
            if (count <= 0)
            {
                return;
            }

            var job = new CircleFittingBurst.ScoreCandidatesJob
            {
                Points = points,
                Triples = triples,
                ThresholdSq = thresholdSq,
                Start = start,
                Scores = scores,
                InlierCounts = inlierCounts,
                Centers = centers,
                Normals = normals,
                Radii = radii
            };
            job.Schedule(count, ParallelInnerBatchSize).Complete();
        }

        static void ScheduleCollectInliers(
            NativeArray<float3> points,
            float3 center,
            float3 normal,
            float radius,
            float threshold,
            NativeArray<int> inliers,
            out int count)
        {
            var countBuf = new NativeArray<int>(1, Allocator.TempJob);
            try
            {
                var job = new CircleFittingBurst.CollectInliersJob
                {
                    Points = points,
                    Center = center,
                    Normal = normal,
                    Radius = radius,
                    Threshold = threshold,
                    Inliers = inliers,
                    OutCount = countBuf
                };
                job.Schedule().Complete();
                count = countBuf[0];
            }
            finally
            {
                countBuf.Dispose();
            }
        }

        static void DisposeIfCreated<T>(NativeArray<T> array) where T : struct
        {
            if (array.IsCreated)
            {
                array.Dispose();
            }
        }

        static int EstimateIterationCount(float inlierRatio, float successProbability)
        {
            inlierRatio = Mathf.Clamp(inlierRatio, 0.01f, 0.999f);
            var w = Mathf.Pow(inlierRatio, 3f);
            if (w >= 0.999999f)
            {
                return 50;
            }

            var logOneMinusP = Mathf.Log(1f - successProbability);
            var logOneMinusW = Mathf.Log(1f - w);
            return Mathf.CeilToInt(logOneMinusP / logOneMinusW);
        }
    }

    /// <summary>
    /// Burst jobs for MSAC candidate scoring (parallel) and inlier collection.
    /// </summary>
    [BurstCompile]
    internal static class CircleFittingBurst
    {
        internal const float CollinearityEpsilon = 1e-10f;

        [BurstCompile(CompileSynchronously = true)]
        public struct ScoreCandidatesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Points;
            [ReadOnly] public NativeArray<int3> Triples;
            public float ThresholdSq;
            public int Start;

            // Execute(index) writes absolute slots Start+index.
            [NativeDisableParallelForRestriction] public NativeArray<float> Scores;
            [NativeDisableParallelForRestriction] public NativeArray<int> InlierCounts;
            [NativeDisableParallelForRestriction] public NativeArray<float3> Centers;
            [NativeDisableParallelForRestriction] public NativeArray<float3> Normals;
            [NativeDisableParallelForRestriction] public NativeArray<float> Radii;

            public void Execute(int index)
            {
                var absIndex = Start + index;
                var sample = Triples[absIndex];
                var i0 = sample.x;
                var i1 = sample.y;
                var i2 = sample.z;

                if (i0 == i1 || i1 == i2 || i0 == i2
                    || !TryCircleFrom3Points(Points[i0], Points[i1], Points[i2], out var center, out var normal, out var radius))
                {
                    Scores[absIndex] = float.MaxValue;
                    InlierCounts[absIndex] = 0;
                    Centers[absIndex] = default;
                    Normals[absIndex] = default;
                    Radii[absIndex] = 0f;
                    return;
                }

                var score = 0f;
                var inlierCount = 0;
                for (var j = 0; j < Points.Length; j++)
                {
                    var dist = math.sqrt(DistanceToCircleSq(Points[j], center, normal, radius));
                    var distSq = dist * dist;
                    score += distSq < ThresholdSq ? distSq : ThresholdSq;
                    if (distSq < ThresholdSq)
                    {
                        inlierCount++;
                    }
                }

                Scores[absIndex] = score;
                InlierCounts[absIndex] = inlierCount;
                Centers[absIndex] = center;
                Normals[absIndex] = normal;
                Radii[absIndex] = radius;
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        public struct CollectInliersJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Points;
            public float3 Center;
            public float3 Normal;
            public float Radius;
            public float Threshold;
            public NativeArray<int> Inliers;
            public NativeArray<int> OutCount;

            public void Execute()
            {
                var thresholdSq = Threshold * Threshold;
                var count = 0;
                for (var i = 0; i < Points.Length; i++)
                {
                    var dist = math.sqrt(DistanceToCircleSq(Points[i], Center, Normal, Radius));
                    if (dist * dist < thresholdSq)
                    {
                        Inliers[count++] = i;
                    }
                }

                OutCount[0] = count;
            }
        }

        public static bool TryCircleFrom3Points(
            float3 a,
            float3 b,
            float3 c,
            out float3 center,
            out float3 normal,
            out float radius)
        {
            center = default;
            normal = default;
            radius = 0f;

            var t = b - a;
            var u = c - a;
            var v = c - b;
            var w = math.cross(t, u);
            var wsl = math.dot(w, w);
            if (wsl < CollinearityEpsilon)
            {
                return false;
            }

            var iwsl2 = 1f / (2f * wsl);
            var tt = math.dot(t, t);
            var uu = math.dot(u, u);

            center = a + (u * tt * math.dot(u, v) - t * uu * math.dot(t, v)) * iwsl2;
            radius = math.sqrt(tt * uu * math.dot(v, v) * iwsl2 * 0.5f);
            normal = w / math.sqrt(wsl);

            if (radius < 1e-6f || !math.isfinite(radius))
            {
                return false;
            }

            return true;
        }

        public static float DistanceToCircleSq(float3 point, float3 center, float3 normal, float radius)
        {
            var delta = point - center;
            var projOnNormal = math.dot(delta, normal) * normal;
            var vInPlane = delta - projOnNormal;
            var distRadial = math.length(vInPlane) - radius;
            var distAxial = math.length(projOnNormal);
            return distRadial * distRadial + distAxial * distAxial;
        }
    }

    internal struct Matrix3x3
    {
        public float M00;
        public float M01;
        public float M02;
        public float M10;
        public float M11;
        public float M12;
        public float M20;
        public float M21;
        public float M22;

        public Matrix3x3(
            float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22)
        {
            M00 = m00;
            M01 = m01;
            M02 = m02;
            M10 = m10;
            M11 = m11;
            M12 = m12;
            M20 = m20;
            M21 = m21;
            M22 = m22;
        }

        public static Matrix3x3 Zero => default;

        public static Matrix3x3 operator /(Matrix3x3 m, float scalar)
        {
            return new Matrix3x3(
                m.M00 / scalar, m.M01 / scalar, m.M02 / scalar,
                m.M10 / scalar, m.M11 / scalar, m.M12 / scalar,
                m.M20 / scalar, m.M21 / scalar, m.M22 / scalar);
        }
    }

    internal static class SymmetricEigenDecomposition3x3
    {
        public static (float[] eigenvalues, Vector3[] eigenvectors) Decompose(Matrix3x3 matrix)
        {
            var a = new float[3, 3]
            {
                { matrix.M00, matrix.M01, matrix.M02 },
                { matrix.M01, matrix.M11, matrix.M12 },
                { matrix.M02, matrix.M12, matrix.M22 }
            };

            Jacobi(a, out var eigenvalues, out var eigenvectors);

            var indices = new[] { 0, 1, 2 };
            Array.Sort(indices, (i, j) => eigenvalues[i].CompareTo(eigenvalues[j]));

            var sortedValues = new float[3];
            var sortedVectors = new Vector3[3];
            for (var k = 0; k < 3; k++)
            {
                var idx = indices[k];
                sortedValues[k] = eigenvalues[idx];
                sortedVectors[k] = new Vector3(
                    eigenvectors[0, idx],
                    eigenvectors[1, idx],
                    eigenvectors[2, idx]);
            }

            return (sortedValues, sortedVectors);
        }

        private static void Jacobi(float[,] a, out float[] eigenvalues, out float[,] eigenvectors)
        {
            const int n = 3;
            eigenvectors = new float[n, n];
            for (var i = 0; i < n; i++)
            {
                eigenvectors[i, i] = 1f;
            }

            for (var iter = 0; iter < 50; iter++)
            {
                var p = 0;
                var q = 1;
                var max = Mathf.Abs(a[0, 1]);
                if (Mathf.Abs(a[0, 2]) > max)
                {
                    max = Mathf.Abs(a[0, 2]);
                    p = 0;
                    q = 2;
                }

                if (Mathf.Abs(a[1, 2]) > max)
                {
                    p = 1;
                    q = 2;
                }

                if (max < 1e-12f)
                {
                    break;
                }

                var app = a[p, p];
                var aqq = a[q, q];
                var apq = a[p, q];
                var phi = 0.5f * (aqq - app) / apq;
                var t = 1f / (Mathf.Abs(phi) + Mathf.Sqrt(phi * phi + 1f));
                if (phi < 0f)
                {
                    t = -t;
                }

                var c = 1f / Mathf.Sqrt(1f + t * t);
                var s = t * c;
                var tau = s / (1f + c);

                a[p, p] -= t * apq;
                a[q, q] += t * apq;
                a[p, q] = 0f;
                a[q, p] = 0f;

                for (var r = 0; r < n; r++)
                {
                    if (r != p && r != q)
                    {
                        var arp = a[r, p];
                        var arq = a[r, q];
                        a[r, p] = arp - s * (arq + arp * tau);
                        a[p, r] = a[r, p];
                        a[r, q] = arq + s * (arp - arq * tau);
                        a[q, r] = a[r, q];
                    }
                }

                for (var r = 0; r < n; r++)
                {
                    var vrP = eigenvectors[r, p];
                    var vrQ = eigenvectors[r, q];
                    eigenvectors[r, p] = vrP - s * (vrQ + vrP * tau);
                    eigenvectors[r, q] = vrQ + s * (vrP - vrQ * tau);
                }
            }

            eigenvalues = new float[] { a[0, 0], a[1, 1], a[2, 2] };
        }
    }
}
