using System;
using System.Collections.Generic;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    public static class CircleFitting3D
    {
        public const float DefaultThreshold = 0.005f;
        public const float CollinearityEpsilon = 1e-10f;

        public static Circle3D? CircleFrom3Points(Vector3 a, Vector3 b, Vector3 c)
        {
            var t = b - a;
            var u = c - a;
            var v = c - b;
            var w = Vector3.Cross(t, u);
            var wsl = Vector3.Dot(w, w);

            if (wsl < CollinearityEpsilon)
            {
                return null;
            }

            var iwsl2 = 1f / (2f * wsl);
            var tt = Vector3.Dot(t, t);
            var uu = Vector3.Dot(u, u);

            var center = a + (u * tt * Vector3.Dot(u, v) - t * uu * Vector3.Dot(t, v)) * iwsl2;
            var radius = Mathf.Sqrt(tt * uu * Vector3.Dot(v, v) * iwsl2 * 0.5f);
            var normal = w / Mathf.Sqrt(wsl);

            if (radius < 1e-6f || float.IsNaN(radius))
            {
                return null;
            }

            return new Circle3D(center, RotationFromNormal(normal), radius);
        }

        public static float DistanceToCircle(Vector3 point, Circle3D circle)
        {
            var v = point - circle.Center;
            var normal = circle.Normal;
            var projOnNormal = Vector3.Dot(v, normal) * normal;
            var vInPlane = v - projOnNormal;

            var distRadial = vInPlane.magnitude - circle.Radius;
            var distAxial = projOnNormal.magnitude;

            return Mathf.Sqrt(distRadial * distRadial + distAxial * distAxial);
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
            var iterations = maxIterations > 0
                ? maxIterations
                : Mathf.Max(50, EstimateIterationCount(0.9f, successProbability));

            Circle3D? bestCircle = null;
            var bestScore = float.MaxValue;
            var bestInliers = Array.Empty<int>();

            for (var i = 0; i < iterations; i++)
            {
                var i0 = random.Next(pointCount);
                var i1 = random.Next(pointCount);
                var i2 = random.Next(pointCount);

                if (i0 == i1 || i1 == i2 || i0 == i2)
                {
                    continue;
                }

                var candidate = CircleFrom3Points(points[i0], points[i1], points[i2]);
                if (!candidate.HasValue)
                {
                    continue;
                }

                var circle = candidate.Value;
                var score = 0f;
                var inlierCount = 0;

                for (var j = 0; j < pointCount; j++)
                {
                    var dist = DistanceToCircle(points[j], circle);
                    var distSq = dist * dist;
                    score += distSq < thresholdSq ? distSq : thresholdSq;

                    if (distSq < thresholdSq)
                    {
                        inlierCount++;
                    }
                }

                if (score < bestScore)
                {
                    bestScore = score;
                    bestCircle = circle;
                    bestInliers = CollectInliers(points, circle, threshold);

                    if (inlierCount > 0)
                    {
                        var inlierRatio = inlierCount / (float)pointCount;
                        var adaptiveIterations = EstimateIterationCount(inlierRatio, successProbability);
                        if (adaptiveIterations > iterations)
                        {
                            iterations = adaptiveIterations;
                        }
                    }
                }
            }

            if (!bestCircle.HasValue || bestInliers.Length < 3)
            {
                throw new InvalidOperationException("MSAC failed to find a valid circle model.");
            }

            var refined = RefineWithInliers(points, bestInliers);
            return new CircleFitResult(refined, bestInliers);
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

            var (eigenvalues, eigenvectors) = SymmetricEigenDecomposition3x3.Decompose(cov);
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

        private static int[] CollectInliers(IReadOnlyList<Vector3> points, Circle3D circle, float threshold)
        {
            var thresholdSq = threshold * threshold;
            var inliers = new List<int>(points.Count);
            for (var i = 0; i < points.Count; i++)
            {
                var dist = DistanceToCircle(points[i], circle);
                if (dist * dist < thresholdSq)
                {
                    inliers.Add(i);
                }
            }

            return inliers.ToArray();
        }

        private static int EstimateIterationCount(float inlierRatio, float successProbability)
        {
            inlierRatio = Mathf.Clamp(inlierRatio, 0.01f, 0.999f);
            var sampleSize = 3f;
            var w = Mathf.Pow(inlierRatio, sampleSize);
            if (w >= 0.999999f)
            {
                return 50;
            }

            var logOneMinusP = Mathf.Log(1f - successProbability);
            var logOneMinusW = Mathf.Log(1f - w);
            return Mathf.CeilToInt(logOneMinusP / logOneMinusW);
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

    internal static class SymmetricLinearSolver3x3
    {
        public static bool TrySolve(Matrix3x3 a, Vector3 b, out Vector3 x)
        {
            var det =
                a.M00 * (a.M11 * a.M22 - a.M12 * a.M21) -
                a.M01 * (a.M10 * a.M22 - a.M12 * a.M20) +
                a.M02 * (a.M10 * a.M21 - a.M11 * a.M20);

            if (Mathf.Abs(det) < 1e-12f)
            {
                x = default;
                return false;
            }

            var invDet = 1f / det;
            var i00 = (a.M11 * a.M22 - a.M12 * a.M21) * invDet;
            var i01 = (a.M02 * a.M21 - a.M01 * a.M22) * invDet;
            var i02 = (a.M01 * a.M12 - a.M02 * a.M11) * invDet;
            var i10 = (a.M12 * a.M20 - a.M10 * a.M22) * invDet;
            var i11 = (a.M00 * a.M22 - a.M02 * a.M20) * invDet;
            var i12 = (a.M02 * a.M10 - a.M00 * a.M12) * invDet;
            var i20 = (a.M10 * a.M21 - a.M11 * a.M20) * invDet;
            var i21 = (a.M01 * a.M20 - a.M00 * a.M21) * invDet;
            var i22 = (a.M00 * a.M11 - a.M01 * a.M10) * invDet;

            x = new Vector3(
                i00 * b.x + i01 * b.y + i02 * b.z,
                i10 * b.x + i11 * b.y + i12 * b.z,
                i20 * b.x + i21 * b.y + i22 * b.z);
            return true;
        }
    }
}
