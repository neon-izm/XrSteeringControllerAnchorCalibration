using System;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    /// <summary>
    /// 2D 円フィット。短い弧では Taubin 代数フィットが最も誤差が小さい（500 試行ベンチマーク）。
    /// </summary>
    public static class CircleFit2D
    {
        const float SingularEpsilon = 1e-12f;
        const float DistanceEpsilon = 1e-10f;

        public static (float xc, float yc, float radius) Fit(float[] xs, float[] ys)
        {
            if (xs == null || ys == null || xs.Length != ys.Length || xs.Length < 3)
            {
                throw new ArgumentException("At least 3 paired coordinates are required.");
            }

            if (!TryFitTaubin(xs, ys, out var result))
            {
                throw new InvalidOperationException("Taubin circle fit failed: singular or degenerate input.");
            }

            return result;
        }

        public static bool TryFitTaubin(float[] xs, float[] ys, out (float xc, float yc, float radius) result)
        {
            result = default;
            var n = xs.Length;
            var meanX = 0f;
            var meanY = 0f;
            for (var i = 0; i < n; i++)
            {
                meanX += xs[i];
                meanY += ys[i];
            }

            meanX /= n;
            meanY /= n;

            var mxx = 0f;
            var myy = 0f;
            var mxy = 0f;
            var mxz = 0f;
            var myz = 0f;
            var mzz = 0f;
            for (var i = 0; i < n; i++)
            {
                var xi = xs[i] - meanX;
                var yi = ys[i] - meanY;
                var zi = xi * xi + yi * yi;
                mxx += xi * xi;
                myy += yi * yi;
                mxy += xi * yi;
                mxz += xi * zi;
                myz += yi * zi;
                mzz += zi * zi;
            }

            mxx /= n;
            myy /= n;
            mxy /= n;
            mxz /= n;
            myz /= n;
            mzz /= n;

            var mz = mxx + myy;
            var covXy = mxx * myy - mxy * mxy;
            var a3 = 4f * mz;
            var a2 = -3f * mz * mz - mzz;
            var a1 = mz * (mzz - mz * mz) + 4f * mz * covXy - mxz * mxz - myz * myz;
            var a0 = mxz * mxz * myy + myz * myz * mxx - 2f * mxz * myz * mxy - covXy * (mzz - mz * mz);
            var a22 = a2 + a2;
            var a33 = a3 + a3 + a3;

            var xNew = 0f;
            var polyValue = 1e20f;
            for (var iter = 0; iter < 99; iter++)
            {
                var polyOld = polyValue;
                polyValue = a0 + xNew * (a1 + xNew * (a2 + xNew * a3));
                if (Mathf.Abs(polyValue) > Mathf.Abs(polyOld))
                {
                    break;
                }

                var dy = a1 + xNew * (a22 + xNew * a33);
                if (Mathf.Abs(dy) < SingularEpsilon)
                {
                    break;
                }

                var xOld = xNew;
                xNew = xOld - polyValue / dy;
                if (xNew != 0f && Mathf.Abs((xNew - xOld) / xNew) < 1e-12f)
                {
                    break;
                }

                if (xNew < 0f)
                {
                    xNew = 0f;
                }
            }

            var det = xNew * xNew - xNew * mz + covXy;
            if (Mathf.Abs(det) < SingularEpsilon)
            {
                return false;
            }

            var xcRel = (mxz * (myy - xNew) - myz * mxy) / det * 0.5f;
            var ycRel = (myz * (mxx - xNew) - mxz * mxy) / det * 0.5f;
            var xc = xcRel + meanX;
            var yc = ycRel + meanY;
            var radius = ComputeMeanDistance(xs, ys, xc, yc);
            if (radius <= DistanceEpsilon || float.IsNaN(radius) || float.IsInfinity(radius))
            {
                return false;
            }

            result = (xc, yc, radius);
            return true;
        }

        static float ComputeMeanDistance(float[] xs, float[] ys, float xc, float yc)
        {
            var sum = 0f;
            for (var i = 0; i < xs.Length; i++)
            {
                var dx = xs[i] - xc;
                var dy = ys[i] - yc;
                sum += Mathf.Sqrt(dx * dx + dy * dy);
            }

            return sum / xs.Length;
        }
    }
}
