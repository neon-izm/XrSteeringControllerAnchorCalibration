using System;
using System.Collections.Generic;
using UnityEngine;

namespace XrSteeringControllerAnchorCalibration
{
    [ExecuteAlways]
    public class SteeringAnchorCalibrationSample : MonoBehaviour
    {
        const float CgHandleAxisLength = 0.08f;

        static readonly Color SamplePointColor = new Color(1f, 0.4f, 0.1f, 1f);
        static readonly Color InlierPointColor = new Color(0.2f, 0.9f, 0.3f, 1f);
        static readonly Color OutlierPointColor = new Color(0.9f, 0.2f, 0.2f, 1f);
        static readonly Color SourceArcColor = new Color(0.7f, 0.7f, 0.7f, 1f);
        static readonly Color EstimatedCircleColor = new Color(0.2f, 0.8f, 1f, 1f);
        static readonly Color MappedCircleColor = new Color(0.8f, 0.4f, 1f, 1f);
        static readonly Color MappedPointColor = new Color(0.95f, 0.5f, 1f, 1f);

        [Header("References")]
        [SerializeField] Transform cgHandleTransform;
        [SerializeField] Transform worldArcCenter;
        [SerializeField] float worldArcRadius = 0.15f;

        [Header("Sample Generation")]
        [SerializeField] ArcPointGenerator.Settings generationSettings = ArcPointGenerator.Settings.Default;
        [SerializeField] bool useRandomWorldArcOnGenerate;

        [Header("Calibration")]
        [SerializeField] float calibrationThreshold = CircleFitting3D.DefaultThreshold;
        [SerializeField] int calibrationMaxIterations;

        [HideInInspector] [SerializeField] List<Vector3> sampleWorldPoints = new List<Vector3>();
        [HideInInspector] [SerializeField] bool hasCalibrationResult;
        [HideInInspector] [SerializeField] Vector3 estimatedCenterWorld;
        [HideInInspector] [SerializeField] Quaternion estimatedRotationWorld;
        [HideInInspector] [SerializeField] float estimatedRadiusWorld;
        [HideInInspector] [SerializeField] Matrix4x4 storedModelView;
        [HideInInspector] [SerializeField] float mappedCenterErrorMeters;
        [HideInInspector] [SerializeField] float mappedInliersMaxErrorMeters;
        [HideInInspector] [SerializeField] int lastInlierCount;
        [HideInInspector] [SerializeField] string lastError;

        const int CircleSegments = 72;
        const float PointGizmoSize = 0.006f;
        const float MappedPointGizmoSize = 0.004f;

        int[] lastInlierIndices = Array.Empty<int>();

        public Transform CgHandleTransform => cgHandleTransform;
        public Transform WorldArcCenter => worldArcCenter;
        public IReadOnlyList<Vector3> SampleWorldPoints => sampleWorldPoints;
        public bool HasCalibrationResult => hasCalibrationResult;
        public string LastError => lastError;
        public int LastInlierCount => lastInlierCount;
        public float MappedCenterErrorMeters => mappedCenterErrorMeters;
        public float MappedInliersMaxErrorMeters => mappedInliersMaxErrorMeters;
        public Matrix4x4 ModelView => storedModelView;

        public CgHandlePose GetTargetHandlePose()
        {
            if (cgHandleTransform == null)
            {
                throw new InvalidOperationException("CG handle transform is not assigned.");
            }

            return CgHandlePose.FromTransform(cgHandleTransform);
        }

        public void GenerateSamplePoints()
        {
            lastError = string.Empty;

            ArcPointGenerator.GeneratedArc generated;
            if (useRandomWorldArcOnGenerate)
            {
                generated = ArcPointGenerator.GenerateRandom(generationSettings);
            }
            else
            {
                var centerTransform = worldArcCenter != null ? worldArcCenter : transform;
                generated = ArcPointGenerator.Generate(
                    centerTransform.position,
                    centerTransform.rotation,
                    worldArcRadius,
                    generationSettings);
            }

            sampleWorldPoints.Clear();
            sampleWorldPoints.AddRange(generated.Points);
            ClearCalibrationResult();
        }

        public void RunCalibration()
        {
            lastError = string.Empty;
            ClearCalibrationResult();

            if (sampleWorldPoints.Count < 3)
            {
                lastError = "At least 3 sample points are required.";
                return;
            }

            if (cgHandleTransform == null)
            {
                lastError = "CG handle transform is not assigned.";
                return;
            }

            try
            {
                var targetHandle = GetTargetHandlePose();
                var fit = CircleFitting3D.FitCircleMsac(
                    sampleWorldPoints,
                    calibrationThreshold,
                    calibrationMaxIterations);

                hasCalibrationResult = true;
                lastInlierIndices = fit.InlierIndices;
                lastInlierCount = fit.InlierIndices.Length;
                estimatedCenterWorld = fit.Circle.Position;
                estimatedRotationWorld = fit.Circle.Rotation;
                estimatedRadiusWorld = fit.Circle.Radius;
                storedModelView = AnchorCalibration.ComputeModelView(fit.Circle, targetHandle);
                UpdateAlignmentMetrics(storedModelView, targetHandle);
            }
            catch (Exception exception)
            {
                lastError = exception.Message;
            }
        }

        public void ClearSamplePoints()
        {
            sampleWorldPoints.Clear();
            ClearCalibrationResult();
            lastError = string.Empty;
        }

        public Circle3D GetEstimatedCircleWorld()
        {
            return new Circle3D(estimatedCenterWorld, estimatedRotationWorld, estimatedRadiusWorld);
        }

        public Matrix4x4 GetModelView()
        {
            return storedModelView;
        }

        void OnDrawGizmos()
        {
            DrawCgHandleAxes();
            DrawSourceArc();
            DrawSamplePoints();

            if (!hasCalibrationResult)
            {
                return;
            }

            var modelView = GetModelView();
            var estimatedWorld = GetEstimatedCircleWorld();
            var aligned = AnchorCalibration.AlignNormalToTargetForward(
                estimatedWorld,
                GetTargetHandlePose().Forward);

            CircleGizmoDrawer.DrawCircle(estimatedWorld, EstimatedCircleColor, CircleSegments);

            var mappedCircle = AnchorCalibration.TransformCircleRigid(modelView, aligned);
            CircleGizmoDrawer.DrawCircle(mappedCircle, MappedCircleColor, CircleSegments);
            DrawMappedSamplePoints(modelView);
        }

        void DrawCgHandleAxes()
        {
            if (cgHandleTransform == null)
            {
                return;
            }

            CircleGizmoDrawer.DrawPoseAxes(
                cgHandleTransform.position,
                cgHandleTransform.rotation,
                CgHandleAxisLength);
        }

        void DrawSourceArc()
        {
            if (useRandomWorldArcOnGenerate || sampleWorldPoints.Count == 0)
            {
                return;
            }

            var centerTransform = worldArcCenter != null ? worldArcCenter : transform;
            var source = Circle3D.FromPose(centerTransform.position, centerTransform.rotation, worldArcRadius);
            CircleGizmoDrawer.DrawArc(source, SourceArcColor, 90f, CircleSegments);
            CircleGizmoDrawer.DrawCircle(
                source,
                new Color(SourceArcColor.r, SourceArcColor.g, SourceArcColor.b, 0.35f),
                CircleSegments);
        }

        void DrawSamplePoints()
        {
            if (sampleWorldPoints.Count == 0)
            {
                return;
            }

            var inlierSet = hasCalibrationResult
                ? new HashSet<int>(lastInlierIndices)
                : null;

            for (var i = 0; i < sampleWorldPoints.Count; i++)
            {
                var color = SamplePointColor;
                if (inlierSet != null)
                {
                    color = inlierSet.Contains(i) ? InlierPointColor : OutlierPointColor;
                }

                Gizmos.color = color;
                Gizmos.DrawSphere(sampleWorldPoints[i], PointGizmoSize);
            }
        }

        void DrawMappedSamplePoints(Matrix4x4 modelView)
        {
            var inlierSet = new HashSet<int>(lastInlierIndices);

            for (var i = 0; i < sampleWorldPoints.Count; i++)
            {
                if (!inlierSet.Contains(i))
                {
                    continue;
                }

                var mapped = AnchorCalibration.WorldToCg(sampleWorldPoints[i], modelView);
                Gizmos.color = MappedPointColor;
                Gizmos.DrawSphere(mapped, MappedPointGizmoSize);
            }
        }

        void UpdateAlignmentMetrics(Matrix4x4 modelView, CgHandlePose targetHandle)
        {
            var estimatedWorld = GetEstimatedCircleWorld();
            var aligned = AnchorCalibration.AlignNormalToTargetForward(estimatedWorld, targetHandle.Forward);
            var mappedCircle = AnchorCalibration.TransformCircleRigid(modelView, aligned);
            mappedCenterErrorMeters = Vector3.Distance(mappedCircle.Position, targetHandle.Position);

            var maxInlierError = 0f;
            var inlierSet = new HashSet<int>(lastInlierIndices);
            for (var i = 0; i < sampleWorldPoints.Count; i++)
            {
                if (!inlierSet.Contains(i))
                {
                    continue;
                }

                var mapped = AnchorCalibration.WorldToCg(sampleWorldPoints[i], modelView);
                var error = CircleFitting3D.DistanceToCircle(mapped, mappedCircle);
                maxInlierError = Mathf.Max(maxInlierError, error);
            }

            mappedInliersMaxErrorMeters = maxInlierError;
        }

        void ClearCalibrationResult()
        {
            hasCalibrationResult = false;
            lastInlierIndices = Array.Empty<int>();
            lastInlierCount = 0;
            estimatedCenterWorld = Vector3.zero;
            estimatedRotationWorld = Quaternion.identity;
            estimatedRadiusWorld = 0f;
            storedModelView = Matrix4x4.identity;
            mappedCenterErrorMeters = 0f;
            mappedInliersMaxErrorMeters = 0f;
        }
    }
}
