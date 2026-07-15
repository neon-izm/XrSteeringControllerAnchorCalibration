using System;
using System.Collections.Generic;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Sample
{
    [ExecuteAlways]
    public class SteeringAnchorCalibrationSample : MonoBehaviour
    {
        const float CgHandleAxisLength = 0.08f;
        const float UserForwardArrowLength = 0.25f;

        static readonly Color SamplePointColor = new Color(1f, 0.4f, 0.1f, 1f);
        static readonly Color InlierPointColor = new Color(0.2f, 0.9f, 0.3f, 1f);
        static readonly Color OutlierPointColor = new Color(0.9f, 0.2f, 0.2f, 1f);
        static readonly Color SourceArcColor = new Color(0.7f, 0.7f, 0.7f, 1f);
        static readonly Color EstimatedCircleColor = new Color(0.2f, 0.8f, 1f, 1f);
        static readonly Color MappedCircleColor = new Color(0.8f, 0.4f, 1f, 1f);
        static readonly Color MappedPointColor = new Color(0.95f, 0.5f, 1f, 1f);
        static readonly Color HeadColor = new Color(0.3f, 0.85f, 1f, 1f);
        static readonly Color MappedHeadColor = new Color(1f, 0.85f, 0.2f, 1f);
        static readonly Color UserForwardColor = new Color(1f, 0.95f, 0.2f, 1f);

        [Header("References")]
        [SerializeField] Transform cgHandleTransform;
        [SerializeField] Transform headPoseTransform;
        [SerializeField] Transform worldArcCenter;
        [SerializeField] float worldArcRadius = 0.15f;

        [Header("Sample Generation")]
        [SerializeField] ArcGenerationSettings generationSettings = ArcGenerationSettings.Default;
        [SerializeField] bool useRandomWorldArcOnGenerate;
        [SerializeField] bool placeHeadOnBackSideOnGenerate = true;

        [Header("Calibration")]
        [SerializeField] float calibrationThreshold = CircleFitting3D.DefaultThreshold;
        [SerializeField] int calibrationMaxIterations;
        [SerializeField] bool forceFlippedNormal;

        [HideInInspector] [SerializeField] List<Vector3> sampleWorldPoints = new List<Vector3>();
        [HideInInspector] [SerializeField] bool hasCalibrationResult;
        [HideInInspector] [SerializeField] Vector3 estimatedCenterWorld;
        [HideInInspector] [SerializeField] Quaternion estimatedRotationWorld;
        [HideInInspector] [SerializeField] float estimatedRadiusWorld;
        [HideInInspector] [SerializeField] Matrix4x4 storedModelView;
        [HideInInspector] [SerializeField] float mappedCenterErrorMeters;
        [HideInInspector] [SerializeField] float mappedInliersMaxErrorMeters;
        [HideInInspector] [SerializeField] int lastInlierCount;
        [HideInInspector] [SerializeField] float frontBackScore;
        [HideInInspector] [SerializeField] float handleLocalRollDeg;
        [HideInInspector] [SerializeField] bool isOnDriverSeatSide;
        [HideInInspector] [SerializeField] string lastError;

        const int CircleSegments = 72;
        const float PointGizmoSize = 0.006f;
        const float MappedPointGizmoSize = 0.004f;
        const float HeadGizmoSize = 0.025f;

        int[] lastInlierIndices = Array.Empty<int>();
        Circle3D lastOrientedCircle;

        public Transform CgHandleTransform => cgHandleTransform;
        public Transform HeadPoseTransform => headPoseTransform;
        public Transform WorldArcCenter => worldArcCenter;
        public IReadOnlyList<Vector3> SampleWorldPoints => sampleWorldPoints;
        public bool HasCalibrationResult => hasCalibrationResult;
        public string LastError => lastError;
        public int LastInlierCount => lastInlierCount;
        public float MappedCenterErrorMeters => mappedCenterErrorMeters;
        public float MappedInliersMaxErrorMeters => mappedInliersMaxErrorMeters;
        public float FrontBackScore => frontBackScore;
        public float HandleLocalRollDeg => handleLocalRollDeg;
        public bool IsOnDriverSeatSide => isOnDriverSeatSide;
        public Matrix4x4 ModelView => storedModelView;
        public bool ForceFlippedNormal => forceFlippedNormal;
        public float ArcHalfAngleDeg => generationSettings.ArcHalfAngleDeg;

        internal ArcGenerationSettings GenerationSettings => generationSettings;
        internal bool UseRandomWorldArcOnGenerate => useRandomWorldArcOnGenerate;
        internal bool PlaceHeadOnBackSideOnGenerate => placeHeadOnBackSideOnGenerate;
        internal float WorldArcRadiusValue => worldArcRadius;
        internal Transform ArcCenterTransform => worldArcCenter != null ? worldArcCenter : transform;

        internal void ReplaceSampleWorldPoints(IReadOnlyList<Vector3> points)
        {
            lastError = string.Empty;
            sampleWorldPoints.Clear();
            sampleWorldPoints.AddRange(points);
            ClearCalibrationResult();
        }

        public CgHandlePose GetTargetHandlePose()
        {
            if (cgHandleTransform == null)
            {
                throw new InvalidOperationException("CG handle transform is not assigned.");
            }

            return CgHandlePose.FromTransform(cgHandleTransform);
        }

        public HeadPose GetHeadPose()
        {
            if (headPoseTransform == null)
            {
                throw new InvalidOperationException("Head pose transform is not assigned.");
            }

            return HeadPose.FromTransform(headPoseTransform);
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

            if (headPoseTransform == null)
            {
                lastError = "Head pose transform is not assigned.";
                return;
            }

            try
            {
                var targetHandle = GetTargetHandlePose();
                var headPose = GetHeadPose();
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
                lastOrientedCircle = AnchorCalibration.OrientCircleWithWorldUp(fit.Circle);
                storedModelView = AnchorCalibration.ComputeCalibratedModelView(
                    fit.Circle,
                    targetHandle,
                    headPose,
                    forceFlippedNormal,
                    out lastOrientedCircle);
                UpdateAlignmentMetrics(storedModelView, targetHandle, headPose, lastOrientedCircle);
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

        public Circle3D GetOrientedCircleWorld()
        {
            return hasCalibrationResult ? lastOrientedCircle : GetEstimatedCircleWorld();
        }

        public Matrix4x4 GetModelView()
        {
            return storedModelView;
        }

        public Vector3 GetUserForwardWorld()
        {
            if (sampleWorldPoints.Count == 0 || headPoseTransform == null)
            {
                return Vector3.zero;
            }

            return AnchorCalibration.EstimateUserForward(
                headPoseTransform.position,
                sampleWorldPoints);
        }

        void OnDrawGizmos()
        {
            DrawCgHandleAxes();
            DrawHeadPose();
            DrawUserForward();
            DrawSourceArc();
            DrawSamplePoints();

            if (!hasCalibrationResult)
            {
                return;
            }

            var modelView = GetModelView();
            var estimatedWorld = GetEstimatedCircleWorld();
            var orientedWorld = GetOrientedCircleWorld();

            CircleGizmoDrawer.DrawCircle(estimatedWorld, EstimatedCircleColor, CircleSegments);

            var mappedCircle = AnchorCalibration.TransformCircleRigid(modelView, orientedWorld);
            CircleGizmoDrawer.DrawCircle(mappedCircle, MappedCircleColor, CircleSegments);
            DrawMappedHead(modelView);
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

        void DrawHeadPose()
        {
            if (headPoseTransform == null)
            {
                return;
            }

            Gizmos.color = HeadColor;
            Gizmos.DrawSphere(headPoseTransform.position, HeadGizmoSize);
            CircleGizmoDrawer.DrawPoseAxes(
                headPoseTransform.position,
                headPoseTransform.rotation,
                CgHandleAxisLength * 0.6f);
        }

        void DrawUserForward()
        {
            if (headPoseTransform == null || sampleWorldPoints.Count == 0)
            {
                return;
            }

            try
            {
                var userForward = GetUserForwardWorld();
                Gizmos.color = UserForwardColor;
                Gizmos.DrawLine(
                    headPoseTransform.position,
                    headPoseTransform.position + userForward * UserForwardArrowLength);
            }
            catch (InvalidOperationException)
            {
            }
        }

        void DrawMappedHead(Matrix4x4 modelView)
        {
            if (headPoseTransform == null)
            {
                return;
            }

            var mappedHead = AnchorCalibration.WorldToCg(headPoseTransform.position, modelView);
            Gizmos.color = MappedHeadColor;
            Gizmos.DrawSphere(mappedHead, HeadGizmoSize * 0.7f);
        }

        void DrawSourceArc()
        {
            if (useRandomWorldArcOnGenerate || sampleWorldPoints.Count == 0)
            {
                return;
            }

            var centerTransform = worldArcCenter != null ? worldArcCenter : transform;
            var source = Circle3D.FromPose(centerTransform.position, centerTransform.rotation, worldArcRadius);
            var totalArcDeg = generationSettings.ArcHalfAngleDeg > 0f
                ? generationSettings.ArcHalfAngleDeg * 2f
                : 90f;
            CircleGizmoDrawer.DrawArc(source, SourceArcColor, totalArcDeg, CircleSegments);
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

        void UpdateAlignmentMetrics(
            Matrix4x4 modelView,
            CgHandlePose targetHandle,
            HeadPose headPose,
            Circle3D orientedWorld)
        {
            var mappedCircle = AnchorCalibration.TransformCircleRigid(modelView, orientedWorld);
            mappedCenterErrorMeters = Vector3.Distance(mappedCircle.Position, targetHandle.Position);
            frontBackScore = AnchorCalibration.ScoreDriverSeat(modelView, headPose, targetHandle);
            handleLocalRollDeg = AnchorCalibration.GetHandleLocalRollDeg(modelView, orientedWorld, targetHandle);
            isOnDriverSeatSide = AnchorCalibration.IsOnDriverSeatSide(modelView, headPose, targetHandle);

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
            frontBackScore = 0f;
            handleLocalRollDeg = 0f;
            isOnDriverSeatSide = false;
            lastOrientedCircle = default;
        }
    }
}
