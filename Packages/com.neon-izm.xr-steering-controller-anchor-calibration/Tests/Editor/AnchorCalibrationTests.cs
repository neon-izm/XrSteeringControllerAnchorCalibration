using System;
using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public class AnchorCalibrationTests
    {
        const float CenterTolerance = 0.001f;
        const float AngleToleranceDeg = 1f;
        const float RollToleranceDeg = 5f;

        [Test]
        public void StoredModelView_RoundTrip_PreservesRigidTransform()
        {
            var estimatedWorld = Circle3D.FromPose(Vector3.one, Quaternion.Euler(12f, 34f, 0f), 0.14f);
            var targetHandle = new CgHandlePose(new Vector3(2f, 0.5f, -1f), Quaternion.Euler(-8f, 60f, 0f));
            var modelView = AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);

            var reconstructed = new Matrix4x4();
            reconstructed.SetRow(0, modelView.GetRow(0));
            reconstructed.SetRow(1, modelView.GetRow(1));
            reconstructed.SetRow(2, modelView.GetRow(2));
            reconstructed.SetRow(3, modelView.GetRow(3));

            var mappedOriginal = AnchorCalibration.TransformCircleRigid(modelView, estimatedWorld);
            var mappedReconstructed = AnchorCalibration.TransformCircleRigid(reconstructed, estimatedWorld);

            Assert.That(Vector3.Distance(mappedOriginal.Position, mappedReconstructed.Position), Is.LessThan(1e-4f));
            Assert.That(Vector3.Distance(mappedOriginal.Position, targetHandle.Position), Is.LessThan(CenterTolerance));
        }

        [Test]
        public void UserForward_FromHeadToHands_PointsTowardHandle()
        {
            var handleCenter = new Vector3(0f, 1f, 0.5f);
            var head = new Vector3(0f, 1.2f, -0.2f);
            var points = new[]
            {
                handleCenter + new Vector3(0.1f, 0f, 0f),
                handleCenter + new Vector3(-0.05f, 0.02f, 0.03f),
                handleCenter + new Vector3(0.02f, -0.01f, 0.05f),
            };

            var userForward = AnchorCalibration.EstimateUserForward(head, points);
            var toHandle = (handleCenter - head).normalized;

            Assert.That(Vector3.Angle(userForward, toHandle), Is.LessThan(15f));
        }

        [Test]
        public void ComputeModelView_MapsEstimatedWorldCircleToCgTargetPose()
        {
            var estimatedWorld = Circle3D.FromPose(
                new Vector3(0.2f, 0.1f, -0.3f),
                Quaternion.Euler(20f, 35f, 0f),
                0.14f);

            var targetHandle = new CgHandlePose(
                new Vector3(1.1f, 0.8f, 0.4f),
                Quaternion.Euler(-15f, 70f, 0f));

            var modelView = AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);
            var mapped = AnchorCalibration.TransformCircleRigid(modelView, estimatedWorld);

            Assert.That(Vector3.Distance(mapped.Position, targetHandle.Position), Is.LessThan(CenterTolerance));
            Assert.That(Vector3.Angle(mapped.Normal, targetHandle.Forward), Is.LessThan(AngleToleranceDeg));
            Assert.That(mapped.Radius, Is.EqualTo(estimatedWorld.Radius).Within(1e-5f));
        }

        [Test]
        public void TransformWorldPointsToCg_AppliesCalibratedModelView()
        {
            var estimatedWorld = Circle3D.FromPose(Vector3.one, Quaternion.Euler(0f, 45f, 0f), 0.15f);
            var targetHandle = new CgHandlePose(new Vector3(2f, 0f, 1f), Quaternion.Euler(0f, 90f, 0f));
            var modelView = AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);

            var worldPoints = new[] { Vector3.zero, Vector3.right };
            var cgPoints = AnchorCalibration.TransformWorldPointsToCg(worldPoints, modelView);

            Assert.That(Vector3.Distance(cgPoints[0], AnchorCalibration.WorldToCg(worldPoints[0], modelView)), Is.LessThan(1e-5f));
            Assert.That(Vector3.Distance(cgPoints[1], AnchorCalibration.WorldToCg(worldPoints[1], modelView)), Is.LessThan(1e-5f));
        }

        [Test]
        public void Calibrate_ReturnsOnlyModelView()
        {
            var data = CreateDriverSeatScenario(seed: 99, out var targetHandle, out var headPose);

            var result = AnchorCalibration.Calibrate(data.Points, targetHandle, headPose, random: new System.Random(4));

            Assert.That(result.ModelView, Is.Not.EqualTo(Matrix4x4.zero));
        }

        [Test]
        public void Calibrate_WithHeadBehindHandle_PlacesHeadInDriverSeat()
        {
            var data = CreateDriverSeatScenario(seed: 99, out var targetHandle, out var headPose);

            var result = AnchorCalibration.Calibrate(data.Points, targetHandle, headPose, random: new System.Random(4));

            Assert.That(
                AnchorCalibration.IsOnDriverSeatSide(result.ModelView, headPose, targetHandle),
                Is.True);
            Assert.That(
                AnchorCalibration.ScoreDriverSeat(result.ModelView, headPose, targetHandle),
                Is.GreaterThan(0f));
        }

        [Test]
        public void Calibrate_FlippedNormalCandidate_IsRejected()
        {
            var data = CreateDriverSeatScenario(seed: 99, out var targetHandle, out var headPose);
            var fit = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(4));

            var forcedFlipped = AnchorCalibration.ComputeCalibratedModelView(
                fit.Circle,
                targetHandle,
                headPose,
                forceFlippedCandidate: true);
            var autoSelected = AnchorCalibration.ComputeCalibratedModelView(
                fit.Circle,
                targetHandle,
                headPose,
                forceFlippedCandidate: false);

            Assert.That(
                AnchorCalibration.IsOnDriverSeatSide(forcedFlipped, headPose, targetHandle),
                Is.False);
            Assert.That(
                AnchorCalibration.IsOnDriverSeatSide(autoSelected, headPose, targetHandle),
                Is.True);
        }

        [Test]
        public void Calibrate_RepeatedRuns_FrontBackIsStable()
        {
            var data = CreateDriverSeatScenario(seed: 42, out var targetHandle, out var headPose);
            var first = AnchorCalibration.Calibrate(data.Points, targetHandle, headPose, random: new System.Random(1));
            var second = AnchorCalibration.Calibrate(data.Points, targetHandle, headPose, random: new System.Random(99));

            var firstOnDriver = AnchorCalibration.IsOnDriverSeatSide(first.ModelView, headPose, targetHandle);
            var secondOnDriver = AnchorCalibration.IsOnDriverSeatSide(second.ModelView, headPose, targetHandle);

            Assert.That(firstOnDriver, Is.True);
            Assert.That(secondOnDriver, Is.True);
        }

        [Test]
        public void RemoveHandleLocalRoll_ZerosRollAboutHandleAxis()
        {
            var estimatedWorld = Circle3D.FromPose(Vector3.zero, Quaternion.Euler(10f, 20f, 0f), 0.15f);
            var targetHandle = new CgHandlePose(Vector3.one, Quaternion.Euler(5f, 30f, 0f));
            var baseModelView = AnchorCalibration.ComputeModelView(estimatedWorld, targetHandle);
            var twistedRotation = Quaternion.AngleAxis(25f, targetHandle.Forward) * baseModelView.rotation;
            var twisted = Matrix4x4.TRS((Vector3)baseModelView.GetColumn(3), twistedRotation, Vector3.one);
            var rollBefore = AnchorCalibration.GetHandleLocalRollDeg(twisted, estimatedWorld, targetHandle);

            var corrected = AnchorCalibration.RemoveHandleLocalRoll(twisted, estimatedWorld, targetHandle);
            var rollAfter = AnchorCalibration.GetHandleLocalRollDeg(corrected, estimatedWorld, targetHandle);

            Assert.That(Mathf.Abs(rollBefore), Is.GreaterThan(10f));
            Assert.That(Mathf.Abs(rollAfter), Is.LessThan(RollToleranceDeg));
        }

        [Test]
        public void Calibrate_WithHeadPose_RollWithinTolerance()
        {
            var data = CreateDriverSeatScenario(seed: 17, out var targetHandle, out var headPose);
            var fit = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(2));
            var result = AnchorCalibration.Calibrate(data.Points, targetHandle, headPose, random: new System.Random(2));
            AnchorCalibration.ComputeCalibratedModelView(
                fit.Circle,
                targetHandle,
                headPose,
                false,
                out var orientedCircleUsed);
            var roll = AnchorCalibration.GetHandleLocalRollDeg(result.ModelView, orientedCircleUsed, targetHandle);

            Assert.That(Mathf.Abs(roll), Is.LessThan(RollToleranceDeg));
        }

        [Test]
        public void Calibrate_PitchedAndRolledHeadRotation_DoesNotChangeRollPhase()
        {
            var data = CreateDriverSeatScenario(seed: 21, out var targetHandle, out var levelHead);
            var tiltedHead = new HeadPose(
                levelHead.Position,
                Quaternion.Euler(35f, 0f, 20f) * levelHead.Rotation);

            var resultLevel = AnchorCalibration.Calibrate(
                data.Points,
                targetHandle,
                levelHead,
                random: new System.Random(3));
            var resultTilted = AnchorCalibration.Calibrate(
                data.Points,
                targetHandle,
                tiltedHead,
                random: new System.Random(3));

            var fit = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(3));
            AnchorCalibration.ComputeCalibratedModelView(fit.Circle, targetHandle, levelHead, false, out var orientedLevel);
            AnchorCalibration.ComputeCalibratedModelView(fit.Circle, targetHandle, tiltedHead, false, out var orientedTilted);

            Assert.That(
                Quaternion.Angle(orientedLevel.Rotation, orientedTilted.Rotation),
                Is.LessThan(1e-3f));
            Assert.That(
                Quaternion.Angle(resultLevel.ModelView.rotation, resultTilted.ModelView.rotation),
                Is.LessThan(1e-3f));
        }

        [Test]
        public void OrientCircleWithWorldUp_UsesWorldUpNotHeadForward()
        {
            var normal = new Vector3(0.2f, 0.1f, 1f).normalized;
            var raw = Circle3D.FromPose(new Vector3(0.1f, 1f, 0.3f), CircleFitting3D.RotationFromNormal(normal), 0.15f);
            var oriented = AnchorCalibration.OrientCircleWithWorldUp(raw);

            var upOnPlane = Vector3.ProjectOnPlane(Vector3.up, oriented.Normal).normalized;
            var circleUp = (oriented.Rotation * Vector3.up).normalized;
            Assert.That(Vector3.Angle(circleUp, upOnPlane), Is.LessThan(1e-2f));
            Assert.That(Mathf.Abs(SignedRollAroundForward(oriented.Rotation)), Is.LessThan(1e-2f));
        }

        [Test]
        public void Calibrate_ArcPhaseAmbiguity_DoesNotProduceLargeRoll()
        {
            var center = new Vector3(0.1f, 1f, 0.2f);
            var rotation = Quaternion.Euler(25f, 40f, 0f);
            var targetHandle = new CgHandlePose(new Vector3(0.5f, 1.2f, -0.2f), Quaternion.Euler(10f, 55f, 0f));
            var headPose = CreateHeadBehindHandle(center, rotation);

            var settingsA = ArcGenerationSettings.Default;
            settingsA.RandomSeed = 7;
            var arcA = ArcPointGenerator.Generate(center, rotation, 0.15f, settingsA);

            var settingsB = settingsA;
            settingsB.RandomSeed = 13;
            var arcB = ArcPointGenerator.Generate(center, rotation, 0.15f, settingsB);

            var resultA = AnchorCalibration.Calibrate(arcA.Points, targetHandle, headPose, random: new System.Random(0));
            var resultB = AnchorCalibration.Calibrate(arcB.Points, targetHandle, headPose, random: new System.Random(0));

            var fitA = CircleFitting3D.FitCircleMsac(arcA.Points, random: new System.Random(0));
            var fitB = CircleFitting3D.FitCircleMsac(arcB.Points, random: new System.Random(0));
            AnchorCalibration.ComputeCalibratedModelView(fitA.Circle, targetHandle, headPose, false, out var orientedA);
            AnchorCalibration.ComputeCalibratedModelView(fitB.Circle, targetHandle, headPose, false, out var orientedB);

            var rollA = AnchorCalibration.GetHandleLocalRollDeg(resultA.ModelView, orientedA, targetHandle);
            var rollB = AnchorCalibration.GetHandleLocalRollDeg(resultB.ModelView, orientedB, targetHandle);

            Assert.That(Mathf.Abs(rollA), Is.LessThan(RollToleranceDeg));
            Assert.That(Mathf.Abs(rollB), Is.LessThan(RollToleranceDeg));
        }

        [Test]
        public void EndToEnd_GenerateAndCalibrate_WithoutKnownCgRadius()
        {
            var data = CreateDriverSeatScenario(seed: 99, out var targetHandle, out var headPose);
            var fit = CircleFitting3D.FitCircleMsac(data.Points, random: new System.Random(4));
            var result = AnchorCalibration.Calibrate(data.Points, targetHandle, headPose, random: new System.Random(4));
            AnchorCalibration.ComputeCalibratedModelView(fit.Circle, targetHandle, headPose, false, out var oriented);
            var mapped = AnchorCalibration.TransformCircleRigid(result.ModelView, oriented);

            Assert.That(Vector3.Distance(mapped.Position, targetHandle.Position), Is.LessThan(0.01f));
            Assert.That(Vector3.Angle(mapped.Normal, targetHandle.Forward), Is.LessThan(3f));
            Assert.That(Mathf.Abs(mapped.Radius - data.GroundTruth.Radius), Is.LessThan(0.01f));
            Assert.That(AnchorCalibration.IsOnDriverSeatSide(result.ModelView, headPose, targetHandle), Is.True);
        }

        static TestDataGenerator.GeneratedArcData CreateDriverSeatScenario(
            int seed,
            out CgHandlePose targetHandle,
            out HeadPose headPose)
        {
            var data = TestDataGenerator.GenerateArc(pointCount: 150, outlierRatio: 0.05f, seed: seed);
            targetHandle = new CgHandlePose(
                new Vector3(0.5f, 1.2f, -0.2f),
                Quaternion.Euler(10f, 55f, 0f));
            headPose = CreateHeadBehindHandle(data.GroundTruth.Center, data.GroundTruth.Rotation);
            return data;
        }

        static HeadPose CreateHeadBehindHandle(Vector3 handleCenter, Quaternion handleRotation)
        {
            var backOffset = -(handleRotation * Vector3.forward) * 0.35f;
            var headPosition = handleCenter + backOffset + Vector3.up * 0.08f;
            var headRotation = Quaternion.LookRotation(handleRotation * Vector3.forward, Vector3.up);
            return new HeadPose(headPosition, headRotation);
        }

        static float SignedRollAroundForward(Quaternion rotation)
        {
            var forward = rotation * Vector3.forward;
            if (forward.sqrMagnitude < 1e-8f)
            {
                return 0f;
            }

            var up = rotation * Vector3.up;
            var referenceUp = Vector3.ProjectOnPlane(Vector3.up, forward);
            var projectedUp = Vector3.ProjectOnPlane(up, forward);
            if (referenceUp.sqrMagnitude < 1e-8f || projectedUp.sqrMagnitude < 1e-8f)
            {
                return 0f;
            }

            return Vector3.SignedAngle(referenceUp, projectedUp, forward);
        }
    }
}
