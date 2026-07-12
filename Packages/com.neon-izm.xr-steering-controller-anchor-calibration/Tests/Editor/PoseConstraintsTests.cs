using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    public class PoseConstraintsTests
    {
        const float PivotTolerance = 1e-4f;
        const float AngleToleranceDeg = 1e-3f;

        [Test]
        public void RemoveRoll_PreservesForward_AndClearsWorldRoll()
        {
            var rotation = Quaternion.Euler(18f, -35f, 6f);

            var constrained = PoseConstraints.RemoveRoll(rotation, Vector3.up);

            var forwardBefore = rotation * Vector3.forward;
            var forwardAfter = constrained * Vector3.forward;
            Assert.That(Vector3.Angle(forwardBefore, forwardAfter), Is.LessThan(AngleToleranceDeg));
            Assert.That(Mathf.Abs(SignedRollAroundForward(rotation)), Is.GreaterThan(1f));
            Assert.That(Mathf.Abs(SignedRollAroundForward(constrained)), Is.LessThan(AngleToleranceDeg));
        }

        [Test]
        public void ConstrainPosePreservingWorldPoint_KeepsPivot_RemoveRoll()
        {
            var pivot = new Vector3(0.4f, 1.1f, 0.55f);
            var position = new Vector3(0f, 1.6f, 0f);
            var rotation = Quaternion.Euler(18f, -35f, 6f);

            PoseConstraints.ConstrainPosePreservingWorldPoint(
                position,
                rotation,
                TrackingHorizonConstraint.RemoveRoll,
                Vector3.up,
                pivot,
                out var constrainedPosition,
                out var constrainedRotation);

            AssertPivotPreserved(position, rotation, constrainedPosition, constrainedRotation, pivot);
        }

        [Test]
        public void ComputeTrackingOriginPose_RemoveRoll_PreservesHandleWorldPosition()
        {
            var pivot = new Vector3(0.35f, 1.05f, 0.5f);
            var proposedPosition = new Vector3(0f, 1.55f, 0.05f);
            var proposedRotation = Quaternion.Euler(12f, 40f, 8f);

            PoseConstraints.ComputeTrackingOriginPose(
                proposedPosition,
                proposedRotation,
                pivot,
                CalibrationOptions.Default,
                out var originPosition,
                out var originRotation);

            AssertPivotPreserved(proposedPosition, proposedRotation, originPosition, originRotation, pivot);
        }

        [Test]
        public void RemovePitch_FlattensForward_PreservesYawSense()
        {
            var rotation = Quaternion.Euler(25f, 50f, 10f);
            var up = Vector3.up;

            var constrained = PoseConstraints.RemovePitch(rotation, up);
            var flatForward = Vector3.ProjectOnPlane(constrained * Vector3.forward, up);

            Assert.That(flatForward.sqrMagnitude, Is.GreaterThan(0.5f));
            Assert.That(Mathf.Abs(Vector3.Angle(constrained * Vector3.forward, flatForward)), Is.LessThan(AngleToleranceDeg));

            var yawBefore = SignedYawAroundUp(rotation, up);
            var yawAfter = SignedYawAroundUp(constrained, up);
            Assert.That(Mathf.Abs(Mathf.DeltaAngle(yawBefore, yawAfter)), Is.LessThan(5f));
        }

        [Test]
        public void RemovePitchAndRoll_ResultsInHorizontalForward_AndZeroRoll()
        {
            var rotation = Quaternion.Euler(25f, 50f, 10f);

            var constrained = PoseConstraints.RemovePitchAndRoll(rotation, Vector3.up);

            var forward = constrained * Vector3.forward;
            var horizontalForward = Vector3.ProjectOnPlane(forward, Vector3.up);
            Assert.That(Mathf.Abs(Vector3.Angle(forward, horizontalForward)), Is.LessThan(AngleToleranceDeg));
            Assert.That(Mathf.Abs(SignedRollAroundForward(constrained)), Is.LessThan(AngleToleranceDeg));
        }

        [Test]
        public void ConstrainPosePreservingWorldPoint_KeepsPivot_AllConstraints()
        {
            var pivot = new Vector3(0.2f, 1.2f, -0.1f);
            var position = new Vector3(-0.1f, 1.5f, 0.2f);
            var rotation = Quaternion.Euler(-15f, 70f, 12f);

            AssertPivotForConstraint(TrackingHorizonConstraint.None, position, rotation, pivot);
            AssertPivotForConstraint(TrackingHorizonConstraint.RemoveRoll, position, rotation, pivot);
            AssertPivotForConstraint(TrackingHorizonConstraint.RemovePitch, position, rotation, pivot);
            AssertPivotForConstraint(TrackingHorizonConstraint.RemovePitchAndRoll, position, rotation, pivot);
        }

        [Test]
        public void ConstrainPosePreservingWorldPoint_None_LeavesPoseUnchanged()
        {
            var pivot = new Vector3(0.3f, 1f, 0.4f);
            var position = new Vector3(0.1f, 1.4f, -0.2f);
            var rotation = Quaternion.Euler(5f, -20f, 3f);

            PoseConstraints.ConstrainPosePreservingWorldPoint(
                position,
                rotation,
                TrackingHorizonConstraint.None,
                Vector3.up,
                pivot,
                out var constrainedPosition,
                out var constrainedRotation);

            Assert.That(Vector3.Distance(constrainedPosition, position), Is.LessThan(PivotTolerance));
            Assert.That(Quaternion.Angle(constrainedRotation, rotation), Is.LessThan(AngleToleranceDeg));
        }

        static void AssertPivotForConstraint(
            TrackingHorizonConstraint constraint,
            Vector3 position,
            Quaternion rotation,
            Vector3 pivot)
        {
            PoseConstraints.ConstrainPosePreservingWorldPoint(
                position,
                rotation,
                constraint,
                Vector3.up,
                pivot,
                out var constrainedPosition,
                out var constrainedRotation);

            AssertPivotPreserved(position, rotation, constrainedPosition, constrainedRotation, pivot);
        }

        static void AssertPivotPreserved(
            Vector3 position,
            Quaternion rotation,
            Vector3 constrainedPosition,
            Quaternion constrainedRotation,
            Vector3 pivot)
        {
            var localOffset = Quaternion.Inverse(rotation) * (pivot - position);
            var reconstructedPivot = constrainedPosition + constrainedRotation * localOffset;
            Assert.That(Vector3.Distance(reconstructedPivot, pivot), Is.LessThan(PivotTolerance));
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

        static float SignedYawAroundUp(Quaternion rotation, Vector3 up)
        {
            var forward = Vector3.ProjectOnPlane(rotation * Vector3.forward, up);
            if (forward.sqrMagnitude < 1e-8f)
            {
                return 0f;
            }

            return Vector3.SignedAngle(Vector3.forward, forward.normalized, up);
        }
    }
}
