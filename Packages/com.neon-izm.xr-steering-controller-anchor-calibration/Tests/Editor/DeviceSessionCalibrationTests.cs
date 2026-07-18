using System.IO;
using NUnit.Framework;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    /// <summary>
    /// Regression tests seeded from Quest device sessions (raw dumps or log-reconstructed).
    /// Drop new JSON under <c>Tests/Editor/Fixtures/</c> after
    /// <c>adb pull …/files/CalibrationSessions/</c>.
    /// </summary>
    public class DeviceSessionCalibrationTests
    {
        [TestCase("quest-20260716-midtilt-log-reconstructed.json")]
        [TestCase("quest-20260715-155630-device.json")]
        [TestCase("quest-20260715-160854-device-contentAlign-ok.json")]
        public void DeviceSession_Calibrate_MatchesFixtureExpectations(string fileName)
        {
            var session = CalibrationSessionFixture.Load(fileName);
            var dto = session.Raw;

            Assert.That(session.Points.Count, Is.GreaterThanOrEqualTo(8));

            var fit = CircleFitting3D.FitCircleMsac(session.Points, random: new System.Random(4));
            var modelView = AnchorCalibration.ComputeCalibratedModelView(
                fit.Circle,
                session.Handle,
                session.Head,
                forceFlippedCandidate: false,
                out var orientedCircle);

            var roll = AnchorCalibration.GetHandleLocalRollDeg(modelView, orientedCircle, session.Handle);
            var normalDotUp = Mathf.Abs(Vector3.Dot(orientedCircle.Normal.normalized, Vector3.up));
            var mapped = AnchorCalibration.TransformCircleRigid(modelView, orientedCircle);

            Assert.That(modelView, Is.Not.EqualTo(Matrix4x4.zero));
            // ModelView + RemoveHandleLocalRoll 後は CG ハンドル姿勢に代数的に一致する。
            Assert.That(
                Vector3.Distance(mapped.Position, session.Handle.Position),
                Is.LessThan(1e-3f));
            Assert.That(
                Vector3.Angle(mapped.Normal, session.Handle.Forward),
                Is.LessThan(1e-2f));

            var maxRoll = dto.expectedMaxHandleLocalRollDeg > 0f
                ? dto.expectedMaxHandleLocalRollDeg
                : 1e-2f;
            Assert.That(Mathf.Abs(roll), Is.LessThan(maxRoll), $"handleLocalRollDeg={roll}");

            Assert.That(
                AnchorCalibration.IsOnDriverSeatSide(modelView, session.Head, session.Handle),
                Is.True);

            if (dto.expectedCircleRadiusApprox > 0f)
            {
                Assert.That(
                    orientedCircle.Radius,
                    Is.EqualTo(dto.expectedCircleRadiusApprox).Within(dto.expectedRadiusTolerance));
            }

            if (dto.expectedNormalDotUpApprox > 0f)
            {
                Assert.That(
                    normalDotUp,
                    Is.EqualTo(dto.expectedNormalDotUpApprox).Within(dto.expectedNormalDotTolerance));
            }
        }

        [Test]
        public void FixturesDirectory_ExistsAndContainsAtLeastOneSession()
        {
            var dir = CalibrationSessionFixture.FixturesDirectory;
            Assert.That(Directory.Exists(dir), Is.True, dir);
            var files = Directory.GetFiles(dir, "*.json");
            Assert.That(files.Length, Is.GreaterThan(0));
        }
    }
}
