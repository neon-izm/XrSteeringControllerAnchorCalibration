using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using PackageInfo = UnityEditor.PackageManager.PackageInfo;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Tests
{
    /// <summary>
    /// Loads calibration session JSON written by the demo app
    /// (<c>CalibrationSessionExporter</c>) or reconstructed from device logs.
    /// </summary>
    public static class CalibrationSessionFixture
    {
        [Serializable]
        public class Dto
        {
            public string schemaVersion;
            public string name;
            public string source;
            public string capturedAtUtc;
            public string notes;
            public string appVersion;
            public float[] handlePosition;
            public float[] handleRotation;
            public float[] headPosition;
            public float[] headRotation;
            public float[] points;

            public float expectedCircleRadiusApprox;
            public float expectedNormalDotUpApprox;
            public float expectedMaxHandleLocalRollDeg = 1e-2f;
            public bool expectedRequireDriverSeatSide = true;
            public float expectedRadiusTolerance = 0.02f;
            public float expectedNormalDotTolerance = 0.005f;
        }

        public readonly struct Session
        {
            public readonly Dto Raw;
            public readonly List<Vector3> Points;
            public readonly CgHandlePose Handle;
            public readonly HeadPose Head;

            public Session(Dto raw, List<Vector3> points, CgHandlePose handle, HeadPose head)
            {
                Raw = raw;
                Points = points;
                Handle = handle;
                Head = head;
            }
        }

        const string FixtureAnchorGuid = "310e2904b63ba3947875dda4c5d396f3";

        public static string FixturesDirectory
        {
            get
            {
                var assetPath = AssetDatabase.GUIDToAssetPath(FixtureAnchorGuid);
                Assert.That(assetPath, Is.Not.Empty, "Fixture anchor asset was not found");

                var package = PackageInfo.FindForAssetPath(assetPath);
                string directory;
                if (package != null)
                {
                    var relative = assetPath.Substring(package.assetPath.Length).TrimStart('/');
                    directory = Path.GetDirectoryName(Path.Combine(package.resolvedPath, relative));
                }
                else
                {
                    var projectRoot = Path.GetDirectoryName(Application.dataPath);
                    directory = Path.GetDirectoryName(Path.Combine(projectRoot, assetPath));
                }

                Assert.That(Directory.Exists(directory), Is.True, $"Fixtures directory missing: {directory}");
                return directory;
            }
        }

        public static Session Load(string fileName)
        {
            var path = Path.Combine(FixturesDirectory, fileName);
            Assert.That(File.Exists(path), Is.True, $"Missing fixture: {path}");
            var json = File.ReadAllText(path);
            var dto = JsonUtility.FromJson<Dto>(json);
            Assert.That(dto, Is.Not.Null);
            Assert.That(dto.points, Is.Not.Null);
            Assert.That(dto.points.Length % 3, Is.EqualTo(0));
            Assert.That(dto.handlePosition, Is.Not.Null.And.Length.EqualTo(3));
            Assert.That(dto.handleRotation, Is.Not.Null.And.Length.EqualTo(4));
            Assert.That(dto.headPosition, Is.Not.Null.And.Length.EqualTo(3));
            Assert.That(dto.headRotation, Is.Not.Null.And.Length.EqualTo(4));

            var points = new List<Vector3>(dto.points.Length / 3);
            for (var i = 0; i < dto.points.Length; i += 3)
            {
                points.Add(new Vector3(dto.points[i], dto.points[i + 1], dto.points[i + 2]));
            }

            var handle = new CgHandlePose(
                ToVector3(dto.handlePosition),
                ToQuaternion(dto.handleRotation));
            var head = new HeadPose(
                ToVector3(dto.headPosition),
                ToQuaternion(dto.headRotation));
            return new Session(dto, points, handle, head);
        }

        static Vector3 ToVector3(float[] v) => new(v[0], v[1], v[2]);

        static Quaternion ToQuaternion(float[] q) => new(q[0], q[1], q[2], q[3]);
    }
}
