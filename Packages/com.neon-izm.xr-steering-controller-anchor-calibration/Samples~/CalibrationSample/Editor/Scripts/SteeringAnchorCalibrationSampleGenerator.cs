using UnityEngine;
using XrSteeringControllerAnchorCalibration;
using XrSteeringControllerAnchorCalibration.Sample;

namespace XrSteeringControllerAnchorCalibration.Sample.Editor
{
    /// <summary>SteeringAnchorCalibrationSample の Editor 専用サンプル点群生成。</summary>
    public static class SteeringAnchorCalibrationSampleGenerator
    {
        const float HeadBackOffsetMeters = 0.35f;
        const float HeadPositionJitterMeters = 0.3f;

        public static void GenerateSamplePoints(SteeringAnchorCalibrationSample sample)
        {
            ArcPointGenerator.GeneratedArc generated;
            if (sample.UseRandomWorldArcOnGenerate)
            {
                generated = ArcPointGenerator.GenerateRandom(sample.GenerationSettings);
            }
            else
            {
                var centerTransform = sample.ArcCenterTransform;
                generated = ArcPointGenerator.Generate(
                    centerTransform.position,
                    centerTransform.rotation,
                    sample.WorldArcRadiusValue,
                    sample.GenerationSettings);

                if (sample.PlaceHeadOnBackSideOnGenerate)
                {
                    PlaceHeadOnBackSide(sample, centerTransform);
                }
            }

            sample.ReplaceSampleWorldPoints(generated.Points);
        }

        static void PlaceHeadOnBackSide(SteeringAnchorCalibrationSample sample, Transform arcCenter)
        {
            var headPoseTransform = sample.HeadPoseTransform;
            if (headPoseTransform == null)
            {
                return;
            }

            var backDirection = -(arcCenter.rotation * Vector3.forward);
            var basePosition = arcCenter.position + backDirection * HeadBackOffsetMeters;

            var random = new System.Random(sample.GenerationSettings.RandomSeed ^ 0x48EAD);
            var lateralOffset = ((float)random.NextDouble() * 2f - 1f) * HeadPositionJitterMeters;
            var verticalOffset = ((float)random.NextDouble() * 2f - 1f) * HeadPositionJitterMeters;

            headPoseTransform.position = basePosition
                + arcCenter.right * lateralOffset
                + arcCenter.up * verticalOffset;
        }
    }
}
