using UnityEditor;
using UnityEngine;
using XrSteeringControllerAnchorCalibration.Sample;

namespace XrSteeringControllerAnchorCalibration.Sample.Editor
{
    [CustomEditor(typeof(SteeringAnchorCalibrationSample))]
    public class SteeringAnchorCalibrationSampleEditor : UnityEditor.Editor
    {
        bool showAdvanced;

        public override void OnInspectorGUI()
        {
            var sample = (SteeringAnchorCalibrationSample)target;
            serializedObject.Update();

            EditorGUILayout.HelpBox(
                "HMD 世界座標の軌跡点と頭姿勢から ModelView を求めるサンプルです。\n" +
                "1. CgHandle / Head を設定\n" +
                "2. Generate → Run Calibration",
                MessageType.Info);

            EditorGUILayout.PropertyField(serializedObject.FindProperty("cgHandleTransform"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("headPoseTransform"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("worldArcCenter"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("worldArcRadius"));

            EditorGUILayout.Space(6f);
            DrawArcSpanControls();

            EditorGUILayout.Space(6f);
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("Generate Sample Points"))
                {
                    Undo.RecordObject(sample, "Generate Sample Points");
                    if (sample.HeadPoseTransform != null)
                    {
                        Undo.RecordObject(sample.HeadPoseTransform, "Generate Sample Points");
                    }

                    MappedHeadVisualUtility.Destroy(sample);
                    SteeringAnchorCalibrationSampleGenerator.GenerateSamplePoints(sample);
                    EditorUtility.SetDirty(sample);
                }

                if (GUILayout.Button("Run Calibration"))
                {
                    Undo.RecordObject(sample, "Run Calibration");
                    sample.RunCalibration();
                    if (sample.HasCalibrationResult && string.IsNullOrEmpty(sample.LastError))
                    {
                        MappedHeadVisualUtility.CreateOrUpdate(sample);
                    }

                    EditorUtility.SetDirty(sample);
                }

                if (GUILayout.Button("Clear"))
                {
                    Undo.RecordObject(sample, "Clear Sample Points");
                    MappedHeadVisualUtility.Destroy(sample);
                    sample.ClearSamplePoints();
                    EditorUtility.SetDirty(sample);
                }
            }

            if (!string.IsNullOrEmpty(sample.LastError))
            {
                EditorGUILayout.HelpBox(sample.LastError, MessageType.Error);
            }
            else if (sample.HasCalibrationResult)
            {
                var seatLabel = sample.IsOnDriverSeatSide ? "運転席側" : "ボンネット側";
                var totalArcDeg = sample.ArcHalfAngleDeg * 2f;
                EditorGUILayout.HelpBox(
                    $"ModelView を算出しました。Inliers: {sample.LastInlierCount}/{sample.SampleWorldPoints.Count}  " +
                    $"中心誤差: {sample.MappedCenterErrorMeters * 1000f:F1} mm  " +
                    $"inlier 最大: {sample.MappedInliersMaxErrorMeters * 1000f:F1} mm\n" +
                    $"弧: {totalArcDeg:F0}°  " +
                    $"前後スコア: {sample.FrontBackScore:F3}  roll: {sample.HandleLocalRollDeg:F1}°  → {seatLabel}",
                    MessageType.None);
            }

            showAdvanced = EditorGUILayout.Foldout(showAdvanced, "Advanced", true);
            if (showAdvanced)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("generationSettings"), true);
                EditorGUILayout.PropertyField(serializedObject.FindProperty("useRandomWorldArcOnGenerate"));
                EditorGUILayout.PropertyField(
                    serializedObject.FindProperty("placeHeadOnBackSideOnGenerate"),
                    new GUIContent(
                        "Place Head On Back Side",
                        "Generate 時に Head をハンドル Back 側へ移動し、左右・上下に ±0.3 m のランダムオフセットを加える。回転は変更しない。"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("calibrationThreshold"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("calibrationMaxIterations"));
                EditorGUILayout.PropertyField(
                    serializedObject.FindProperty("forceFlippedNormal"),
                    new GUIContent("Force Flipped Normal", "デバッグ用。前後反転候補を強制採用する。"));
            }

            serializedObject.ApplyModifiedProperties();
        }

        void DrawArcSpanControls()
        {
            var generationSettings = serializedObject.FindProperty("generationSettings");
            var arcHalfAngle = generationSettings.FindPropertyRelative("ArcHalfAngleDeg");

            EditorGUILayout.LabelField("Arc Span", EditorStyles.boldLabel);
            using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
            {
                EditorGUILayout.Slider(
                    arcHalfAngle,
                    5f,
                    90f,
                    new GUIContent(
                        "Arc Half Angle (deg)",
                        "Generate する弧の半開角。合計弧長 = 2 × この値。"));

                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("45° 弧"))
                    {
                        arcHalfAngle.floatValue = 22.5f;
                    }

                    if (GUILayout.Button("90° 弧"))
                    {
                        arcHalfAngle.floatValue = 45f;
                    }
                }

                EditorGUILayout.LabelField(
                    "Total Arc Span",
                    $"{arcHalfAngle.floatValue * 2f:F1}°",
                    EditorStyles.miniLabel);
            }
        }

        void OnSceneGUI()
        {
            var sample = (SteeringAnchorCalibrationSample)target;
            if (sample.CgHandleTransform == null)
            {
                return;
            }

            var handle = sample.CgHandleTransform;
            var forward = handle.rotation * Vector3.forward;
            var frontPos = handle.position + forward * 0.12f;
            var backPos = handle.position - forward * 0.12f;

            Handles.color = Color.cyan;
            Handles.Label(frontPos, "Front");
            Handles.color = Color.yellow;
            Handles.Label(backPos, "Back");
        }
    }
}
