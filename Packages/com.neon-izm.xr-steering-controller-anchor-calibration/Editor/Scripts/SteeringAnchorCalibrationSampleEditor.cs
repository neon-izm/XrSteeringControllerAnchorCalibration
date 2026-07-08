using UnityEditor;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Editor
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
                "HMD 世界座標の軌跡点から ModelView を求めるサンプルです。\n" +
                "1. CgHandle に既知の CG ハンドル姿勢を設定\n" +
                "2. Generate → Run Calibration",
                MessageType.Info);

            EditorGUILayout.PropertyField(serializedObject.FindProperty("cgHandleTransform"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("worldArcCenter"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("worldArcRadius"));

            EditorGUILayout.Space(6f);
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("Generate Sample Points"))
                {
                    Undo.RecordObject(sample, "Generate Sample Points");
                    sample.GenerateSamplePoints();
                    EditorUtility.SetDirty(sample);
                }

                if (GUILayout.Button("Run Calibration"))
                {
                    Undo.RecordObject(sample, "Run Calibration");
                    sample.RunCalibration();
                    EditorUtility.SetDirty(sample);
                }

                if (GUILayout.Button("Clear"))
                {
                    Undo.RecordObject(sample, "Clear Sample Points");
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
                EditorGUILayout.HelpBox(
                    $"ModelView を算出しました。Inliers: {sample.LastInlierCount}/{sample.SampleWorldPoints.Count}  " +
                    $"中心誤差: {sample.MappedCenterErrorMeters * 1000f:F1} mm  " +
                    $"inlier 最大: {sample.MappedInliersMaxErrorMeters * 1000f:F1} mm",
                    MessageType.None);
            }

            showAdvanced = EditorGUILayout.Foldout(showAdvanced, "Advanced", true);
            if (showAdvanced)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("generationSettings"), true);
                EditorGUILayout.PropertyField(serializedObject.FindProperty("useRandomWorldArcOnGenerate"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("calibrationThreshold"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("calibrationMaxIterations"));
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
}
