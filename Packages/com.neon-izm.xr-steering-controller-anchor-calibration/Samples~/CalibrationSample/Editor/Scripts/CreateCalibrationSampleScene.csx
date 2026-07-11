using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using XrSteeringControllerAnchorCalibration.Sample;

var scenePath = "Packages/com.neon-izm.xr-steering-controller-anchor-calibration/Samples~/CalibrationSample/CalibrationSample.unity";
var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

var root = new GameObject("CalibrationSample");
var sample = root.AddComponent<SteeringAnchorCalibrationSample>();

var worldArc = new GameObject("WorldArcCenter");
worldArc.transform.SetParent(root.transform);
worldArc.transform.localPosition = new Vector3(-0.3f, 1.2f, 0.4f);
worldArc.transform.localRotation = Quaternion.Euler(15f, 30f, 0f);

var cgHandle = new GameObject("CgHandle");
cgHandle.transform.SetParent(root.transform);
cgHandle.transform.localPosition = new Vector3(0.35f, 1.0f, -0.2f);
cgHandle.transform.localRotation = Quaternion.Euler(-10f, 55f, 0f);

var so = new SerializedObject(sample);
so.FindProperty("worldArcCenter").objectReferenceValue = worldArc.transform;
so.FindProperty("cgHandleTransform").objectReferenceValue = cgHandle.transform;
so.FindProperty("worldArcRadius").floatValue = 0.15f;
so.ApplyModifiedPropertiesWithoutUndo();

EditorSceneManager.SaveScene(scene, scenePath);
AssetDatabase.Refresh();
return scenePath;
