using UnityEditor;
using UnityEngine;
using XrSteeringControllerAnchorCalibration;
using XrSteeringControllerAnchorCalibration.Sample;

namespace XrSteeringControllerAnchorCalibration.Sample.Editor
{
    static class MappedHeadVisualUtility
    {
        const string CloneName = "Head (Mapped)";
        static readonly Color MappedHeadColor = Color.red;

        public static void CreateOrUpdate(SteeringAnchorCalibrationSample sample)
        {
            if (sample == null || !sample.HasCalibrationResult || sample.HeadPoseTransform == null)
            {
                return;
            }

            Destroy(sample);

            var source = sample.HeadPoseTransform.gameObject;
            var clone = Object.Instantiate(source);
            clone.name = CloneName;
            clone.transform.SetParent(sample.transform, false);

            ApplyModelView(clone.transform, sample.HeadPoseTransform, sample.ModelView);
            SetRenderersRed(clone);

            Undo.RegisterCreatedObjectUndo(clone, "Mapped Head Visual");
            EditorUtility.SetDirty(clone);
            Selection.activeGameObject = clone;
        }

        public static void Destroy(SteeringAnchorCalibrationSample sample)
        {
            if (sample == null)
            {
                return;
            }

            var existing = sample.transform.Find(CloneName);
            if (existing == null)
            {
                return;
            }

            Undo.DestroyObjectImmediate(existing.gameObject);
        }

        static void ApplyModelView(Transform clone, Transform source, Matrix4x4 modelView)
        {
            clone.position = AnchorCalibration.WorldToCg(source.position, modelView);
            clone.rotation = modelView.rotation * source.rotation;
            clone.localScale = source.lossyScale;
        }

        static void SetRenderersRed(GameObject root)
        {
            foreach (var renderer in root.GetComponentsInChildren<Renderer>(true))
            {
                var materials = renderer.sharedMaterials;
                for (var i = 0; i < materials.Length; i++)
                {
                    var material = new Material(materials[i]);
                    if (material.HasProperty("_BaseColor"))
                    {
                        material.SetColor("_BaseColor", MappedHeadColor);
                    }

                    if (material.HasProperty("_Color"))
                    {
                        material.SetColor("_Color", MappedHeadColor);
                    }

                    materials[i] = material;
                }

                renderer.sharedMaterials = materials;
            }
        }
    }
}
