using UnityEngine;
using XrSteeringControllerAnchorCalibration;

namespace XrSteeringControllerAnchorCalibration.Sample
{
    public static class CircleGizmoDrawer
    {
        public static void DrawCircle(Circle3D circle, Color color, int segments = 64)
        {
            if (circle.Radius <= 0f || segments < 3)
            {
                return;
            }

            GetPlaneBasis(circle, out var tangent, out var bitangent);
            var previous = circle.Position + circle.Radius * tangent;
            Gizmos.color = color;

            for (var i = 1; i <= segments; i++)
            {
                var angle = i / (float)segments * Mathf.PI * 2f;
                var point = circle.Position + circle.Radius * (Mathf.Cos(angle) * tangent + Mathf.Sin(angle) * bitangent);
                Gizmos.DrawLine(previous, point);
                previous = point;
            }

            DrawAxis(circle, color);
        }

        public static void DrawArc(Circle3D circle, Color color, float arcDegrees, int segments = 64)
        {
            if (arcDegrees >= 359.9f)
            {
                DrawCircle(circle, color, segments);
                return;
            }

            if (circle.Radius <= 0f || segments < 3)
            {
                return;
            }

            GetPlaneBasis(circle, out var tangent, out var bitangent);
            var halfArc = arcDegrees * 0.5f;
            var previous = Vector3.zero;
            var hasPrevious = false;

            for (var i = 0; i <= segments; i++)
            {
                var t = i / (float)segments;
                var angle = Mathf.Lerp(-halfArc, halfArc, t) * Mathf.Deg2Rad;
                var point = circle.Position + circle.Radius * (Mathf.Cos(angle) * tangent + Mathf.Sin(angle) * bitangent);

                if (hasPrevious)
                {
                    Gizmos.color = color;
                    Gizmos.DrawLine(previous, point);
                }

                previous = point;
                hasPrevious = true;
            }

            DrawAxis(circle, color);
        }

        public static void DrawPoseAxes(Vector3 position, Quaternion rotation, float axisLength)
        {
            if (axisLength <= 0f)
            {
                return;
            }

            Gizmos.color = Color.red;
            Gizmos.DrawLine(position, position + rotation * Vector3.right * axisLength);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(position, position + rotation * Vector3.up * axisLength);
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(position, position + rotation * Vector3.forward * axisLength);
        }

        public static void DrawAxis(Circle3D circle, Color color)
        {
            Gizmos.color = color;
            Gizmos.DrawLine(circle.Position, circle.Position + circle.Normal * circle.Radius * 0.5f);
        }

        public static Vector3 SamplePointOnCircle(Circle3D circle, float angleRadians)
        {
            GetPlaneBasis(circle, out var tangent, out var bitangent);
            return circle.Position + circle.Radius * (Mathf.Cos(angleRadians) * tangent + Mathf.Sin(angleRadians) * bitangent);
        }

        public static void GetPlaneBasis(Circle3D circle, out Vector3 tangent, out Vector3 bitangent)
        {
            tangent = (circle.Rotation * Vector3.right).normalized;
            bitangent = (circle.Rotation * Vector3.up).normalized;

            if (tangent.sqrMagnitude < 1e-8f || bitangent.sqrMagnitude < 1e-8f)
            {
                var normal = circle.Normal.normalized;
                tangent = Vector3.Cross(normal, Vector3.up);
                if (tangent.sqrMagnitude < 1e-8f)
                {
                    tangent = Vector3.Cross(normal, Vector3.right);
                }

                tangent.Normalize();
                bitangent = Vector3.Cross(normal, tangent).normalized;
            }
        }
    }
}
