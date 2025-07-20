// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class PrismProjection : MonoBehaviour
    {
        public static Vector3 CenterOfVectors(Vector3[] vectors)
        {
            Vector3 sum = Vector3.zero;
            if (vectors == null || vectors.Length == 0)
            {
                return sum;
            }

            foreach (Vector3 vec in vectors)
            {
                sum += vec;
            }
            return sum / vectors.Length;
        }

        public static Vector3[] CreatePrismProjection(Vector3 vertexA, Vector3 vertexB, Vector3 vertexC, float projectionHeight = 0.8f)
        {
            Vector3 cross = Vector3.Cross(vertexB - vertexA, vertexC - vertexA);
            Vector3 normal = (cross / cross.magnitude);
            float projectionSize = projectionHeight;//0.8f;
            return new Vector3[] { vertexA + normal * projectionSize, vertexB + normal * projectionSize, vertexC + normal * projectionSize };
        }

        public static void DebugTriangle(Triangle triangle, Color color)
        {
            Vector3[] vertexes = new Vector3[] { triangle.GetVertexWorld(0), triangle.GetVertexWorld(1), triangle.GetVertexWorld(2) };
            Vector3[] prism = CreatePrismProjection(vertexes[0], vertexes[1], vertexes[2]);
            DebugPrism(vertexes, prism, color);
        }


        public static void DebugTriangleRealtime(Triangle triangle, Color color)
        {
            Vector3[] vertexes = triangle.GetVertexWorldRealtime();
            Vector3[] prism = CreatePrismProjection(vertexes[0], vertexes[1], vertexes[2]);
            DebugPrism(vertexes, prism, color);
        }


        public static void DebugPrism(Vector3[] vertexes, Vector3[] prism, Color color)
        {
            if (!Application.isEditor) return;
            Debug.DrawLine(vertexes[0], prism[0], color);
            Debug.DrawLine(vertexes[1], prism[1], color);
            Debug.DrawLine(vertexes[2], prism[2], color);
            Debug.DrawLine(vertexes[0], vertexes[1], color);
            Debug.DrawLine(vertexes[1], vertexes[2], color);
            Debug.DrawLine(vertexes[0], vertexes[2], color);
            Debug.DrawLine(prism[0], prism[1], color);
            Debug.DrawLine(prism[1], prism[2], color);
            Debug.DrawLine(prism[0], prism[2], color);
        }
    }
}