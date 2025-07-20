// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    [System.Serializable]
    public class Triangle
    {
        public string triangleId;

        int v1, v2, v3;
        public Vector3 normalizedPositon;
        public Vector3 trianglePosition;
        Vector3[] triangleVertexes;
        public Vector3[] vertexes;
        Transform owner;
        Mesh mesh;

        public float triangleMagnitude;

        public int triangleIndex;

        public bool block = false;

        public TriangleProvider triangleProvider;

        [HideInInspector] public Quaternion triangleRotation;
        [HideInInspector] public Vector3 v1Pos, v2Pos, v3Pos;

        public Triangle(int v1, int v2, int v3, Vector3[] vertexes, Transform owner, Mesh mesh, TriangleProvider triangleProvider)
        {
            this.triangleProvider = triangleProvider;
            this.owner = owner;
            this.mesh = mesh;
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;
            triangleVertexes = new Vector3[] { vertexes[v1], vertexes[v2], vertexes[v3] };
            trianglePosition = (vertexes[v1] + vertexes[v2] + vertexes[v3]) / 3;

            triangleRotation = GetAbstractTriangleRotation();
            v1Pos = owner.localToWorldMatrix.MultiplyPoint3x4(triangleProvider.realTimeVertices[v1]);
            v1Pos = owner.localToWorldMatrix.MultiplyPoint3x4(triangleProvider.realTimeVertices[v2]);
            v3Pos = owner.localToWorldMatrix.MultiplyPoint3x4(triangleProvider.realTimeVertices[v3]);

            this.vertexes = vertexes;
            triangleMagnitude = trianglePosition.magnitude;
            triangleIndex = v1 + v2 + v3;
            triangleId = triangleIndex.ToString();
        }

        public Transform GetOwner()
        {
            return owner;
        }

        public Vector3 GetVertexWorld(int index)
        {
            return owner.transform.TransformPoint(triangleVertexes[index]);
        }

        public Vector3[] GetVertexWorldRealtime()
        {
            return new Vector3[] {
                owner.transform.TransformPoint(triangleProvider.realTimeVertices[v1]),
                owner.transform.TransformPoint(triangleProvider.realTimeVertices[v2]),
                owner.transform.TransformPoint(triangleProvider.realTimeVertices[v3])
            };
        }

        public Vector3 GetTrianglePositionInWorldSpace()
        {
            return GetTrianglePositionInWorldSpacePersistent();
        }

        public Quaternion GetAbstractTriangleRotation()
        {
            Vector3 a = owner.localToWorldMatrix.MultiplyPoint3x4(triangleProvider.realTimeVertices[v1]);
            Vector3 b = owner.localToWorldMatrix.MultiplyPoint3x4(triangleProvider.realTimeVertices[v2]);
            Vector3 c = owner.localToWorldMatrix.MultiplyPoint3x4(triangleProvider.realTimeVertices[v3]);
            Vector3 forwardDirection = b - a;
            Vector3 rightDirection = c - a;
            Vector3 upDirection = Vector3.Cross(forwardDirection, rightDirection);
            Quaternion orientation = Quaternion.LookRotation(forwardDirection, upDirection);
            return orientation;
        }

        public Vector3 GetTrianglePositionInWorldSpacePersistent()
        {
            Vector3[] realTimeVertexes = triangleProvider.realTimeVertices;//mesh.vertices;
            Vector3 realTimeTrianglePosition = (realTimeVertexes[v1] + realTimeVertexes[v2] + realTimeVertexes[v3]) / 3;
            return owner.localToWorldMatrix.MultiplyPoint3x4(realTimeTrianglePosition);
        }

        public Vector3 ComputeNormalizePosition()
        {
            return ComputeNormalizePositionPersistent();
        }

        public Vector3 ComputeNormalizePositionPersistent()
        {
            Vector3[] realTimeVertexes = triangleProvider.realTimeVertices;//mesh.vertices;
            Vector3 cross = Vector3.Cross(realTimeVertexes[v2] - realTimeVertexes[v1], realTimeVertexes[v3] - realTimeVertexes[v1]);
            Vector3 cross2 = Vector3.Cross(
                owner.localToWorldMatrix.MultiplyPoint3x4(realTimeVertexes[v2]) - owner.localToWorldMatrix.MultiplyPoint3x4(realTimeVertexes[v1]),
                owner.localToWorldMatrix.MultiplyPoint3x4(realTimeVertexes[v3]) - owner.localToWorldMatrix.MultiplyPoint3x4(realTimeVertexes[v1])
            );
            Vector3 vec = (cross / cross.magnitude);
            Vector3 vec2 = (cross2 / cross2.magnitude);
            return vec2;
        }
    }
}