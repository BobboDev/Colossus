using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(SkinnedMeshRenderer))]
public class MyObjectEditor : Editor
{
    private void OnSceneGUI()
    {
        // SkinnedMeshRenderer smr = target as SkinnedMeshRenderer;

        // if (smr == null || smr.sharedMesh == null)
        //     return;

        // Mesh mesh = smr.sharedMesh;
        // Vector3[] vertices = mesh.vertices;
        // int[] triangles = mesh.triangles;

        // // Get the transform of the object to which the mesh is attached
        // Transform transform = smr.transform;

        // // Draw each triangle in the mesh
        // for (int i = 0; i < triangles.Length; i += 3)
        // {
        //     Vector3 vertex1 = transform.TransformPoint(vertices[triangles[i]]);
        //     Vector3 vertex2 = transform.TransformPoint(vertices[triangles[i + 1]]);
        //     Vector3 vertex3 = transform.TransformPoint(vertices[triangles[i + 2]]);

        //     // Draw the triangle
        //     Handles.DrawAAConvexPolygon(vertex1, vertex2, vertex3);
        // }
    }
}