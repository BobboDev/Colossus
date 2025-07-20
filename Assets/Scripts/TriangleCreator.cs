using UnityEngine;
using UnityEditor;

public class TriangleCreator : MonoBehaviour
{
    // public void CreateTriangle()
    // {
    //     GameObject triangleObject = new GameObject("ShadedTriangle");
    //     MeshFilter meshFilter = triangleObject.AddComponent<MeshFilter>();
    //     MeshRenderer meshRenderer = triangleObject.AddComponent<MeshRenderer>();

    //     // Define the vertices of the triangle
    //     Vector3[] vertices = new Vector3[]
    //     {
    //         new Vector3(0, 0, 0),
    //         new Vector3(1, 0, 0),
    //         new Vector3(0.5f, 1, 0)
    //     };

    //     // Define the triangle's normals
    //     Vector3[] normals = new Vector3[]
    //     {
    //         Vector3.forward,
    //         Vector3.forward,
    //         Vector3.forward
    //     };

    //     // Define the triangle's UV coordinates
    //     Vector2[] uv = new Vector2[]
    //     {
    //         new Vector2(0, 0),
    //         new Vector2(1, 0),
    //         new Vector2(0.5f, 1)
    //     };

    //     // Define the triangle's triangles (indices)
    //     int[] triangles = new int[] { 0, 1, 2 };

    //     // Create the mesh and assign data
    //     Mesh mesh = new Mesh();
    //     mesh.vertices = vertices;
    //     mesh.normals = normals;
    //     mesh.uv = uv;
    //     mesh.triangles = triangles;

    //     meshFilter.mesh = mesh;

    //     // Set a default material for shading
    //     meshRenderer.material = new Material(Shader.Find("Standard"));
    // }
}
