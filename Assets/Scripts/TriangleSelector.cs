using UnityEngine;

[ExecuteInEditMode]
public class TriangleSelector : MonoBehaviour
{
    private MeshFilter meshFilter;
    private Mesh mesh;

    private void OnEnable()
    {
        meshFilter = GetComponent<MeshFilter>();
    }

    private void Update()
    {
        HandleInput();
    }

    private void HandleInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit))
            {
                int triangleIndex = FindTriangleUnderCursor(hit.triangleIndex, hit.barycentricCoordinate);
                Debug.Log("Selected Triangle: " + triangleIndex);
            }
        }
    }

    private int FindTriangleUnderCursor(int hitTriangleIndex, Vector3 barycentricCoordinate)
    {
        if (barycentricCoordinate.x > barycentricCoordinate.y && barycentricCoordinate.x > barycentricCoordinate.z)
            return mesh.triangles[hitTriangleIndex * 3];
        else if (barycentricCoordinate.y > barycentricCoordinate.z)
            return mesh.triangles[hitTriangleIndex * 3 + 1];
        else
            return mesh.triangles[hitTriangleIndex * 3 + 2];
    }
}
