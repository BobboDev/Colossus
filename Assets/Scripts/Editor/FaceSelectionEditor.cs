using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

[CustomEditor(typeof(FaceSelection))]
public class FaceSelectionEditor : Editor
{
    private List<int> selectedTriangles = new List<int>();
    private Mesh mesh;
    private Transform meshTransform;
    private Vector3 hitpoint;

    private bool isEditing;
    private Event guiEvent;
    private FaceSelection faceSelection;
    private bool hasUnsavedChanges;

    private void OnEnable()
    {
        faceSelection = (FaceSelection)target;
        if (faceSelection.SkinnedMeshRenderer != null)
        {
            mesh = faceSelection.SkinnedMeshRenderer.sharedMesh;
        }
        if (faceSelection.MeshFilter != null)
        {
            mesh = faceSelection.MeshFilter.sharedMesh;
        }
        meshTransform = faceSelection.transform;

        if (faceSelection.Areas == null)
        {
            faceSelection.Areas = new List<FaceSelection.AreaEntry>();
        }
    }

    private void OnSceneGUI()
    {
        guiEvent = Event.current;

        if (isEditing)
        {
            // Draw the grey overlay when editing
            DrawGreyOverlay();

            if (mesh != null)
            {
                DrawHitpoint();
                HandleFaceSelection();
                DrawFaceHighlights();
            }

            if (IsBlockingInput())
            {
                guiEvent.Use();
            }

            SceneView.RepaintAll();
        }
    }

    private void DrawGreyOverlay()
    {
        if (!isEditing) return;

        // Get the current SceneView
        SceneView sceneView = SceneView.currentDrawingSceneView;
        if (sceneView == null) return;

        // Create a semi-transparent grey overlay
        Handles.BeginGUI();
        Rect rect = new Rect(0, 0, sceneView.position.width, sceneView.position.height);
        GUI.color = new Color(0f, 0f, 0f, 0.5f); // Semi-transparent grey
        GUI.DrawTexture(rect, Texture2D.whiteTexture);
        GUI.color = Color.white; // Reset GUI color
        Handles.EndGUI();
    }

    private void DrawHitpoint()
    {
        Handles.color = Color.blue;
        Handles.DrawWireCube(hitpoint, Vector3.one * 0.01f);
        Handles.color = Color.white;
    }

    private bool IsBlockingInput()
    {
        return (guiEvent.type == EventType.MouseDown || guiEvent.type == EventType.MouseUp || guiEvent.type == EventType.MouseDrag) && !guiEvent.alt;
    }

    private bool ShouldHandleInput()
    {
        if (guiEvent.type == EventType.KeyDown && guiEvent.keyCode == KeyCode.Escape)
        {
            isEditing = false;
            return false;
        }

        if (guiEvent.type == EventType.Used)
        {
            guiEvent.Use();
            return false;
        }

        if (guiEvent.alt || (guiEvent.isMouse && guiEvent.button == 1))
        {
            return false;
        }

        if (guiEvent.isMouse)
        {
            return RaycastHandle();
        }

        return true;
    }

    private bool RaycastHandle()
    {
        Ray ray = HandleUtility.GUIPointToWorldRay(guiEvent.mousePosition);
        if (Physics.Raycast(ray, out RaycastHit hit))
        {
            SkinnedMeshRenderer skinnedMeshRenderer = hit.collider.GetComponent<SkinnedMeshRenderer>();

            if (skinnedMeshRenderer != null && skinnedMeshRenderer.sharedMesh == mesh)
            {
                return true;
            }

            guiEvent.Use();
            return false;
        }

        guiEvent.Use();
        return false;
    }

    private void HandleFaceSelection()
    {
        if (!isEditing || !ShouldHandleInput()) return;

        Ray ray = HandleUtility.GUIPointToWorldRay(guiEvent.mousePosition);

        if (guiEvent.type == EventType.MouseDown && guiEvent.button == 0 && Physics.Raycast(ray, out RaycastHit hit))
        {
            MeshCollider collider = hit.collider.GetComponent<MeshCollider>();

            if (collider != null && collider.GetComponent<SkinnedMeshRenderer>().sharedMesh == mesh)
            {
                ToggleTriangleSelection(hit.triangleIndex);
                hasUnsavedChanges = true; // Set the flag when a change is made
                SceneView.RepaintAll(); // Ensure SceneView updates
            }
        }
    }

    private void ToggleTriangleSelection(int hitTriangleIndex)
    {
        if (selectedTriangles.Contains(hitTriangleIndex))
        {
            selectedTriangles.Remove(hitTriangleIndex);
        }
        else
        {
            selectedTriangles.Add(hitTriangleIndex);
        }
    }

    private void DrawFaceHighlights()
    {
        Handles.zTest = UnityEngine.Rendering.CompareFunction.Less;
        Handles.lighting = false;

        foreach (int triangleIndex in selectedTriangles)
        {
            Vector3[] faceVertices = GetFaceVertices(triangleIndex, 0.002f);
            DrawFaceOutline(faceVertices);
        }
    }

    private void DrawFaceOutline(Vector3[] faceVertices)
    {
        Handles.color = Color.green;
        Handles.DrawAAConvexPolygon(faceVertices);
        Handles.color = Color.white;
        Handles.DrawPolyLine(faceVertices);
    }

    private Vector3[] GetFaceVertices(int triangleIndex, float shrinkDistance)
    {
        // Access mesh data
        int[] triangles = mesh.triangles;
        Vector3[] vertices = mesh.vertices;

        // Get the indices of the vertices for the triangle
        int idx0 = triangles[triangleIndex * 3];
        int idx1 = triangles[triangleIndex * 3 + 1];
        int idx2 = triangles[triangleIndex * 3 + 2];

        // Get the vertices of the triangle
        Vector3 v0 = vertices[idx0];
        Vector3 v1 = vertices[idx1];
        Vector3 v2 = vertices[idx2];

        // // Calculate the incenter and radius of the inscribed circle
        // Vector3 incenter = CalculateIncenter(v0, v1, v2);
        // float radius = CalculateInradius(v0, v1, v2);

        // // Calculate the homothety coefficient k
        // float k = 1 - (radius / (radius + shrinkDistance));

        // Calculate the shrunk vertices
        Vector3 shrunkV0 = v0;// + (incenter - v0) * k;
        Vector3 shrunkV1 = v1;// + (incenter - v1) * k;
        Vector3 shrunkV2 = v2;// + (incenter - v2) * k;
        Vector3 faceNormal = CalculateFaceNormal(vertices[triangles[triangleIndex * 3]], vertices[triangles[triangleIndex * 3 + 1]], vertices[triangles[triangleIndex * 3 + 2]]) * 0.005f;
        // Transform the vertices to world space
        return new Vector3[]
        {
        meshTransform.TransformPoint(shrunkV0 + faceNormal),
        meshTransform.TransformPoint(shrunkV1 + faceNormal),
        meshTransform.TransformPoint(shrunkV2 + faceNormal),
        meshTransform.TransformPoint(shrunkV0 + faceNormal)  // Closing the loop of the triangle
        };
    }

    private Vector3 CalculateIncenter(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        float a = Vector3.Distance(v1, v2);
        float b = Vector3.Distance(v0, v2);
        float c = Vector3.Distance(v0, v1);

        float p = a + b + c;

        return (a * v0 + b * v1 + c * v2) / p;
    }

    private float CalculateInradius(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        float a = Vector3.Distance(v1, v2);
        float b = Vector3.Distance(v0, v2);
        float c = Vector3.Distance(v0, v1);

        float s = (a + b + c) / 2;
        float area = Mathf.Sqrt(s * (s - a) * (s - b) * (s - c));

        return area / s;
    }

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        if (GUILayout.Button("Clear Selection"))
        {
            ClearSelection();
        }

        DrawAreaButtonsWithRenameDeleteAndSave();

        if (GUILayout.Button("Create New Area"))
        {
            SaveNewSelection();
            isEditing = true; // Enter edit mode when creating a new area
            SceneView.RepaintAll(); // Force SceneView to refresh
        }

        Repaint();
    }

    private void DrawAreaButtonsWithRenameDeleteAndSave()
    {
        if (faceSelection.Areas.Count > 0)
        {
            for (int i = 0; i < faceSelection.Areas.Count; i++)
            {
                string areaKey = faceSelection.Areas[i].Key;
                bool isSelectedArea = (areaKey == faceSelection.selectedKey);
                GUILayout.BeginHorizontal(); // Start a horizontal layout

                // Disable GUI elements if this area is not selected
                GUI.enabled = isSelectedArea;

                // Text field for renaming the area
                string newName = GUILayout.TextField(faceSelection.Areas[i].Key);

                // Save new name if changed
                if (newName != faceSelection.Areas[i].Key)
                {
                    RenameArea(faceSelection.Areas[i].Key, newName);
                }

                // Draw the "Save" button only if there are unsaved changes and this is the selected area
                if (hasUnsavedChanges && isSelectedArea)
                {
                    if (GUILayout.Button("Save", GUILayout.Width(60)))
                    {
                        SaveSelectedFaces();
                    }
                }

                // Enable the GUI for the "Select" button regardless of the selected area
                GUI.enabled = true;

                // Draw the "Select" button
                if (GUILayout.Button("Select", GUILayout.Width(60)))
                {
                    SelectArea(faceSelection.Areas[i].Key);
                    isEditing = true; // Enter edit mode when selecting an area
                }

                // Disable GUI elements again if this area is not selected
                GUI.enabled = isSelectedArea;

                // Draw the delete button
                if (GUILayout.Button("X", GUILayout.Width(20)))
                {
                    DeleteArea(faceSelection.Areas[i].Key);
                    i--; // Adjust the index after deletion
                }

                // Re-enable GUI for the next iteration
                GUI.enabled = true;

                GUILayout.EndHorizontal(); // End the horizontal layout
            }
        }
    }

    private void RenameArea(string oldName, string newName)
    {
        if (string.IsNullOrEmpty(newName)) return;

        var area = faceSelection.GetAreaByKey(oldName);
        if (area != null && faceSelection.GetAreaByKey(newName) == null)
        {
            area.AreaName = newName;
            faceSelection.selectedKey = newName;
            // Update the area entry key in the list
            for (int i = 0; i < faceSelection.Areas.Count; i++)
            {
                if (faceSelection.Areas[i].Key == oldName)
                {
                    faceSelection.Areas[i] = new FaceSelection.AreaEntry(newName, area);
                    break;
                }
            }

            Debug.Log($"Renamed area: {oldName} to {newName}");
            MarkDirtyAndUpdate();
        }
    }

    private void DeleteArea(string areaName)
    {
        var areaToRemove = faceSelection.GetAreaByKey(areaName);
        if (areaToRemove != null)
        {
            faceSelection.Areas.RemoveAll(a => a.Key == areaName);
            Debug.Log($"Deleted area: {areaName}");
            MarkDirtyAndUpdate();
        }
    }

    private void SaveNewSelection()
    {
        hasUnsavedChanges = false;
        string areaName = GenerateNewAreaName();
        var area = CreateArea(areaName);

        AddAreaEntry(areaName, area);
        Debug.Log($"Saved Faces for {areaName}");
        MarkDirtyAndUpdate();
    }

    private string GenerateNewAreaName()
    {
        int i = 1;
        while (faceSelection.GetAreaByKey($"New Area {i}") != null && i <= 1000)
        {
            i++;
        }

        return $"New Area {i}";
    }

    private Area CreateArea(string areaName)
    {
        return new Area()
        {
            AreaName = areaName,
            TrianglesInSelection = selectedTriangles.ToList()
        };
    }

    private void AddAreaEntry(string areaName, Area area)
    {
        faceSelection.Areas.Add(new FaceSelection.AreaEntry(areaName, area));
        faceSelection.selectedKey = areaName;
    }

    private void SaveSelectedFaces()
    {
        var area = CreateArea(faceSelection.selectedKey);
        faceSelection.GetAreaByKey(faceSelection.selectedKey).TrianglesInSelection = area.TrianglesInSelection;
        Debug.Log($"Saved Faces for {faceSelection.selectedKey}");
        MarkDirtyAndUpdate();
        hasUnsavedChanges = false; // Reset the flag after saving
    }

    private void MarkDirtyAndUpdate()
    {
        EditorUtility.SetDirty(faceSelection);
        serializedObject.Update();
    }

    private void ClearSelection()
    {
        hasUnsavedChanges = true;
        selectedTriangles.Clear();
        hitpoint = Vector3.zero;
        Debug.Log("Selection Cleared");
    }

    private void SelectArea(string areaName)
    {
        hasUnsavedChanges = false;
        faceSelection.selectedKey = areaName;

        var savedTriangles = faceSelection.GetAreaByKey(areaName);
        if (savedTriangles != null)
        {
            selectedTriangles = new List<int>(savedTriangles.TrianglesInSelection);
            Debug.Log($"Selected {areaName} {selectedTriangles.Count}");
        }
        else
        {
            Debug.LogWarning($"Area {areaName} not found");
        }
    }

    public static Vector3 CalculateFaceNormal(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        return Vector3.Cross((v1 - v0).normalized, (v2 - v0).normalized).normalized;
    }
}
