using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(TriangleCreator))]
public class TriangleCreatorEditor : Editor
{
    private void OnSceneGUI()
    {
        TriangleCreator triangleCreator = (TriangleCreator)target;

        Handles.BeginGUI();
        GUILayout.Label("Create Shaded Triangle");

        if (GUILayout.Button("Create Triangle"))
        {
            // triangleCreator.CreateTriangle();
        }

        Handles.EndGUI();
    }
}

