using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Area
{
    public string AreaName;
    public List<int> TrianglesInSelection;
}

public class FaceSelection : MonoBehaviour
{
    public SkinnedMeshRenderer SkinnedMeshRenderer;
    public MeshFilter MeshFilter;

    // Use a list of key-value pairs to mimic a dictionary
    [SerializeField]
    public List<AreaEntry> Areas = new List<AreaEntry>();

    [HideInInspector]
    public string selectedKey;

    private void Awake()
    {
        SkinnedMeshRenderer = GetComponent<SkinnedMeshRenderer>();
        MeshFilter = GetComponent<MeshFilter>();
    }
    // Class to represent a key-value pair
    [System.Serializable]
    public class AreaEntry
    {
        public string Key;
        public Area Value;

        public AreaEntry(string key, Area value)
        {
            Key = key;
            Value = value;
        }
    }

    // Utility method to get an area by key
    public Area GetAreaByKey(string key)
    {
        foreach (var areaEntry in Areas)
        {
            if (areaEntry.Key == key)
            {
                return areaEntry.Value;
            }
        }
        return null; // Return null if the key is not found
    }
}