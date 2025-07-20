using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class MeshManager : MonoBehaviour
{
    public List<SkinnedMeshRenderer> skinnedMeshRenderers = new();
    public List<MeshRenderer> meshRenderers = new();

    void Awake()
    {
        skinnedMeshRenderers = FindObjectsOfType<SkinnedMeshRenderer>().ToList();
        meshRenderers = FindObjectsOfType<MeshRenderer>().ToList();
    }
}
