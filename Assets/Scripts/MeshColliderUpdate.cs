using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Overhang;
using System;

public class MeshColliderUpdate : MonoBehaviour
{
    SkinnedMeshRenderer meshRenderer;
    ClimbableMesh climbableMesh;
    MeshCollider col;
    Mesh colliderMesh;

    // Start is called before the first frame update
    void Start()
    {
        meshRenderer = GetComponent<SkinnedMeshRenderer>();
        col = GetComponent<MeshCollider>();
        climbableMesh = GetComponent<ClimbableMesh>();
        UpdateCollider();
    }

    void FixedUpdate()
    {
        UpdateCollider();
    }

    public void UpdateCollider()
    {
        // Destroy(colliderMesh);
        colliderMesh = new();
        if (meshRenderer != null)
        {
            meshRenderer.BakeMesh(colliderMesh);
        }
        col.sharedMesh = colliderMesh;
    }
}