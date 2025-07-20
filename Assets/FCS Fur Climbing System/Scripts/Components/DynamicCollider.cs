// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections;
using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class DynamicCollider : MonoBehaviour
    {
        [HideInInspector] public Mesh bakedMesh;

        private SkinnedMeshRenderer meshRenderer;
        private MeshCollider meshCollider;

        private Vector3[] _baseVertices;

        public FCSTypes.UpdateMode updateMode = FCSTypes.UpdateMode.LATE_UPDATE;
        public float ScaleX = 1.0f;
        public float ScaleY = 1.0f;
        public float ScaleZ = 1.0f;

        public bool recalculateNormals, recalculateBounds, recalculateTangents;


        void Start()
        {
            bakedMesh = new Mesh();
            meshRenderer = GetComponent<SkinnedMeshRenderer>();
            meshCollider = GetComponent<MeshCollider>();

            if (meshRenderer == null)
            {
                enabled = false;
                return;
            }

            StartCoroutine(RecalculateMesh());
        }

        public void Resync()
        {
            if(meshRenderer != null && meshCollider != null)
            {
                Recalculate();
            }
        }

        SkinnedMeshRenderer skinnedMeshRenderer;
        private void Recalculate()
        {
            skinnedMeshRenderer = meshRenderer;
            skinnedMeshRenderer.BakeMesh(bakedMesh);

            if (recalculateNormals)
                bakedMesh.RecalculateNormals();
            if (recalculateBounds)
                bakedMesh.RecalculateBounds();
            if (recalculateTangents)
                bakedMesh.RecalculateTangents();

            if (ScaleX != 1 || ScaleY != 1 || ScaleZ != 1)
            {
                _baseVertices = bakedMesh.vertices;

                var vertices = new Vector3[_baseVertices.Length];
                for (var i = 0; i < vertices.Length; i++)
                {
                    var vertex = _baseVertices[i];
                    vertex.x = vertex.x * ScaleX;
                    vertex.y = vertex.y * ScaleY;
                    vertex.z = vertex.z * ScaleZ;
                    vertices[i] = vertex;
                }
                bakedMesh.vertices = vertices;
            }

            meshCollider.sharedMesh = bakedMesh;
        }


        IEnumerator RecalculateMesh()
        {
            while (true)
            {
                if (updateMode == FCSTypes.UpdateMode.IENUMERATOR)
                {
                    Recalculate();
                }
                yield return null;
            }
        }


        private void FixedUpdate()
        {
            if (updateMode == FCSTypes.UpdateMode.FIXED_UPDATE)
            {
                Recalculate();
            }
        }

        private void Update()
        {
            if (updateMode == FCSTypes.UpdateMode.UPDATE)
            {
                Recalculate();
            }
        }

        private void LateUpdate()
        {
            if (updateMode == FCSTypes.UpdateMode.LATE_UPDATE)
            {
                Recalculate();
            }
        }


    }
}