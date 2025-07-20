// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections;
using UnityEngine;
using UnityEngine.Events;

namespace VinforlabTeam.FurClimbingSystem
{
    public class TriangleProvider : MonoBehaviour
    {
        [HideInInspector] public Mesh sharedMesh;

        public MeshCollider targetMeshCollider;
        public FCSTypes.UpdateMode updateMode = FCSTypes.UpdateMode.LATE_UPDATE;

        [HideInInspector] public Vector3[] realTimeVertices;


        [System.Serializable]
        public class Debugging
        {
            public Vector3[] realTimeVertices;
        }

        [System.Serializable]
        public class ClimbingEvents
        {
            public UnityEvent OnTriangleProviderDetected = new UnityEvent();
            public UnityEvent OnTriangleProviderDetectionLeft = new UnityEvent();
            public UnityEvent OnTriangleProviderEnterEvent = new UnityEvent();
            public UnityEvent OnTriangleProviderLeaveEvent = new UnityEvent();
        }

        [System.Serializable]
        public class WalkerEvents
        {
            public UnityEvent OnWalkerTriangleProviderEnterEvent = new UnityEvent();
            public UnityEvent OnWalkerTriangleProviderLeaveEvent = new UnityEvent();
        }

        [System.Serializable]
        public class Support
        {
            public bool exoticObjectMode = false;
        }

        public ClimbingEvents climbingEvents = new ClimbingEvents();
        public WalkerEvents walkerEvents = new WalkerEvents();
        [HideInInspector]
        public Support support = new Support();

        public Debugging debugging;

        [Multiline]
        public string providerTag;

        private void Start()
        {
            if (IsExoticObject())
            {
                support.exoticObjectMode = true;
                this.enabled = false;
                return;
            }

            sharedMesh = targetMeshCollider.sharedMesh;
            sharedMesh.RecalculateBounds();
            realTimeVertices = sharedMesh.vertices;
            StartCoroutine(RecalculateBounds());
        }

        /// <summary>
        /// Checks if an object supports or needs to use triangle motion tracking.
        /// </summary>
        /// <returns></returns>
        public bool IsExoticObject()
        {
            return GetComponent<MeshCollider>() == null;
        }

        private void Recalculate()
        {
            sharedMesh = targetMeshCollider.sharedMesh;
            //sharedMesh.RecalculateBounds();
            realTimeVertices = sharedMesh.vertices;
        }

        IEnumerator RecalculateBounds()
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

        private void Update()
        {
            debugging.realTimeVertices = realTimeVertices;

            if(updateMode == FCSTypes.UpdateMode.UPDATE)
            {
                Recalculate();
            }


            float sum = Mathf.Floor( Mathf.Floor(transform.localScale.x + transform.localScale.y + transform.localScale.z) / 3 );
            if (sum != Mathf.Floor( transform.localScale.x ) || sum != Mathf.Floor(transform.localScale.y ) || sum != Mathf.Floor(transform.localScale.z))
            {
                Debug.LogWarning("Non-uniform scale detected! "); 
            }
        }

        private void FixedUpdate()
        {
            if (updateMode == FCSTypes.UpdateMode.FIXED_UPDATE)
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

        public Triangle GetTriangleFromRaycastIndex(int triangleIndex)
        {
            int[] triangles = sharedMesh.triangles;
            Vector3[] vertexes = sharedMesh.vertices;
            Triangle triangle = new Triangle(
                    triangles[triangleIndex * 3 + 0],
                    triangles[triangleIndex * 3 + 1],
                    triangles[triangleIndex * 3 + 2],
                    vertexes,
                    targetMeshCollider.transform,
                    sharedMesh,
                    this
                );
            return triangle;
        }
    }
}