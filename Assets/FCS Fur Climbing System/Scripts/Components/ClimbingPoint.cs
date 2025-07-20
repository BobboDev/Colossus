// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections;
using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class ClimbingPoint : MonoBehaviour
    {
        public Triangle triangle;

        public bool rotateToNormalizedRotation = true;

        public FCSTypes.UpdateMode updateMode = FCSTypes.UpdateMode.LATE_UPDATE;

        [HideInInspector] public ObjectVelocity objectVelocity;

        public static ClimbingPoint CreateClimbingPoint(Transform fur)
        {
            GameObject point = new GameObject();
            ClimbingPoint cPoint = point.AddComponent<ClimbingPoint>();
            point.transform.parent = fur.transform;
            point.name = "Climbing Point " + cPoint.GetHashCode();
            return cPoint;
        }

        public static ClimbingPoint CreateClimbingPointFromTriangle(Triangle triangle)
        {
            GameObject point = new GameObject();
            ClimbingPoint cPoint = point.AddComponent<ClimbingPoint>();
            point.name = "Climbing Point " + cPoint.GetHashCode();
            cPoint.triangle = triangle;
            return cPoint;
        }

        void TriangleUpdate()
        {
            if (triangle != null)
            {
                if (triangle.GetOwner() != null)
                {
                    transform.position = triangle.GetTrianglePositionInWorldSpace();

                    PrismProjection.DebugTriangleRealtime(triangle, Color.yellow);

                    if (rotateToNormalizedRotation)
                    {
                        transform.rotation = triangle.GetAbstractTriangleRotation();
                    }
                }
            }
        }

        public void Resync()
        {
            TriangleUpdate();
        }

        /// <summary>
        /// Make an object become a child of this ClimbingPoint. This makes the object follow the mesh triangle that this ClimbingPoint is attached to.
        /// </summary>
        public void AttachObject(Transform obj, bool worldPositionStays = true)
        {
            TriangleUpdate();
            obj.SetParent(transform, worldPositionStays);
        }

        /// <summary>
        /// Make an object become a child of this ClimbingPoint. This makes the object follow the mesh triangle that this ClimbingPoint is attached to.
        /// </summary>
        public void AttachObject(GameObject obj, bool worldPositionStays = true)
        {
            AttachObject(obj.transform, worldPositionStays);
        }

        private void Awake()
        {
            objectVelocity = gameObject.AddComponent<ObjectVelocity>();    
        }

        private void Start()
        {
            StartCoroutine(IESurfaceUpdate());
        }

        private void FixedUpdate()
        {
            if (updateMode == FCSTypes.UpdateMode.FIXED_UPDATE)
            {
                TriangleUpdate();
            }
        }

        private void LateUpdate()
        {
            if (updateMode == FCSTypes.UpdateMode.LATE_UPDATE)
            {
                TriangleUpdate();
            }
        }

        private void Update()
        {
            if (updateMode == FCSTypes.UpdateMode.UPDATE)
            {
                TriangleUpdate();
            }
        }

        IEnumerator IESurfaceUpdate()
        {
            while (true)
            {
                if (updateMode == FCSTypes.UpdateMode.IENUMERATOR)
                {
                    TriangleUpdate();
                }
                yield return null;
            }
        }
    }
}