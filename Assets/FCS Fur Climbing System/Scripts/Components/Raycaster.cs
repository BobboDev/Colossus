// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class Raycaster : MonoBehaviour
    {
        public LayerMask layerMask;
        public FCSTypes.Axis raycastAxis = FCSTypes.Axis.FORWARD;
        public float distance = 3;

        public class TriangleHit
        {
            public RaycastHit raycastHit;
            public Vector3 normal = Vector3.zero, point;
            public TriangleProvider triangleProviderHit;
            public Transform hitTransform = null;
            public int triangleIndex = -999;
            public bool raycast = false;
        }

#if UNITY_EDITOR
        void OnDrawGizmosSelected()
        {
            Gizmos.DrawLine(transform.position, transform.position + FCSTypes.AxisEnumToLocalVector3(transform, raycastAxis) * distance);
        }
#endif

        public TriangleHit Raycast()
        {
            Transform raycaster = transform;
            RaycastHit raycastHit;
            TriangleHit triangleHit = new TriangleHit();

            if (raycastAxis == FCSTypes.Axis.DOWN)
            {
                if (Physics.Raycast(raycaster.position, -raycaster.up, out raycastHit, distance/*3*/, layerMask))
                {
                    triangleHit.normal = raycastHit.normal;
                    triangleHit.triangleIndex = raycastHit.triangleIndex;
                    triangleHit.raycast = true;
                    triangleHit.hitTransform = raycastHit.transform;
                    triangleHit.triangleProviderHit = triangleHit.hitTransform.GetComponent<TriangleProvider>();
                    triangleHit.point = raycastHit.point;
                }

                Debug.DrawLine(raycaster.position, raycaster.position + -raycaster.up * distance);
            }

            if (raycastAxis == FCSTypes.Axis.UP)
            {
                if (Physics.Raycast(raycaster.position, raycaster.up, out raycastHit, distance/*3*/, layerMask))
                {
                    triangleHit.normal = raycastHit.normal;
                    triangleHit.triangleIndex = raycastHit.triangleIndex;
                    triangleHit.raycast = true;
                    triangleHit.hitTransform = raycastHit.transform;
                    triangleHit.triangleProviderHit = triangleHit.hitTransform.GetComponent<TriangleProvider>();
                    triangleHit.point = raycastHit.point;
                }

                Debug.DrawLine(raycaster.position, raycaster.position + raycaster.up * distance);
            }

            if (raycastAxis == FCSTypes.Axis.RIGHT)
            {
                if (Physics.Raycast(raycaster.position, raycaster.right, out raycastHit, distance/*3*/, layerMask))
                {
                    triangleHit.normal = raycastHit.normal;
                    triangleHit.triangleIndex = raycastHit.triangleIndex;
                    triangleHit.raycast = true;
                    triangleHit.hitTransform = raycastHit.transform;
                    triangleHit.triangleProviderHit = triangleHit.hitTransform.GetComponent<TriangleProvider>();
                    triangleHit.point = raycastHit.point;
                }

                Debug.DrawLine(raycaster.position, raycaster.position + raycaster.right * distance);
            }


            if (raycastAxis == FCSTypes.Axis.LEFT)
            {
                if (Physics.Raycast(raycaster.position, -raycaster.right, out raycastHit, distance/*3*/, layerMask))
                {
                    triangleHit.normal = raycastHit.normal;
                    triangleHit.triangleIndex = raycastHit.triangleIndex;
                    triangleHit.raycast = true;
                    triangleHit.hitTransform = raycastHit.transform;
                    triangleHit.triangleProviderHit = triangleHit.hitTransform.GetComponent<TriangleProvider>();
                    triangleHit.point = raycastHit.point;
                }

                Debug.DrawLine(raycaster.position, raycaster.position + -raycaster.right * distance);
            }

            if (raycastAxis == FCSTypes.Axis.FORWARD)
            {
                if (Physics.Raycast(raycaster.position, raycaster.forward, out raycastHit, distance/*3*/, layerMask))
                {
                    triangleHit.normal = raycastHit.normal;
                    triangleHit.triangleIndex = raycastHit.triangleIndex;
                    triangleHit.raycast = true;
                    triangleHit.hitTransform = raycastHit.transform;
                    triangleHit.triangleProviderHit = triangleHit.hitTransform.GetComponent<TriangleProvider>();
                    triangleHit.point = raycastHit.point;
                }

                Debug.DrawLine(raycaster.position, raycaster.position + raycaster.forward * distance);
            }

            if (raycastAxis == FCSTypes.Axis.BACKWARD)
            {
                if (Physics.Raycast(raycaster.position, -raycaster.forward, out raycastHit, distance/*3*/, layerMask))
                {
                    triangleHit.normal = raycastHit.normal;
                    triangleHit.triangleIndex = raycastHit.triangleIndex;
                    triangleHit.raycast = true;
                    triangleHit.hitTransform = raycastHit.transform;
                    triangleHit.triangleProviderHit = triangleHit.hitTransform.GetComponent<TriangleProvider>();
                    triangleHit.point = raycastHit.point;
                }

                Debug.DrawLine(raycaster.position, raycaster.position + -raycaster.forward * distance);
            }

            if (triangleHit.point != Vector3.zero)
                Debug.DrawLine(raycaster.position, triangleHit.point, Color.red);

            return triangleHit;
        }
    }
}