// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections;
using UnityEngine;
using UnityEngine.Events;

namespace VinforlabTeam.FurClimbingSystem
{
    public class SurfaceGrab : MonoBehaviour
    {

        [HideInInspector] public TriangleProvider surfaceTriangleProvider;

        [System.Serializable]
        public class RaycastSettings
        {
            public Raycaster raycaster;
            public Raycaster raycasterDown;
        }


        [System.Serializable]
        public class RotationSettings
        {
            public bool useNormalRotation = true;
            public float normalRotationSpeed = 7f;
            public FCSTypes.Axis normalRotateAxis = FCSTypes.Axis.BACKWARD;
        }


        [System.Serializable]
        public class PositionSettings
        {
            public bool useSurfaceHeight = true;
            public float height = 0.1f;
            public float surfaceGrabSpeed = 0.07f;
        }


        [System.Serializable]
        public class ClimbingEvents
        {
            public UnityEvent OnClimbingDetected = new UnityEvent();
            public UnityEvent OnClimbingDetectionLeft = new UnityEvent();
            public UnityEvent OnClimbingStartEvent = new UnityEvent();
            public UnityEvent OnClimbingDropEvent = new UnityEvent();
            public UnityEvent OnClimbingTriangleChange = new UnityEvent();
            public UnityEvent OnClimbingTriangleProviderChange = new UnityEvent();
        }


        [System.Serializable]
        public class AdvancedSettings
        {
            public FCSTypes.UpdateMode updateMode = FCSTypes.UpdateMode.LATE_UPDATE;
            public FCSTypes.UpdateMode climbingPointUpdateMode = FCSTypes.UpdateMode.LATE_UPDATE;
            public bool useScalePatcherOnClimbingPoint = true;
            public bool dropIfLeaveClimbableArea = false;
            public bool forceUpdateDynamicCollider = false;
            public bool forceUpdateClimbingPoint = false;
        }


        public RaycastSettings raycastSettings = new RaycastSettings();
        public PositionSettings positionSettings = new PositionSettings();
        public RotationSettings rotationSettings = new RotationSettings();
        public AdvancedSettings advancedSettings = new AdvancedSettings();
        public ClimbingEvents climbingEvents = new ClimbingEvents();


        private GameObject surfaceAux;
        private FCSFollower followerAux;

        private Transform defaultParentPlayer;
        private Triangle currentTriangle;
        private ClimbingPoint climbingPoint;
        private int currentTriangleIndex;

        private bool isOnClimbing = false;

        private bool firstClimbing = true;

        public bool IsOnClimbing
        {
            get
            {
                return isOnClimbing;
            }
        }

        public bool IsClimbingAvailableNow
        {
            get
            {
                return detected;
            }
        }

        public int CurrentTriangleIndex
        {
            get
            {
                return currentTriangleIndex;
            }
        }

        public Triangle CurrentTriangle
        {
            get
            {
                return currentTriangle;
            }
        }

        public TriangleProvider CurrentTriangleProvider
        {
            get
            {
                return surfaceTriangleProvider;
            }
        }

        public ObjectVelocity CurrentObjectVelocity
        {
            get { return climbingPoint.objectVelocity; }
        }

        public ClimbingPoint CurrentClimbingPoint
        {
            get
            {
                return climbingPoint;
            }
        }

        public Raycaster.TriangleHit CastAndGetTriangleHit()
        {
            return Cast();
        }

        /// <summary>
        /// With this function you can create a Climbing Point based on the player's current triangle, if you want to attach something to the triangle.
        /// </summary>
        public ClimbingPoint CreateClimbingPointForCurrentTriangle()
        {
            return ClimbingPoint.CreateClimbingPointFromTriangle(currentTriangle);
        }

        /// <summary>
        /// This function will cause the player to start climbing an object when available.
        /// </summary>
        public void GrabSurface()
        {
            if (isOnClimbing) return;

            Raycaster.TriangleHit triangleHit = Cast();
            if (triangleHit.raycast)
            {
                if(triangleHit.triangleProviderHit != null)
                {
                    defaultParentPlayer = transform.parent;
                    climbingEvents.OnClimbingStartEvent.Invoke();
                    isOnClimbing = true;
                }
            }
        }

        /// <summary>
        /// This function will make the player drop the climb.
        /// </summary>
        public void Drop()
        {
            if (!isOnClimbing) return;

            currentTriangle = null;
            currentTriangleIndex = -999;
            isOnClimbing = false;
            transform.SetParent(defaultParentPlayer);
            climbingPoint.transform.SetParent(null);
            climbingEvents.OnClimbingDropEvent.Invoke();
            if(CurrentTriangleProvider != null)
            {
                CurrentTriangleProvider.climbingEvents.OnTriangleProviderLeaveEvent.Invoke();
            }

            surfaceTriangleProvider = null;
        }


        private void Start()
        {
            surfaceAux = FCSAux.CreateAux("Climbing Surface Aux").gameObject;
            followerAux = FCSFollower.CreateFollower();
            followerAux.target = this;
            StartCoroutine(IESurfaceUpdate());
        }

        private bool detected = false;
        private TriangleProvider lastTriangleProviderDetected;
        private void Update()
        {
            Raycaster.TriangleHit triangleHit = Cast();
            if (triangleHit.triangleProviderHit != null && !detected)
            {
                climbingEvents.OnClimbingDetected.Invoke();
                triangleHit.triangleProviderHit.climbingEvents.OnTriangleProviderDetected.Invoke();
                lastTriangleProviderDetected = triangleHit.triangleProviderHit;
                detected = true;
            }
            
            if(triangleHit.triangleProviderHit == null && detected)
            {
                climbingEvents.OnClimbingDetectionLeft.Invoke();
                lastTriangleProviderDetected.climbingEvents.OnTriangleProviderDetectionLeft.Invoke();
                lastTriangleProviderDetected = null;
                detected = false;
            }

            if (advancedSettings.updateMode == FCSTypes.UpdateMode.UPDATE)
            {
                SurfaceUpdate();
            }
        }

        private ClimbingPoint CreateClimbingPoint()
        {
            climbingPoint = ClimbingPoint.CreateClimbingPoint(followerAux.transform);
            climbingPoint.updateMode = advancedSettings.climbingPointUpdateMode;
            if( advancedSettings.useScalePatcherOnClimbingPoint)
            {
                climbingPoint.gameObject.AddComponent<ScalePatcher>().useStartScale = false;
            }
            return climbingPoint;
        }


        private void AttachTriangle(int triangleIndex)
        {
            if (surfaceTriangleProvider.support.exoticObjectMode) return;

            Triangle tri = surfaceTriangleProvider.GetTriangleFromRaycastIndex(triangleIndex);
            currentTriangle = tri;
            climbingPoint.triangle = tri;

            if (currentTriangleIndex != triangleIndex)
            {
                transform.SetParent(null);
                climbingPoint.triangle = tri;
                currentTriangleIndex = triangleIndex;
                climbingPoint.transform.position = tri.GetTrianglePositionInWorldSpace();
                climbingPoint.transform.rotation = tri.GetAbstractTriangleRotation();
                transform.SetParent(climbingPoint.transform);
                climbingEvents.OnClimbingTriangleChange.Invoke();
            }
        }


        private Raycaster.TriangleHit Cast()
        {
            Raycaster.TriangleHit triangleHit = raycastSettings.raycaster.Raycast();

            if (triangleHit.hitTransform == null)
            {
                triangleHit = raycastSettings.raycasterDown.Raycast();
            }

            return triangleHit;
        }

        public void Resync(Raycaster.TriangleHit triangleHit)
        {
            DynamicCollider currentDynamicCollider = triangleHit.hitTransform.GetComponent<DynamicCollider>();

            /////// Resync for resolve stuttering
            if (currentDynamicCollider != null)
            {
                if (advancedSettings.forceUpdateDynamicCollider)
                {
                    Debug.Log("Updating dynamic collider...");
                    currentDynamicCollider.Resync();
                }
            }

            if (CurrentClimbingPoint != null)
            {
                if (advancedSettings.forceUpdateClimbingPoint)
                {
                    Debug.Log("Updating climbing point...");
                    CurrentClimbingPoint.Resync();
                }
            }
            //////// End Resync
           
        }

        private void SurfaceUpdate()
        {
            if (!IsOnClimbing) return;

            Raycaster.TriangleHit triangleHit = Cast();

            Resync(triangleHit);

            if (climbingPoint != null)
            {
                climbingPoint.updateMode = advancedSettings.climbingPointUpdateMode;
            }
            if (triangleHit.hitTransform != null )
            {
                if (triangleHit.hitTransform.GetComponent<TriangleProvider>() != surfaceTriangleProvider || firstClimbing)
                {
                    firstClimbing = false;
                    TriangleProvider previousProvider = surfaceTriangleProvider;
                    surfaceTriangleProvider = triangleHit.hitTransform.GetComponent<TriangleProvider>();

                    if (climbingPoint == null)
                    {
                        CreateClimbingPoint();
                    }

                    transform.SetParent(null);
                    followerAux.FollowBasic ();
                    followerAux.Follow();
                    climbingPoint.transform.SetParent(followerAux.child.transform);
                    transform.SetParent(climbingPoint.transform);
                    climbingEvents.OnClimbingTriangleProviderChange.Invoke();
                    surfaceTriangleProvider.climbingEvents.OnTriangleProviderEnterEvent.Invoke();

                    if (CurrentTriangleProvider != null)
                    {
                        if(previousProvider != CurrentTriangleProvider && previousProvider != null)
                            previousProvider.climbingEvents.OnTriangleProviderLeaveEvent.Invoke();
                    }
                }

            }

            if (climbingPoint == null)
            {
                CreateClimbingPoint();
            }

            if (triangleHit.triangleIndex == -999)
            {
                if (advancedSettings.dropIfLeaveClimbableArea)
                {
                    Drop();
                }
                else
                {
                    Debug.LogWarning("SurfaceGrab - It looks like your object has moved out of the climbable area. You can check the option \"dropIfLeaveClimbableArea\".");
                }
            }
            else
            {
                AttachTriangle(triangleHit.triangleIndex);
            }

            if (transform.parent != null)
            {
                surfaceAux.transform.SetParent(transform.parent);
            }

            if (!triangleHit.raycast) return;

            Vector3 axis = -transform.forward;

            if (rotationSettings.normalRotateAxis == FCSTypes.Axis.FORWARD)
            {
                axis = transform.forward;
            }

            if (rotationSettings.normalRotateAxis == FCSTypes.Axis.BACKWARD)
            {
                axis = -transform.forward;
            }

            if (rotationSettings.normalRotateAxis == FCSTypes.Axis.UP)
            {
                axis = transform.up;
            }

            if (rotationSettings.normalRotateAxis == FCSTypes.Axis.DOWN)
            {
                axis = -transform.up;
            }

            if (rotationSettings.normalRotateAxis == FCSTypes.Axis.RIGHT)
            {
                axis = transform.right;
            }

            if (rotationSettings.normalRotateAxis == FCSTypes.Axis.LEFT)
            {
                axis = -transform.right;
            }

            Vector3 newLocalPos = Vector3.zero;
            Vector3 triangleNormal = Vector3.zero;
            Vector3 triangleWorldPosition = Vector3.zero;

            if (!surfaceTriangleProvider.support.exoticObjectMode)
            {
                triangleNormal = currentTriangle.ComputeNormalizePosition();
                triangleWorldPosition = currentTriangle.GetTrianglePositionInWorldSpace();
            }
            else
            {
                triangleNormal = triangleHit.normal;
                triangleWorldPosition = triangleHit.point;
            }

            if (rotationSettings.useNormalRotation)
                transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.FromToRotation(axis, triangleHit.normal) * transform.rotation, Time.deltaTime * rotationSettings.normalRotationSpeed);


            surfaceAux.transform.position = triangleWorldPosition + triangleNormal * positionSettings.height;
            surfaceAux.transform.rotation = Quaternion.FromToRotation(surfaceAux.transform.up, triangleNormal) * surfaceAux.transform.rotation;

            newLocalPos = transform.localPosition;
            newLocalPos.y = surfaceAux.transform.localPosition.y;

            if (transform.parent != null && positionSettings.useSurfaceHeight)
                transform.localPosition = Vector3.Lerp(transform.localPosition, newLocalPos, positionSettings.surfaceGrabSpeed);
        }

        private void FixedUpdate()
        {
            if(advancedSettings.updateMode == FCSTypes.UpdateMode.FIXED_UPDATE)
            {
                SurfaceUpdate();
            }
        }

        private void LateUpdate()
        {
            if (advancedSettings.updateMode == FCSTypes.UpdateMode.LATE_UPDATE)
            {
                SurfaceUpdate();
            }
        }

        IEnumerator IESurfaceUpdate()
        {
            while (true)
            {
                if (advancedSettings.updateMode == FCSTypes.UpdateMode.IENUMERATOR)
                {
                    SurfaceUpdate();
                }
                yield return null;
            }
        }
    }
}