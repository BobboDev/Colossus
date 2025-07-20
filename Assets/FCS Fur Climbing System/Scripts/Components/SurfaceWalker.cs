// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections;
using UnityEngine;
using UnityEngine.Events;

namespace VinforlabTeam.FurClimbingSystem {
    public class SurfaceWalker : MonoBehaviour
    {
        [SerializeField]
        private bool walkerEnabled = true;

        public bool WalkerEnabled
        {
            get
            {
                return walkerEnabled;
            }
            set
            {
                walkerEnabled = value;
            }
        }

        [System.Serializable]
        public class RaycastSettings
        {
            public Raycaster raycaster;
            public LayerMask layerMask;
            public FCSTypes.Axis raycastAxis = FCSTypes.Axis.DOWN;
            public float distance = 3;
        }


        [System.Serializable]
        public class PositionSettings
        {
            public bool followSurfaceHeight = false;
            public FCSTypes.Pivot heightPivot = FCSTypes.Pivot.Y;
            public float followSpeed = 8f;
            public float height = 2f;
        }

        [System.Serializable]
        public class ClimbingEvents
        {
            public UnityEvent OnWalkerStartEvent = new UnityEvent();
            public UnityEvent OnWalkerStopEvent = new UnityEvent();
            public UnityEvent OnWalkerTriangleChange = new UnityEvent();
            public UnityEvent OnWalkerTriangleProviderChange = new UnityEvent();
        }


        [System.Serializable]
        public class AdvancedSettings
        {
            public FCSTypes.UpdateMode updateMode = FCSTypes.UpdateMode.LATE_UPDATE;
            public FCSTypes.UpdateMode climbingPointUpdateMode = FCSTypes.UpdateMode.LATE_UPDATE;
            public bool useScalePatcherOnClimbingPoint = true;
            public bool forceUpdateDynamicCollider = false;
            public bool forceUpdateClimbingPoint = true;
        }


        [System.Serializable]
        public class SlopeSettings
        {
            public bool useMaxSlopAngle = true;
            public FCSTypes.Axis worldAxis = FCSTypes.Axis.UP;
            public float maxSlopeAngle = 45f;
        }

        [System.Serializable]
        public class Debugging
        {
            public float currentSlopeAngle;
        }

        public RaycastSettings raycastSettings = new RaycastSettings();
        public PositionSettings positionSettings = new PositionSettings();
        public AdvancedSettings advancedSettings = new AdvancedSettings();
        public SlopeSettings slopeSettings = new SlopeSettings();
        public ClimbingEvents climbingEvents = new ClimbingEvents();
        public Debugging debugging = new Debugging();

        private Transform defaultParentPlayer;
        private TriangleProvider surfaceTriangleProvider;
        private ClimbingPoint climbingPoint;
        private Triangle currentTriangle;
        private int currentTriangleIndex;

        private GameObject surfaceWalkerAux;
        private FCSFollower followerAux;

        private bool isWalkingSurface = false;

        private bool detected = false;

        private float currentSlopeAngle;

        public float CurrentSlopeAngle
        {
            get
            {
                return currentSlopeAngle;
            }
        }

        public bool IsWalkingInMesh
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

        private void Start()
        {
            surfaceWalkerAux = FCSAux.CreateAux("Surface Walker Aux ").gameObject;
            followerAux = FCSFollower.CreateFollower();
            followerAux.targetWalker = this;
            StartCoroutine(IEWalkUpdate());
        }

        /// <summary>
        /// With this function you can create a Climbing Point based on the player's current triangle, if you want to attach something to the triangle.
        /// </summary>
        public ClimbingPoint CreateClimbingPointForCurrentTriangle()
        {
            return ClimbingPoint.CreateClimbingPointFromTriangle(currentTriangle);
        }

        private bool HasSurfaceGrab(out SurfaceGrab surfaceGrab)
        {
            return TryGetComponent<SurfaceGrab>(out surfaceGrab);
        }

        private void AttachTriangle(TriangleProvider surfaceTriangleProvider,  int triangleIndex)
        {
            if (surfaceTriangleProvider.support.exoticObjectMode) return;

            Triangle tri = surfaceTriangleProvider.GetTriangleFromRaycastIndex(triangleIndex);
            currentTriangle = tri;
            climbingPoint.triangle = tri;

            if (currentTriangleIndex != triangleIndex)
            {
                transform.SetParent(null, true);
                climbingPoint.triangle = tri;
                currentTriangleIndex = triangleIndex;
                climbingPoint.transform.position = tri.GetTrianglePositionInWorldSpace();
                climbingPoint.transform.rotation = tri.GetAbstractTriangleRotation();
                transform.SetParent(climbingPoint.transform, true);
                climbingEvents.OnWalkerTriangleChange.Invoke();
            }
        }


        public void Drop()
        {
            if (!isWalkingSurface) return;
            TriangleProvider lastTriangleProvider = surfaceTriangleProvider; 
            surfaceTriangleProvider = null;
            currentTriangle = null;
            currentTriangleIndex = -999;
            isWalkingSurface = false;
            transform.SetParent(defaultParentPlayer);
            climbingPoint.transform.SetParent(null);
            detected = false;
            climbingEvents.OnWalkerStopEvent.Invoke();
            if (lastTriangleProvider != null)
            {
                lastTriangleProvider.walkerEvents.OnWalkerTriangleProviderLeaveEvent.Invoke();
            }
        }

        private void Grab(Raycaster.TriangleHit triangleHit)
        {
            if (climbingPoint == null)
            {
                climbingPoint = ClimbingPoint.CreateClimbingPoint(followerAux.transform);// triangleHit.triangleProviderHit.transform);
                climbingPoint.updateMode = advancedSettings.climbingPointUpdateMode;
                if (advancedSettings.useScalePatcherOnClimbingPoint)
                {
                    climbingPoint.gameObject.AddComponent<ScalePatcher>().useStartScale = false;
                }
            }

            if (triangleHit != null)
            {
                if (triangleHit.hitTransform != null)
                {

                    if (!isWalkingSurface)
                    {
                        isWalkingSurface = true;
                        detected = true;
                        defaultParentPlayer = transform.parent;
                        climbingEvents.OnWalkerStartEvent.Invoke();
                    }

                    if (triangleHit.triangleProviderHit != surfaceTriangleProvider)
                    {
                        TriangleProvider previousProvider = surfaceTriangleProvider;
                        surfaceTriangleProvider = triangleHit.triangleProviderHit;

                        transform.SetParent(null);
                        followerAux.FollowBasic();
                        followerAux.Follow();
                        climbingPoint.transform.SetParent(followerAux.child.transform/*surfaceTriangleProvider.transform*/, true);
                        climbingEvents.OnWalkerTriangleProviderChange.Invoke();
                        surfaceTriangleProvider.walkerEvents.OnWalkerTriangleProviderEnterEvent.Invoke();

                        if (CurrentTriangleProvider != null)
                        {
                            if (previousProvider != CurrentTriangleProvider && previousProvider != null)
                                previousProvider.walkerEvents.OnWalkerTriangleProviderLeaveEvent.Invoke();
                        }
                    }

                    AttachTriangle(triangleHit.triangleProviderHit, triangleHit.triangleIndex);
                }
            }

            if (positionSettings.followSurfaceHeight)
            {
                Vector3 triangleNormal = currentTriangle.ComputeNormalizePosition();
                Vector3 localPos = transform.localPosition;
                if(positionSettings.heightPivot == FCSTypes.Pivot.Y)
                    localPos.y = Mathf.Lerp(localPos.y, transform.InverseTransformPoint(triangleHit.point + triangleNormal * positionSettings.height).y, Time.deltaTime * positionSettings.followSpeed);
                if(positionSettings.heightPivot == FCSTypes.Pivot.X)
                    localPos.x = Mathf.Lerp(localPos.x, transform.InverseTransformPoint(triangleHit.point + triangleNormal * positionSettings.height).x, Time.deltaTime * positionSettings.followSpeed);
                if (positionSettings.heightPivot == FCSTypes.Pivot.Z)
                    localPos.z = Mathf.Lerp(localPos.z, transform.InverseTransformPoint(triangleHit.point + triangleNormal * positionSettings.height).z, Time.deltaTime * positionSettings.followSpeed);

                transform.localPosition = localPos;
            }
        }

        public void Resync(Raycaster.TriangleHit triangleHit)
        {
            if (triangleHit == null || triangleHit.hitTransform == null) return;

            DynamicCollider currentDynamicCollider = triangleHit.hitTransform.GetComponent<DynamicCollider>();

            /////// Resync for resolve stuttering
            if (currentDynamicCollider != null)
            {
                if (advancedSettings.forceUpdateDynamicCollider)
                {
                    currentDynamicCollider.Resync();
                }
            }

            if (CurrentClimbingPoint != null)
            {
                if (advancedSettings.forceUpdateClimbingPoint)
                {
                    CurrentClimbingPoint.Resync();
                }
            }
            //////// End Resync
        }

        private void WalkUpdate()
        {
            if (!walkerEnabled)
            {
                Drop();
                return;
            }

            if (climbingPoint != null)
            {
                climbingPoint.updateMode = advancedSettings.climbingPointUpdateMode;
            }

            Raycaster.TriangleHit triangleHit = null;
            SurfaceGrab surfaceGrab;

            surfaceWalkerAux.transform.SetParent(transform.parent, true);

            if (HasSurfaceGrab(out surfaceGrab))
            {
                if (!surfaceGrab.IsOnClimbing)
                {
                    triangleHit = raycastSettings.raycaster.Raycast();

                    Resync(triangleHit);

                    if (slopeSettings.useMaxSlopAngle) {
                        float slopeAngle = Vector3.Angle(triangleHit.normal, FCSTypes.AxisEnumToWorldVector3(slopeSettings.worldAxis));
                        currentSlopeAngle = slopeAngle;
                        debugging.currentSlopeAngle = currentSlopeAngle;
                        if (slopeAngle >= slopeSettings.maxSlopeAngle)
                        {
                            triangleHit.triangleProviderHit = null;
                        }
                    }

                    if (triangleHit.triangleProviderHit != null)
                    {
                        Grab(triangleHit);
                    }
                    else
                    {
                        Drop();
                    }
                }
            }
            else
            {
                triangleHit = raycastSettings.raycaster.Raycast();

                Resync(triangleHit);

                if (slopeSettings.useMaxSlopAngle)
                {
                    float slopeAngle = Vector3.Angle(triangleHit.normal, FCSTypes.AxisEnumToWorldVector3(slopeSettings.worldAxis));
                    currentSlopeAngle = slopeAngle;
                    if (slopeAngle >= slopeSettings.maxSlopeAngle)
                    {
                        triangleHit.triangleProviderHit = null;
                    }
                }

                if (triangleHit.triangleProviderHit != null)
                {
                    Grab(triangleHit);
                }
                else
                {
                    Drop();
                }
            }


        }

        private void Update()
        {
            if (advancedSettings.updateMode == FCSTypes.UpdateMode.UPDATE)
            {
                WalkUpdate();
            }
        }

        private void FixedUpdate()
        {
            if (advancedSettings.updateMode == FCSTypes.UpdateMode.FIXED_UPDATE)
            {
                WalkUpdate();
            }
        }

        private void LateUpdate()
        {
            if (advancedSettings.updateMode == FCSTypes.UpdateMode.LATE_UPDATE)
            {
                WalkUpdate();
            }
        }

        IEnumerator IEWalkUpdate()
        {
            while (true)
            {
                if (advancedSettings.updateMode == FCSTypes.UpdateMode.IENUMERATOR)
                {
                    WalkUpdate();
                }
                yield return null;
            }
        }

    }
}