// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;
using UnityEngine.Events;

namespace VinforlabTeam.FurClimbingSystem
{
    public class SlopeChecker : MonoBehaviour
    {
        public bool checkerEnabled = true;
        public Raycaster raycaster;

        [System.Serializable]
        public class CheckerSettings
        {
            public float targetSlope = 20;
            public FCSTypes.NumberComparison slopeCheckLogic = FCSTypes.NumberComparison.LESS_THAN;
            public FCSTypes.Axis slopeWorldAxis = FCSTypes.Axis.UP;
        }

        [System.Serializable]
        public class SlopeCheckerEvents
        {
            public UnityEvent OnSlopeDetect = new UnityEvent();
        }

        [System.Serializable]
        public class Debugging
        {
            public float currentSlope;
        }

        private Raycaster.TriangleHit triangleHit;

        public CheckerSettings slopeCheckerSettings = new CheckerSettings();
        public SlopeCheckerEvents slopeCheckerEvents = new SlopeCheckerEvents();
        public Debugging debugging = new Debugging();

        public Raycaster.TriangleHit LastTriangleHit
        {
            get
            {
                return triangleHit;
            }
        }

        public float LastSlope
        {
            get
            {
                return debugging.currentSlope;
            }
        }

        public void Check()
        {
            triangleHit = raycaster.Raycast();

            if (triangleHit.hitTransform == null)
            {

                return;
            }

            float slopeAngle = Vector3.Angle(triangleHit.normal, FCSTypes.AxisEnumToWorldVector3(slopeCheckerSettings.slopeWorldAxis));

            debugging.currentSlope = slopeAngle;

            if (slopeCheckerSettings.slopeCheckLogic == FCSTypes.NumberComparison.EQUAL) { 
                if (slopeAngle >= slopeCheckerSettings.targetSlope)
                {
                    slopeCheckerEvents.OnSlopeDetect.Invoke();
                }
            }

            if (slopeCheckerSettings.slopeCheckLogic == FCSTypes.NumberComparison.LESS_THAN)
            {
                if (slopeAngle < slopeCheckerSettings.targetSlope)
                {
                    slopeCheckerEvents.OnSlopeDetect.Invoke();
                }
            }

            if (slopeCheckerSettings.slopeCheckLogic == FCSTypes.NumberComparison.GREATER_THAN)
            {
                if (slopeAngle > slopeCheckerSettings.targetSlope)
                {
                    slopeCheckerEvents.OnSlopeDetect.Invoke();
                }
            }
        }

        // Update is called once per frame
        void Update()
        {
            if(checkerEnabled)
                Check();
        }
    }
}