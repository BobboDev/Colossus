// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;
using UnityEngine.Events;

namespace VinforlabTeam.FurClimbingSystem
{
    [RequireComponent(typeof(Raycaster))]
    public class Sensor : MonoBehaviour
    {
        public Raycaster raycaster;

        [System.Serializable]
        public class SensorEvents
        {
            public UnityEvent onProviderUndetectedStay = new UnityEvent();
            public UnityEvent onProviderDetectedStay = new UnityEvent();
        }

        [System.Serializable]
        public class Debugging
        {
            public bool isDetecting = false;
        }

        private Raycaster.TriangleHit triangleHit;
        private bool isDetecting = false;

        public SensorEvents sensorEvents = new SensorEvents();
        public Debugging debugging;

        public bool IsDetecting
        {
            get
            {
                return isDetecting;
            }
        }

        public Raycaster.TriangleHit TriangleHit
        {
            get
            {
                return triangleHit;
            }
        }

        private void Start()
        {
            if (raycaster == null)
                raycaster = GetComponent<Raycaster>();
        }

        private void Update()
        {
            triangleHit = raycaster.Raycast();
            isDetecting = triangleHit.triangleProviderHit != null;
            debugging.isDetecting = isDetecting;

            if (isDetecting)
            {
                sensorEvents.onProviderDetectedStay.Invoke();
            }
            else
            {
                sensorEvents.onProviderUndetectedStay.Invoke();
            }
        }
    }
}