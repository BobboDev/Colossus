// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class ObjectVelocity : MonoBehaviour
    {
        [System.Serializable]
        public class VelocityInfo
        {
            [SerializeField] private Vector3 velocity;
            [SerializeField] private Vector3 velocityNormalized;
            [SerializeField] private float magnitude;

            public Vector3 Velocity
            {
                get { return velocity; }
                set { velocity = value; }
            }

            public Vector3 VelocityNormalized
            {
                get { return velocityNormalized; }
                set { velocityNormalized = value; }
            }

            public float Magnitude
            {
                get { return magnitude; }
                set { magnitude = value; }
            }
        }

        [System.Serializable]
        public class VelocityDirectionInfo
        {
            [SerializeField] public Vector3 currentPosition, delta;

            /// <summary>
            /// Get just the normalized of the calculated velocity delta. Useful for you to calculate the direction manually.
            /// </summary>
            /// <param name="distanceMultiplier">Specifies a distance multiplier. By default the value is 1.</param>
            /// <param name="inverse">Specifies whether you want to get the inverse normalized.</param>
            public Vector3 GetDirection(float distanceMultiplier = 1, bool inverse = false)
            {
                float normalMultiplier = (inverse) ? -1 : 1;
                return currentPosition + (normalMultiplier * delta.normalized) * distanceMultiplier;
            }

            /// <summary>
            /// Get just the normalized of the calculated velocity delta. Useful for you to calculate the direction manually.
            /// </summary>
            /// <param name="inverse">Specifies whether you want to get the inverse normalized.</param>
            public Vector3 GetRawDirection(bool inverse = false)
            {
                float multiplier = (inverse) ? -1 : 1;
                return (multiplier * delta.normalized);
            }
        }

        public VelocityInfo velocityInfo = new VelocityInfo();
        [HideInInspector] public VelocityDirectionInfo velocityDirectionInfo = new VelocityDirectionInfo();

        private Vector3 lastPosition;
        private Vector3 directionSmoothDebug, inverseDirectionSmoothDebug;

        void Start()
        {
            lastPosition = transform.position;

        }

        Vector3 prevLocation;
        private Vector3 difference;


        void Update()
        {
            difference = transform.position - prevLocation;
            prevLocation = transform.position;

            Vector3 velocity = (transform.position - lastPosition) / Time.deltaTime;
            velocityInfo.Velocity = velocity;
            velocityInfo.VelocityNormalized = velocity.normalized;
            velocityInfo.Magnitude = ((transform.position - lastPosition).magnitude) / Time.deltaTime;

            velocityDirectionInfo.currentPosition = transform.position;
            velocityDirectionInfo.delta = difference;

            directionSmoothDebug = Vector3.Lerp(directionSmoothDebug, velocityDirectionInfo.GetDirection(300), Time.deltaTime * 8);
            inverseDirectionSmoothDebug = Vector3.Lerp(inverseDirectionSmoothDebug, velocityDirectionInfo.GetDirection(300, true), Time.deltaTime * 8);

            Debug.DrawLine(transform.position, directionSmoothDebug, Color.yellow) ;
            Debug.DrawLine(transform.position, inverseDirectionSmoothDebug, Color.red);

            lastPosition = transform.position;
        }
    }
}