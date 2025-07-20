// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class InsectDemo : MonoBehaviour
    {
        public bool enableInsect = true;
        public SurfaceGrab surfaceGrab;
        public Transform targetTentacle;

        public FCSTypes.Axis onWalkingMoveAxis = FCSTypes.Axis.FORWARD, onClimbingMoveAxis = FCSTypes.Axis.UP;

        public bool lockLookX, lockLookY = true, lockLookZ;

        public float lookRotationSpeed = 8f;
        public float moveSpeed = 4f;

        private void Start()
        {
            if (surfaceGrab == null)
                surfaceGrab = GetComponent<SurfaceGrab>();
        }

        void Update()
        {
            if (!enableInsect) return;

            if(surfaceGrab == null)
            {
                Debug.LogWarning("SurfaceGrab not detected. Please configure climbing on this insect.");
                return;
            }

            if(targetTentacle == null)
            {
                Debug.LogWarning("Target tentacle not defined. Please assign in inspector.");
                return; 
            }

            if (!surfaceGrab.IsOnClimbing)
            {
                surfaceGrab.GrabSurface();

                transform.position = transform.position + FCSTypes.AxisEnumToLocalVector3(transform, onWalkingMoveAxis) * Time.deltaTime * moveSpeed;

                var lookPos = targetTentacle.position - transform.position;
                if (lockLookX)
                    lookPos.x = 0;

                if(lockLookY)
                    lookPos.y = 0;

                if (lockLookZ)
                    lookPos.z = 0;

                var rotation = Quaternion.LookRotation(lookPos);
                transform.rotation = Quaternion.Slerp(transform.rotation, rotation, Time.deltaTime * lookRotationSpeed);
            }
            else
            {
                transform.position = transform.position + FCSTypes.AxisEnumToLocalVector3(transform, onClimbingMoveAxis) * Time.deltaTime * moveSpeed;
            }
        }
    }
}