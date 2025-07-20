// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class CameraDemo : MonoBehaviour
    {
        public Transform target;
        public Camera mainCamera;
        public PlayerDemo playerDemo;
        [Range(0.1f, 5f)]
        public float mouseRotateSpeed = 0.8f;
        [Range(0.01f, 100)]
        public float slerpValue = 0.25f;
        private Quaternion cameraRot;
        private float distanceBetweenCameraAndTarget, minXRotAngle = -80, maxXRotAngle = 80, rotX, rotY;

        public bool stop = false;

        private GameObject cameraAux;

        private void Awake()
        {
            if (mainCamera == null)
            {
                mainCamera = Camera.main;
            }
        }

        void Start()
        {
            distanceBetweenCameraAndTarget = Vector3.Distance(mainCamera.transform.position, target.position);
            defaultCameraDistance = distanceBetweenCameraAndTarget;

            cameraAux = new GameObject();
            cameraAux.name = "CameraDemoAux";

            cameraAux.transform.position = target.transform.position;
        }

        public void EnableCameraRotation(bool b)
        {
            stop = !b;
        }

        float startZoomingDelay = 0f;
        float zoomClimbing = 0;
        float defaultCameraDistance;

        void Update()
        {
            if (!stop)
            {
                rotX += -Input.GetAxis("Mouse Y") * mouseRotateSpeed;
                rotY += Input.GetAxis("Mouse X") * mouseRotateSpeed;
                if (rotX < minXRotAngle)
                {
                    rotX = minXRotAngle;
                }
                else if (rotX > maxXRotAngle)
                {
                    rotX = maxXRotAngle;
                }
            }

            // Zooming while climbing.
            if (playerDemo.surfaceGrab != null)
            {
                if (playerDemo.surfaceGrab.IsOnClimbing)
                {
                    startZoomingDelay += Time.deltaTime;

                    if (startZoomingDelay >= 8f && zoomClimbing <= 5f)
                    {
                        zoomClimbing += Time.deltaTime;
                    }
                }
                else
                {
                    startZoomingDelay = 0;
                    zoomClimbing = Mathf.Lerp(zoomClimbing, 0, Time.deltaTime);
                }


                distanceBetweenCameraAndTarget = Mathf.Lerp(distanceBetweenCameraAndTarget, defaultCameraDistance + zoomClimbing, Time.deltaTime * 8f);
            }
        }

        private void LateUpdate()
        {
            float speed = 8f;

            if (playerDemo.surfaceGrab != null)
            {
                if (playerDemo.surfaceGrab.CurrentClimbingPoint != null)
                    speed = Mathf.Clamp(playerDemo.surfaceGrab.CurrentObjectVelocity.velocityInfo.Magnitude, 8f, 12f);
            }

            Vector3 dir = new Vector3(0, 0, -distanceBetweenCameraAndTarget);
            cameraRot = Quaternion.Slerp(cameraRot, Quaternion.Euler(rotX, rotY, 0), slerpValue);

            if (playerDemo.surfaceGrab != null && playerDemo.surfaceGrab.IsOnClimbing)
            {
                mainCamera.transform.position = Vector3.Lerp(mainCamera.transform.position, target.position + cameraRot * dir, Time.deltaTime * speed);
            }
            else
            {
                mainCamera.transform.position = target.position + cameraRot * dir;
            }

            cameraAux.transform.position = Vector3.Lerp(cameraAux.transform.position, target.position, Time.deltaTime * speed);

            if (playerDemo.surfaceGrab != null && playerDemo.surfaceGrab.IsOnClimbing)
            {
                Vector3 relativePos = cameraAux.transform.position - mainCamera.transform.transform.position;
                Quaternion rotation = Quaternion.LookRotation(relativePos, Vector3.up);
                mainCamera.transform.rotation = Quaternion.Slerp(mainCamera.transform.rotation, rotation, Time.deltaTime * 18f);
            }
            else
            {
                mainCamera.transform.LookAt(target.position);
            }

            //Debug.Log(speed);
        }

        public void SetCamPos()
        {
            if (mainCamera == null)
            {
                mainCamera = Camera.main;
            }
            mainCamera.transform.position = new Vector3(0, 0, -distanceBetweenCameraAndTarget);
        }

    }
}