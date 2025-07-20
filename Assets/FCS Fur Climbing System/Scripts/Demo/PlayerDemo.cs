// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections;
using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class PlayerDemo : MonoBehaviour
    {
        public CameraDemo cameraDemo;
        public SurfaceGrab surfaceGrab;
        public Transform rightHand;
        public Animator animator;
        public bool useVelocity = true, playerFacesToCamera = true, fakeGravity = true, riseWhenDetectGroundOnClimb = true;
        public float fakeGravityY = 15f;
        public Sensor sRight, sLeft, sUp, sDown;
        public float speed = 4;
        public bool lockClimb = false;

        private Vector3 defaultPlayerPos, defaultPlayerRot;

        GameObject pendulumAux;

        [System.Serializable]
        public class LockKeys
        {
            public bool up, down, left, right;
        }

        public LockKeys lockKeys = new LockKeys();

        void Start()
        {
            defaultPlayerPos = transform.position;
            defaultPlayerRot = transform.eulerAngles;
            pendulumAux = new GameObject();
        }

        public void StopClimb()
        {
            if (!onClimbingGround)
                animator.Play("IDLE");
            animator.SetBool("Up", false);
            animator.SetBool("Left", false);
            animator.SetBool("Right", false);
            animator.SetBool("Down", false);
        }

        public void ResetPlayer()
        {
            if (onClimbingGround) return;
            //transform.position = defaultPlayerPos;
            transform.eulerAngles = defaultPlayerRot;
            //StartCoroutine(ResetPlayerTimer());
        }

        IEnumerator ResetPlayerTimer()
        {
            yield return new WaitForSeconds(2f);
            transform.position = defaultPlayerPos;
            transform.eulerAngles = defaultPlayerRot;
            StopClimb();
        }

        private void FakeGravity()
        {
            if (GetComponent<SurfaceWalker>() != null)
            {
                if (GetComponent<SurfaceWalker>().IsWalkingInMesh)
                {
                    return;
                }
            }
            Vector3 pos = transform.position;
            pos.y = Mathf.MoveTowards(pos.y, fakeGravityY, Time.deltaTime * 3f);
            transform.position = pos;
        }

        public bool onClimbingGround = false;
        public void ClimbingGround()
        {
            if (!riseWhenDetectGroundOnClimb) return;
            float angle = Vector3.Angle(transform.forward, Vector3.up);
            if(angle > 60 && angle < 95 && !onClimbingGround && surfaceGrab.IsOnClimbing)
            {
                Raycaster.TriangleHit triangleHit = surfaceGrab.CastAndGetTriangleHit();
                transform.rotation = Quaternion.FromToRotation(-transform.forward, triangleHit.normal) * transform.rotation;
                transform.position = triangleHit.point;
                transform.position = transform.position + -transform.forward * 0.45f;
                onClimbingGround = true;
                surfaceGrab.Drop();
                GetComponent<SurfaceWalker>().enabled = false;
                animator.SetBool("Ground", true);
                animator.Play("ClimbingGround");
                StopAllCoroutines();
                StartCoroutine(DisableClimbGround());
            }
        }

        IEnumerator DisableClimbGround()
        {
            yield return new WaitForSeconds(3.833f);
            animator.SetBool("Ground", false);
            GetComponent<SurfaceWalker>().enabled = true;
            onClimbingGround = false;
        }

        public void Attack()
        {
            GameObject res = Resources.Load<GameObject>("FCSBlood");
            GameObject blood = Instantiate(res);
            blood.transform.position = rightHand.position;
            blood.transform.rotation = Quaternion.FromToRotation(blood.transform.forward,
                surfaceGrab.CurrentTriangle.ComputeNormalizePosition()) * blood.transform.rotation;
            surfaceGrab.CreateClimbingPointForCurrentTriangle().AttachObject(blood);
        }

        // Update is called once per frame
        void Update()
        {
            /*if (surfaceGrab.IsOnClimbing)
            {
                surfaceGrab.CurrentClimbingPoint.AttachObject(pendulumAux.transform);

                //transform.rotation = Quaternion.LookRotation(surfaceGrab.CurrentObjectVelocity.velocityDirectionInfo.GetDirection());
                //Debug.Log("ABC");
            }*/

            if (onClimbingGround) return;

            if (Input.GetKeyDown(KeyCode.E) && surfaceGrab.IsOnClimbing && rightHand != null) /*Blood Test*/
            {
                Attack();
            }

            if (Input.GetKey(KeyCode.Mouse1) || lockClimb)
            {
                if(surfaceGrab != null)
                    surfaceGrab.GrabSurface();
            }
            else
            {
                if(surfaceGrab != null)
                    surfaceGrab.Drop();

                if (playerFacesToCamera)
                {
                    if (((Input.GetKey(KeyCode.W) || lockKeys.down) || (Input.GetKey(KeyCode.S) || lockKeys.up)) /*&& (Input.GetAxis("Mouse Y") != 0 || Input.GetAxis("Mouse X") != 0)*/)
                        transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.Euler(0, cameraDemo.mainCamera.transform.eulerAngles.y, 0), Time.deltaTime * 12f);
                }

            }

            bool canClimb;

            if(surfaceGrab != null)
            {
                canClimb = surfaceGrab.IsOnClimbing;
            }
            else
            {
                canClimb = false;
            }

            if (canClimb)
            {
                animator.SetBool("Running", false);

                if ((Input.GetKey(KeyCode.W) || lockKeys.up) && sUp.IsDetecting)
                {
                    animator.SetBool("Up", true);
                }
                else
                {
                    animator.SetBool("Up", false);
                }

                if ((Input.GetKey(KeyCode.A) || lockKeys.left) && sLeft.IsDetecting)
                {
                    animator.SetBool("Left", true);
                }
                else
                {
                    animator.SetBool("Left", false);
                }

                if ((Input.GetKey(KeyCode.D) || lockKeys.right) && sRight.IsDetecting)
                {
                    animator.SetBool("Right", true);
                }
                else
                {
                    animator.SetBool("Right", false);
                }

                if ((Input.GetKey(KeyCode.S) || lockKeys.down) && sDown.IsDetecting)
                {
                    animator.SetBool("Down", true);
                }
                else
                {
                    animator.SetBool("Down", false);
                }
            }
            else
            {
                animator.SetBool("Running", true);

                float m = 1;

                if (Input.GetKey(KeyCode.W) || lockKeys.up)
                {
                    animator.SetFloat("Run Speed", 1);
                    m = 1;
                }
                
                if(Input.GetKey(KeyCode.S) || lockKeys.down)
                {
                    animator.SetFloat("Run Speed", -1);
                    m = -1;
                }

                if (Input.GetKey(KeyCode.A) || lockKeys.left)
                {
                    transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.Euler(0, cameraDemo.mainCamera.transform.eulerAngles.y - 90 * m, 0), Time.deltaTime * 12f);
                    animator.SetFloat("Run Speed", 1 * m);
                }

                if (Input.GetKey(KeyCode.D) || lockKeys.right)
                {
                    transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.Euler(0, cameraDemo.mainCamera.transform.eulerAngles.y + 90 * m, 0), Time.deltaTime * 12f);
                    animator.SetFloat("Run Speed", 1 * m);
                }

                if (!(Input.GetKey(KeyCode.W) || lockKeys.up) && !(Input.GetKey(KeyCode.S) || lockKeys.down) && !(Input.GetKey(KeyCode.A) || lockKeys.left) && !(Input.GetKey(KeyCode.D) || lockKeys.right ))
                {
                    animator.SetFloat("Run Speed", 0);
                }

                if(fakeGravity) 
                    FakeGravity();
            }
        }
    }
}