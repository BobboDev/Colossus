using System;
using System.Collections;
using System.Collections.Generic;
using KinematicCharacterController;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.UIElements;

public class Platformer : MonoBehaviour
{
    public Rigidbody rb;
    public Transform cam;
    public float moveSpeed = 1;
    Vector3 mostRecentInput;
    Vector3 mostRecentMove;
    Vector3 moveForceLastInput;
    public LayerMask layerMask;
    Vector3 hitSpherecastOrigin;
    Vector3 groundHitSpherecastOrigin;
    Vector3 wallCheckHitSphereCastOrigin;
    Vector3 ceilingCheckHitSphereCastOrigin;
    Vector3 cornerCheckSphereCastOrigin;
    bool grounded;
    Vector3 cornerGroundCheckStartPosition;
    int cornerFoundCount = 0;
    public float maxSlope = 0.3f;
    bool test = true;
    public CapsuleCollider capsuleCollider;
    Vector3 cornerOrigin;
    // Update is called once per frame
    Vector3 testCapsulePosition;

    void FixedUpdate()
    {
        Vector3 input = CalculateInput();
        Vector3 cameraPlanarDirection = Vector3.ProjectOnPlane(cam.rotation * Vector3.up, Vector3.up).normalized;
        // input = Vector3.right;
        mostRecentInput = input.magnitude > 0 ? Vector3.ProjectOnPlane(cam.rotation * input, Vector3.up).normalized : mostRecentInput;
        mostRecentMove = input.magnitude > 0 ? input : mostRecentMove;

        Vector3 moveForce = Quaternion.LookRotation(cameraPlanarDirection, Vector3.up) * input;
        // moveForce = Vector3.left;

        moveForceLastInput = input.magnitude > 0 ? Quaternion.LookRotation(cameraPlanarDirection, Vector3.up) * mostRecentMove : moveForceLastInput;

        rb.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(mostRecentInput, Vector3.up).normalized, Vector3.up);

        RaycastHit hit;
        RaycastHit groundHit;
        // RaycastHit secondHit;
        RaycastHit wallCheckHit;
        RaycastHit stepCheckHit;
        RaycastHit realHit;
        RaycastHit realWallCheckHit;
        RaycastHit cornerCheckHit;
        RaycastHit ceilingCheckHit;

        if (!grounded)
        {
            if (Physics.SphereCast(rb.position, 0.05f, -Vector3.up, out hit, 0.1f, layerMask))
            {
                grounded = true;
            }
            rb.velocity = moveForce * moveSpeed;
        }
        if (grounded)
        {
            rb.velocity = new Vector3(rb.velocity.x, 0, rb.velocity.z);
            if (Physics.SphereCast(rb.position + Vector3.up * 0.05f, 0.05f, -Vector3.up, out hit, Mathf.Infinity, layerMask))
            {
                Vector3 wallParallelVector = Vector3.Cross(hit.normal, Vector3.up).normalized;
                Vector3 downHillVector = -Vector3.Cross(wallParallelVector, hit.normal).normalized;
                downHillVector = downHillVector.y > 0 ? -downHillVector : downHillVector;
                hitSpherecastOrigin = SphereOrCapsuleCastCenterOnCollision(rb.position + Vector3.up * 0.05f, -Vector3.up, hit.distance);
                testCapsulePosition = rb.position;

                // if (CapsuleC)
                if (Vector3.Dot(hit.normal, Vector3.up) < 0.3f)
                {
                    // NOTE: When we spherecast back down the wall to get the ground position for sliding, if it is going down an edge the sphercast collides with it.
                    if (Physics.SphereCast(hitSpherecastOrigin, 0.0499f, downHillVector, out groundHit, Mathf.Infinity, layerMask))
                    {
                        groundHitSpherecastOrigin = SphereOrCapsuleCastCenterOnCollision(hitSpherecastOrigin, downHillVector, groundHit.distance);
                        Debug.Log("Groundhit 1");
                    }
                    Debug.DrawLine(rb.position, rb.position + downHillVector, Color.yellow);
                    Debug.Log("A");


                    if (Vector3.Dot(groundHit.normal, Vector3.up) < 0.3f && Vector3.Dot(hit.normal, Vector3.up) < 0.3f)
                    {
                        if (Vector3.Dot(groundHit.normal, hit.normal) < 0.9f)
                        {
                            Vector3 cornerSlope = Vector3.Cross(hit.normal, groundHit.normal);

                            if (cornerSlope.y > 0)
                            {
                                cornerSlope = -cornerSlope;
                            }

                            if (Physics.SphereCast(hitSpherecastOrigin, 0.0499f, cornerSlope.normalized, out groundHit, Mathf.Infinity, layerMask))
                            {
                                groundHitSpherecastOrigin = SphereOrCapsuleCastCenterOnCollision(hitSpherecastOrigin, cornerSlope.normalized, groundHit.distance);
                                Debug.Log("Groundhit 2");

                            }

                            Debug.Log("STUCK 0");
                            // Debug.Log("groundhit.normal " + groundHit.normal);
                            // Debug.Log("wallCheckHit.normal " + hit.normal);

                            // Debug.DrawLine(cornerCheckHit.point, cornerCheckHit.point + cornerCheckHit.normal);

                        }
                    }

                }
                else
                {
                    Debug.Log("B");
                    groundHit = hit;
                    groundHitSpherecastOrigin = hitSpherecastOrigin;
                    Debug.Log("Groundhit 3");
                }



                // if (test && hit.collider.name != "Floor")
                // if (Vector3.Dot(groundHit.normal,Vector3.up) >= 0.3f)

                // IF IT'S HITSPHERECASTORIGIN THEN IT BUMPS AGAINST WALLS
                // IF IT'S GROUNDHIT THEN IT DOESN'T WORK ON STEPS

                rb.position = groundHitSpherecastOrigin + 0.05f * Vector3.up;
                rb.velocity = Vector3.Cross(Quaternion.Euler(0, 90, 0) * moveForce.normalized, groundHit.normal).normalized * moveSpeed;

                // Physics.SphereCast(groundHitSpherecastOrigin + 0.05f * Vector3.up, 0.05f,-Vector3.up, out secondHit, Mathf.Infinity, layerMask);

                // hitSpherecastOrigin = SphereOrCapsuleCastCenterOnCollision(groundHitSpherecastOrigin + 0.05f * Vector3.up,-Vector3.up,secondHit.distance);

                Vector3 slopeVector = Vector3.Dot(hit.normal, Vector3.up) < 0.3 ? -Vector3.Cross(Quaternion.Euler(0, 90, 0) * moveForce.normalized, groundHit.normal).normalized * moveSpeed : -Vector3.Cross(Quaternion.Euler(0, 90, 0) * moveForce.normalized, hit.normal).normalized * moveSpeed;

                Physics.CapsuleCast(groundHitSpherecastOrigin + Vector3.up * 0.1f, groundHitSpherecastOrigin, 0.0499f, -slopeVector.normalized, out wallCheckHit, rb.velocity.magnitude * Time.fixedDeltaTime + 0.0001f, layerMask);
                wallCheckHitSphereCastOrigin = SphereOrCapsuleCastCenterOnCollision(groundHitSpherecastOrigin + Vector3.up * 0.0001f, -slopeVector.normalized, wallCheckHit.distance);


                Physics.SphereCast(wallCheckHit.point + Vector3.up * 0.1f, 0.01f, -Vector3.up, out stepCheckHit, Mathf.Infinity, layerMask);
                bool stepFound = false;
                if (Vector3.Dot(stepCheckHit.normal, Vector3.up) > 0.3f && stepCheckHit.point.y <= groundHitSpherecastOrigin.y + 0.05f && stepCheckHit.point.y > groundHit.point.y)
                {
                    stepFound = true;
                    rb.position = hitSpherecastOrigin + 0.0501f * Vector3.up;
                }
                Debug.Log("Stepcheckhit.point.y " + stepCheckHit.point.y);
                Debug.Log("rb.position.y " + rb.position.y);
                Debug.Log("Vector3.Dot(stepCheckHit.normal,Vector3.up) " + Vector3.Dot(stepCheckHit.normal, Vector3.up));
                Debug.Log("stepFound " + stepFound);
                Physics.Raycast(rb.position - Vector3.up * 0.05f, (hit.point - (rb.position - Vector3.up * 0.05f)).normalized, out realHit, Mathf.Infinity, layerMask);
                Physics.Raycast(rb.position - Vector3.up * 0.05f, (wallCheckHit.point - (rb.position - Vector3.up * 0.05f)).normalized, out realWallCheckHit, Mathf.Infinity, layerMask);
                // Debug.DrawLine(groundHit.point, groundHit.point + groundHit.normal, Color.black);

                // if (test && groundHit.collider.name != "Island Floor")
                // {
                //     EditorApplication.isPaused = true;

                //      Debug.Log(groundHit.collider.name + " pause");
                // }
                bool cornerFound = false;

                // If the first hit detects a wall
                if (!stepFound)
                {
                    if (Vector3.Dot(hit.normal, Vector3.up) < 0.3f)
                    {
                        Vector3 wallSlideDirectionAndSpeed = Mathf2.GetClosestPointOnInfiniteLine(Vector3.zero, Vector3.Cross(hit.normal, groundHit.normal).normalized, moveForce * moveSpeed);
                        cornerOrigin = Vector3.zero;

                        bool groundCheckWasStuckInCorner = false;

                        Vector3 cornerSlope = Vector3.Cross(wallCheckHit.normal, groundHit.normal);

                        if (Vector3.Dot(groundHit.normal, Vector3.up) < 0.3f && Vector3.Dot(wallCheckHit.normal, Vector3.up) < 0.3f)
                        {
                            if (Vector3.Dot(groundHit.normal, wallCheckHit.normal) < 0.9f)
                            {
                                if (cornerSlope.y > 0)
                                {
                                    cornerSlope = -cornerSlope;
                                }

                                if (Physics.SphereCast(hitSpherecastOrigin, 0.0499f, cornerSlope.normalized, out groundHit, Mathf.Infinity, layerMask))
                                {
                                    groundHitSpherecastOrigin = SphereOrCapsuleCastCenterOnCollision(hitSpherecastOrigin, cornerSlope.normalized, groundHit.distance);
                                }
                                if (Vector3.Dot(stepCheckHit.normal, Vector3.up) > 0.3f && stepCheckHit.point.y <= groundHitSpherecastOrigin.y + 0.05f && stepCheckHit.point.y > groundHit.point.y)
                                {
                                    stepFound = true;
                                    return;
                                }
                                slopeVector = Vector3.Dot(hit.normal, Vector3.up) < 0.3 ? -Vector3.Cross(Quaternion.Euler(0, 90, 0) * moveForce.normalized, groundHit.normal).normalized * moveSpeed : -Vector3.Cross(Quaternion.Euler(0, 90, 0) * moveForce.normalized, hit.normal).normalized * moveSpeed;
                                Physics.CapsuleCast(groundHitSpherecastOrigin + Vector3.up * 0.1f, groundHitSpherecastOrigin, 0.0499f, -slopeVector.normalized, out wallCheckHit, rb.velocity.magnitude * Time.fixedDeltaTime + 0.0001f, layerMask);
                                wallCheckHitSphereCastOrigin = SphereOrCapsuleCastCenterOnCollision(groundHitSpherecastOrigin + Vector3.up * 0.0001f, -slopeVector.normalized, wallCheckHit.distance);
                                groundCheckWasStuckInCorner = true;
                                wallSlideDirectionAndSpeed = Mathf2.GetClosestPointOnInfiniteLine(Vector3.zero, Vector3.Cross(hit.normal, groundHit.normal).normalized, moveForce * moveSpeed);
                                Debug.Log("STUCK 1");
                                // Debug.Log("groundhit.normal " + groundHit.normal);
                                // Debug.Log("wallCheckHit.normal " + wallCheckHit.normal);

                                // Debug.DrawLine(cornerCheckHit.point, cornerCheckHit.point + cornerCheckHit.normal);

                            }
                        }

                        if (Physics.SphereCast(groundHitSpherecastOrigin, 0.0499f, wallSlideDirectionAndSpeed.normalized, out cornerCheckHit, wallSlideDirectionAndSpeed.magnitude * Time.fixedDeltaTime + 0.001f, layerMask) && cornerCheckHit.normal != wallCheckHit.normal)
                        {
                            cornerFound = true;
                            cornerFoundCount++;
                            Plane wall1 = new Plane(wallCheckHit.normal, wallCheckHit.point + wallCheckHit.normal * 0.0501f);
                            Plane wall2 = new Plane(groundHit.normal, groundHit.point + groundHit.normal * 0.0501f);
                            Plane wall3 = new Plane(cornerCheckHit.normal, cornerCheckHit.point + cornerCheckHit.normal * 0.0501f);
                            // if (test && groundHit.collider.name != "Island Floor")
                            // {
                            //     EditorApplication.isPaused = true;
                            // }
                            cornerOrigin = Mathf2.PlanesIntersection(wall1, wall2, wall3);
                            Debug.DrawLine(cornerOrigin, cornerOrigin + Vector3.up, Color.black);
                            Debug.DrawLine(wallCheckHit.point, wallCheckHit.point + wallCheckHit.normal, Color.green);
                            Debug.DrawLine(groundHit.point, groundHit.point + groundHit.normal, Color.red);
                            Debug.DrawLine(cornerCheckHit.point, cornerCheckHit.point + cornerCheckHit.normal, Color.blue);
                            Debug.Log("1");
                        }
                        else
                        {
                            // Debug.Log("cornerCheckHit.normal " + cornerCheckHit.normal);
                            // Debug.Log("wallCheckHit.normal " + wallCheckHit.normal);
                            Debug.DrawLine(groundHit.point, groundHit.point + groundHit.normal);
                            Debug.DrawLine(wallCheckHit.point, wallCheckHit.point + wallCheckHit.normal);
                            Debug.DrawLine(rb.position, rb.position + wallSlideDirectionAndSpeed.normalized, Color.cyan);
                            // EditorApplication.isPaused = true;
                            Debug.Log("1B");
                            Debug.DrawLine(cornerOrigin, cornerOrigin + Vector3.up, Color.black);
                            Debug.DrawLine(wallCheckHit.point, wallCheckHit.point + wallCheckHit.normal, Color.green);
                            Debug.DrawLine(groundHit.point, groundHit.point + groundHit.normal, Color.red);
                            Debug.DrawLine(cornerCheckHit.point, cornerCheckHit.point + cornerCheckHit.normal, Color.blue);
                        }
                        cornerCheckSphereCastOrigin = SphereOrCapsuleCastCenterOnCollision(groundHitSpherecastOrigin, wallSlideDirectionAndSpeed.normalized, cornerCheckHit.distance);

                        // Then slide along the wall
                        // if (Vector3.Dot(Vector3.Cross(wallSlideDirectionAndSpeed.normalized,hit.normal).normalized,Vector3.ProjectOnPlane(hit.normal,Vector3.up).normalized) < 0)

                        if (cornerFound && Vector3.Dot(cornerCheckHit.normal, Vector3.up) < 0.3f || groundCheckWasStuckInCorner)
                        {
                            rb.velocity = Vector3.zero; // This works for corners where you're meant to stop but not corners which you should slide round 


                            // if (test && groundHit.collider.name != "Floor")
                            rb.position = cornerOrigin + Vector3.up * 0.05f;
                            Plane wallHitPlane = new Plane(cornerCheckHit.normal, cornerCheckHit.point);
                            float cornerPointDistance;
                            Ray wall1Ray = new Ray(wallCheckHit.point, wallSlideDirectionAndSpeed.normalized);
                            wallHitPlane.Raycast(wall1Ray, out cornerPointDistance);
                            Vector3 cornerPoint = wall1Ray.origin + wallSlideDirectionAndSpeed.normalized * cornerPointDistance;
                            // Debug.DrawLine(cornerPoint,cornerPoint + Vector3.up, Color.red);
                            cornerGroundCheckStartPosition = cornerPoint + (wallCheckHit.normal + cornerCheckHit.normal).normalized * (1 - Mathf.Abs(Vector3.Dot(wallCheckHit.normal, cornerCheckHit.normal)));
                            Debug.Log("2");
                            Debug.Log("corner found and corner surface facing up" + (cornerFound && Vector3.Dot(cornerCheckHit.normal, Vector3.up) < 0.3f));
                            Debug.Log("groundcheck was stuck in corner" + groundCheckWasStuckInCorner);

                            // The cross prod
                            // YO!!!
                            // We have to get the slope of the corner here and do another scan to find the ground hit
                            // starting position is the first hit
                            // slope is the two corners vs the up vector
                        }
                        else
                        {
                            // if (test && groundHit.collider.name != "Floor")
                            // IF IT'S HITSPHERECASTORIGIN THEN IT BUMPS AGAINST WALLS
                            // IF IT'S GROUNDHIT THEN IT DOESN'T WORK ON STEPS

                            rb.position = wallCheckHitSphereCastOrigin + 0.05f * Vector3.up;

                            rb.velocity = wallSlideDirectionAndSpeed;
                            // Debug.DrawLine(wallCheckHit.point, wallCheckHit.point + wallCheckHit.normal, Color.green);
                            // Debug.DrawLine(groundHit.point, groundHit.point + groundHit.normal, Color.blue);

                            Debug.DrawLine(wallCheckHit.point, wallCheckHit.point + wallCheckHit.normal, Color.green);
                            Debug.DrawLine(groundHit.point, groundHit.point + groundHit.normal * 0.66f, Color.red);
                            Debug.DrawLine(cornerCheckHit.point, cornerCheckHit.point + cornerCheckHit.normal * 0.33f, Color.blue);
                            Debug.DrawLine(wallCheckHit.point, wallCheckHit.point + slopeVector, Color.yellow);
                            // Debug.Log(Time.frameCount +  " 3");
                            Debug.Log("2B" + stepFound);
                        }
                    }
                    else if (Vector3.Dot(wallCheckHit.normal, Vector3.up) < 0.3f
                    && (Vector3.Dot(stepCheckHit.normal, Vector3.up) < 0.3f))
                    {
                        // If the first hit doesn't detect a wall (Because it's been placed intersecting one)
                        // And the wall is not a step
                        Debug.Log("#10");

                        // NOTE IT'S STEP CHECK THAT IS FALSE
                        // step check is false because when colliding with a wall, step check uses "hit", to check if it's a step. However 'hit' is no longer hitting a wall, but ground
                        Vector3 wallSlideDirectionAndSpeed = Mathf2.GetClosestPointOnInfiniteLine(Vector3.zero, Vector3.Cross(wallCheckHit.normal, hit.normal).normalized, moveForce * moveSpeed);
                        Vector3 cornerOrigin = Vector3.zero;

                        bool groundCheckWasStuckInCorner = false;

                        Vector3 cornerSlope = Vector3.Cross(wallCheckHit.normal, groundHit.normal);
                        if (Vector3.Dot(groundHit.normal, Vector3.up) < 0.3f && Vector3.Dot(wallCheckHit.normal, Vector3.up) < 0.3f)
                        {
                            if (Vector3.Dot(groundHit.normal, wallCheckHit.normal) < 0.9f)
                            {
                                if (cornerSlope.y > 0)
                                {
                                    cornerSlope = -cornerSlope;
                                }

                                if (Physics.SphereCast(hitSpherecastOrigin, 0.0499f, cornerSlope.normalized, out groundHit, Mathf.Infinity, layerMask))
                                {
                                    groundHitSpherecastOrigin = SphereOrCapsuleCastCenterOnCollision(hitSpherecastOrigin, cornerSlope.normalized, groundHit.distance);
                                }
                                if (Vector3.Dot(stepCheckHit.normal, Vector3.up) > 0.3f && stepCheckHit.point.y <= groundHitSpherecastOrigin.y + 0.05f && stepCheckHit.point.y > groundHit.point.y)
                                {
                                    stepFound = true;
                                    return;
                                }
                                slopeVector = Vector3.Dot(hit.normal, Vector3.up) < 0.3 ? -Vector3.Cross(Quaternion.Euler(0, 90, 0) * moveForce.normalized, groundHit.normal).normalized * moveSpeed : -Vector3.Cross(Quaternion.Euler(0, 90, 0) * moveForce.normalized, hit.normal).normalized * moveSpeed;
                                Physics.CapsuleCast(groundHitSpherecastOrigin + Vector3.up * 0.1f, groundHitSpherecastOrigin, 0.0499f, -slopeVector.normalized, out wallCheckHit, rb.velocity.magnitude * Time.fixedDeltaTime + 0.0001f, layerMask);
                                wallCheckHitSphereCastOrigin = SphereOrCapsuleCastCenterOnCollision(groundHitSpherecastOrigin + Vector3.up * 0.0001f, -slopeVector.normalized, wallCheckHit.distance);
                                groundCheckWasStuckInCorner = true;
                                wallSlideDirectionAndSpeed = Mathf2.GetClosestPointOnInfiniteLine(Vector3.zero, Vector3.Cross(hit.normal, groundHit.normal).normalized, moveForce * moveSpeed);
                                Debug.Log("STUCK 2");
                            }
                        }

                        if (Physics.SphereCast(groundHitSpherecastOrigin, 0.0499f, wallSlideDirectionAndSpeed.normalized, out cornerCheckHit, wallSlideDirectionAndSpeed.magnitude * Time.fixedDeltaTime + 0.001f, layerMask))
                        {
                            cornerFound = true;
                            cornerFoundCount++;
                            Plane wall1 = new Plane(wallCheckHit.normal, wallCheckHit.point + wallCheckHit.normal * 0.0501f);
                            Plane wall2 = new Plane(groundHit.normal, groundHit.point + groundHit.normal * 0.0501f);
                            Plane wall3 = new Plane(cornerCheckHit.normal, cornerCheckHit.point + cornerCheckHit.normal * 0.0501f);
                            cornerOrigin = Mathf2.PlanesIntersection(wall1, wall2, wall3);
                            // Debug.DrawLine(cornerOrigin, cornerOrigin + Vector3.up, Color.black);
                            // Debug.DrawLine(wallCheckHit.point, wallCheckHit.point + wallCheckHit.normal, Color.green);  
                            // Debug.DrawLine(groundHit.point, groundHit.point + groundHit.normal, Color.red);  
                            // Debug.DrawLine(cornerCheckHit.point, cornerCheckHit.point + cornerCheckHit.normal, Color.blue);  
                            // if (test && groundHit.collider.name != "Island Floor")
                            // {
                            //     EditorApplication.isPaused = true;
                            //     Debug.Log("4" + " pause");
                            // }

                        }
                        cornerCheckSphereCastOrigin = SphereOrCapsuleCastCenterOnCollision(groundHitSpherecastOrigin, wallSlideDirectionAndSpeed.normalized, cornerCheckHit.distance);

                        // If we are not holding away from the wall
                        // Then slide along the wall
                        // if (Vector3.Dot(Vector3.Cross(wallSlideDirectionAndSpeed.normalized,wallCheckHit.normal).normalized,Vector3.ProjectOnPlane(wallCheckHit.normal,Vector3.up).normalized) < 0)
                        if (cornerFound && Vector3.Dot(cornerCheckHit.normal, Vector3.up) < 0.3f || groundCheckWasStuckInCorner)
                        {
                            rb.velocity = Vector3.zero; // This works for corners where you're meant to stop but not corners which you should slide round 

                            // YO!!!
                            // We have to get the slope of the corner here and do another scan to find the ground hit
                            // starting position is the first hit
                            // slope is the two corners vs the up vector

                            // if (test && groundHit.collider.name != "Floor")
                            rb.position = cornerOrigin + Vector3.up * 0.05f;
                            Plane wallHitPlane = new Plane(cornerCheckHit.normal, cornerCheckHit.point);
                            float cornerPointDistance;
                            Ray wall1Ray = new Ray(wallCheckHit.point, wallSlideDirectionAndSpeed.normalized);
                            wallHitPlane.Raycast(wall1Ray, out cornerPointDistance);
                            Vector3 cornerPoint = wall1Ray.origin + wallSlideDirectionAndSpeed.normalized * cornerPointDistance;
                            // Debug.DrawLine(cornerPoint,cornerPoint + Vector3.up, Color.red);

                            cornerGroundCheckStartPosition = cornerPoint + (wallCheckHit.normal + cornerCheckHit.normal).normalized * (1 - Mathf.Abs(Vector3.Dot(wallCheckHit.normal, cornerCheckHit.normal)));
                            Debug.Log("5");
                        }
                        else
                        {
                            Debug.Log("6");
                            // if (test && groundHit.collider.name != "Floor")
                            rb.position = wallCheckHitSphereCastOrigin + 0.05f * Vector3.up;
                            rb.velocity = wallSlideDirectionAndSpeed;
                        }
                    }
                    else
                    {
                        Debug.Log("7");
                        Debug.DrawLine(groundHit.point, groundHit.point + groundHit.normal);
                        Debug.DrawLine(wallCheckHit.point, wallCheckHit.point + wallCheckHit.normal);
                        // Debug.DrawLine(rb.position, rb.position + wallSlideDirectionAndSpeed.normalized, Color.cyan);
                    }
                }
            }

        }
        // Debug.Log("cornerFoundCount " + cornerFoundCount);
    }

    public static Vector3 SphereOrCapsuleCastCenterOnCollision(Vector3 origin, Vector3 directionCast, float hitInfoDistance)
    {
        return origin + (directionCast.normalized * hitInfoDistance);
    }

    void OnDrawGizmos()
    {
        // Gizmos.DrawWireSphere(testCapsulePosition + Vector3.up * 0.05f, 0.05f);
        // Gizmos.DrawWireSphere(testCapsulePosition - Vector3.up * 0.05f, 0.05f);
        Gizmos.DrawWireSphere(groundHitSpherecastOrigin, 0.05f);
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(wallCheckHitSphereCastOrigin, 0.05f);
    }

    Vector3 CalculateInput()
    {
        Vector3 input = new Vector3(Input.GetAxisRaw("Horizontal"), 0, Input.GetAxisRaw("Vertical")).normalized;
        return input.normalized * input.magnitude;
    }
}
