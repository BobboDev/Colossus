using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Overhang;
using KinematicCharacterController;

public class ClimbShape : MonoBehaviour
{
    ClimbableMesh cm;

    RaycastHit hit;
    RaycastHit forwardProjectionSlideHit;

    public Transform CharacterModel;
    public Rigidbody rb;
    int index, lastIndex, previousLastIndex, previousIndex;

    Vector3 barycentricCoordinate, lastBarycentricCoordinate, barycentricCoordinateBehind;

    Plane plane, testPlane, cornerPlane;

    Vector3 cut, lastCut, testCut;

    int lastEdgeStart;
    int lastEdgeEnd;
    int lastEdgeOther;

    int previousLastEdgeStart;
    int previousLastEdgeEnd;
    int previousLastEdgeOther;

    float targetSpeed;

    [Min(0)]
    public float DirectionalSpeed, ClimbingSpeed;

    Vector3 currentCheckPosition;
    bool stop;

    Vector3 moveDirection;
    Vector3 castDirectionTest;

    public LayerMask layerMask;
    public LayerMask layerMaskForwardProjection;

    Vector3 input;
    Vector3 newPosition;

    public bool climbing;
    public bool onEdge;
    bool cutFound;

    int cornerReached;
    bool firstMoveDone;
    bool goForwardTest = false;

    float deltaTime;
    float frameRate;

    public MovementMode movementMode;
    Vector3 lastForward;
    Quaternion lastRotation;

    public bool forceSlide;
    public bool forceSlideForwardProjection;
    float slideCoefficient = 1;

    public Collider capsule;

    Vector3 checkPositionStart;
    Vector3 checkPositionEnd;
    Quaternion afterDepenetrateRotation;

    Vector3 previousRayCastPosition;

    void Awake()
    {
        Physics.autoSyncTransforms = true;
        UnityEngine.Cursor.visible = false;
        UnityEngine.Cursor.lockState = CursorLockMode.Locked;
        Application.targetFrameRate = 300;
        // set the plane used for pathfinding to be oriented to the character
        plane = new Plane(-transform.right, transform.position);

    }
    void Update()
    {
        // if (Input.GetKeyDown(KeyCode.LeftShift))
        // rb.velocity = transform.position - rb.transform.position;
    }
    void LateUpdate()
    {

        Physics.SyncTransforms();
        // Calculate the time it took to render the last frame
        deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;

        // Calculate the frame rate in frames per second
        frameRate = 1.0f / deltaTime;

        // if (frameRate < 60)
        // {
        //     Debug.Log("UH OH");
        // }

        // if not on a mesh, raycast to find a mesh
        if (!climbing)
        {
            TryStartClimb();
        }
        else
        {
            if (Input.GetKey(KeyCode.Space))
            {
                LeaveClimbableMesh();
                return;
            }
            float speed = targetSpeed;

            cm.RecalculateMesh(false);

            // Get input
            Vector3 tempInput;
            tempInput = new Vector3(Input.GetAxisRaw("Horizontal"), 0, Input.GetAxisRaw("Vertical")).normalized;
            // tempInput = Mathf2.RotateAroundAxis(tempInput, Vector3.up, Camera.main.transform.rotation.eulerAngles.y);

            // This is for debugging - makes the character go forwards and backwards over an edge rapidly, so I don't have to hammer the keyboard to test. 
            if (Input.GetKeyDown(KeyCode.K))
            {
                goForwardTest = !goForwardTest;
                // testPosition = transform.position.x;
            }
            if (goForwardTest)
            {
                // if (transform.position.x > testPosition)
                // {
                //     goForwardTest = true;
                // }
                // if (transform.position.x < testPosition - 0.005f)
                // {
                //     goForwardTest = false;
                // }
                // if (goForwardTest)
                // {
                tempInput = new Vector3(0, 0, 1);
                // }
                // else
                // {
                //     tempInput = new Vector3(0, 0, -1);
                // }
            }

            // if input has magnitude then use it
            if (tempInput.magnitude > 0)
            {
                input = tempInput;
            }

            // calculate movement on the mesh based on input
            Travel(tempInput, false, false);
            Vector3 pointOnTriangle = GetPositionFromBarycentric(barycentricCoordinate, lastEdgeStart, lastEdgeEnd, lastEdgeOther);
            transform.position = pointOnTriangle;


            int depenetrationIterations = 1;
            for (int i = 0; i < depenetrationIterations; i++)
            {
                bool isFinalPass = i == depenetrationIterations - 1;
                Depenetrate(isFinalPass);
                Physics.SyncTransforms();
                // Debug.Log(i + " " + (i == depenetrationIterations - 1));
                // EditorApplication.isPaused = true;
            }
            // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
            pointOnTriangle = GetPositionFromBarycentric(barycentricCoordinate, lastEdgeStart, lastEdgeEnd, lastEdgeOther);
            transform.position = pointOnTriangle;
        }

    }

    void Depenetrate(bool isFinalPass)
    {
        Travel(Vector3.zero, true, isFinalPass);
    }

    void OnDrawGizmos()
    {
        // Gizmos.color = Color.blue;
        // Gizmos.DrawWireSphere(checkPositionStart, 0.05f);
        // Gizmos.DrawWireSphere(checkPositionEnd, 0.05f);
    }


    void SetEdges(Vector3 previousLastEdgeStartPosition, Vector3 previousLastEdgeEndPosition, Vector3 previousLastEdgeOtherPosition, int index)
    {
        if (cm.Vertices[cm.Triangles[index]] == previousLastEdgeStartPosition)
        {
            lastEdgeStart = cm.Triangles[index];
        }
        if (cm.Vertices[cm.Triangles[index + 1]] == previousLastEdgeStartPosition)
        {
            lastEdgeStart = cm.Triangles[index + 1];
        }
        if (cm.Vertices[cm.Triangles[index + 2]] == previousLastEdgeStartPosition)
        {
            lastEdgeStart = cm.Triangles[index + 2];
        }

        if (cm.Vertices[cm.Triangles[index]] == previousLastEdgeEndPosition)
        {
            lastEdgeEnd = cm.Triangles[index];
        }
        if (cm.Vertices[cm.Triangles[index + 1]] == previousLastEdgeEndPosition)
        {
            lastEdgeEnd = cm.Triangles[index + 1];
        }
        if (cm.Vertices[cm.Triangles[index + 2]] == previousLastEdgeEndPosition)
        {
            lastEdgeEnd = cm.Triangles[index + 2];
        }

        if (cm.Vertices[cm.Triangles[index]] == previousLastEdgeOtherPosition)
        {
            lastEdgeOther = cm.Triangles[index];
        }
        if (cm.Vertices[cm.Triangles[index + 1]] == previousLastEdgeOtherPosition)
        {
            lastEdgeOther = cm.Triangles[index + 1];
        }
        if (cm.Vertices[cm.Triangles[index + 2]] == previousLastEdgeOtherPosition)
        {
            lastEdgeOther = cm.Triangles[index + 2];
        }
    }

    void Travel(Vector3 direction, bool depenetratePass, bool isFinalPass)
    {
        // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
        Vector3 pointOnTriangle = GetPositionFromBarycentric(barycentricCoordinate, lastEdgeStart, lastEdgeEnd, lastEdgeOther);
        // At the end of last loop we do a 'test' cut to get the next position BEHIND
        // This is recorded from the tri center
        Vector3 behindPointOnTriangle = GetPositionFromBarycentric(barycentricCoordinateBehind, lastEdgeStart, lastEdgeEnd, lastEdgeOther);
        // Quaternion rotationLast = transform.rotation;
        // At the end of last loop we do a 'test' cut to get the next position in FRONT
        // Here we recalculate it with deformations. This is the forward cut, NOT the movement direction cut 
        // This is also recorded from the tri center
        Vector3 forwardPointOnTriangle = GetPositionFromBarycentric(lastBarycentricCoordinate, lastEdgeStart, lastEdgeEnd, lastEdgeOther);

        transform.position = pointOnTriangle;

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        Vector3 tempForward = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        if (Input.GetKey(KeyCode.LeftShift))
        {
            if (movementMode != MovementMode.Car)
            {

                Vector3 previousLastEdgeStartPosition = cm.Vertices[lastEdgeStart];
                Vector3 previousLastEdgeEndPosition = cm.Vertices[lastEdgeEnd];
                Vector3 previousLastEdgeOtherPosition = cm.Vertices[lastEdgeOther];

                int indexTemp = cm.GetArea(index / 3) * 3;

                if (indexTemp != -3)
                {
                    index = indexTemp;
                    SetEdges(previousLastEdgeStartPosition, previousLastEdgeEndPosition, previousLastEdgeOtherPosition, index);

                    Vector3 triCenterTemp = (cm.Vertices[lastEdgeStart] + cm.Vertices[lastEdgeEnd] + cm.Vertices[lastEdgeOther]) / 3;

                    Plane plane = new Plane(-transform.right, triCenterTemp);

                    barycentricCoordinate = Mathf2.GetBarycentricCoordinates(newPosition, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);

                    testCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, transform.forward, triCenterTemp, plane, CutType.Start);
                    lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);
                    testCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, -transform.forward, triCenterTemp, plane, CutType.Test);
                    barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);

                    movementMode = MovementMode.Car;
                }
            }
        }
        else
        {
            if (movementMode != MovementMode.Directional)
            {
                Vector3 previousLastEdgeStartPosition = cm.Vertices[lastEdgeStart];
                Vector3 previousLastEdgeEndPosition = cm.Vertices[lastEdgeEnd];
                Vector3 previousLastEdgeOtherPosition = cm.Vertices[lastEdgeOther];

                int indexTemp = cm.GetMainBody(index / 3) * 3;

                cm.RecalculateMesh(false);

                if (indexTemp != -3)
                {
                    index = indexTemp;
                    onEdge = false;

                    SetEdges(previousLastEdgeStartPosition, previousLastEdgeEndPosition, previousLastEdgeOtherPosition, index);
                    Vector3 triCenterTemp = (cm.Vertices[lastEdgeStart] + cm.Vertices[lastEdgeEnd] + cm.Vertices[lastEdgeOther]) / 3;

                    Plane plane = new Plane(-transform.right, triCenterTemp);

                    testCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, transform.forward, triCenterTemp, plane, CutType.Start);
                    lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);
                    testCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, -transform.forward, triCenterTemp, plane, CutType.Test);
                    barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);

                    movementMode = MovementMode.Directional;
                }
            }
        }



        int lastEdgeStartReal = 0;
        int lastEdgeEndReal = 0;
        int lastEdgeOtherReal = 0;

        // Because of hard edges, the current triangle will have different vertices to what's currently stores in lastEdgeStart, lastEdgeEnd, lastEdgeOther
        // We need to check this triangle's edge positions against those and update.
        // lastEdgeStartReal,lastEdgeEndReal, lastEdgeOtherReal are the real indices of the current triangle
        if (!cm.TriangleAdjacencyInfo.ContainsKey(index))
        {
            Debug.Log("Index: " + index);
            foreach (var item in cm.TriangleAdjacencyInfo)
            {
                Debug.Log(item.Key);
#if UNITY_EDITOR
                EditorApplication.isPlaying = false;
#endif
            }
        }
        foreach (Edge e in cm.TriangleAdjacencyInfo[index].edges)
        {
            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeStart])
                lastEdgeStartReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeStart])
                lastEdgeStartReal = e.pointB;

            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeEnd])
                lastEdgeEndReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeEnd])
                lastEdgeEndReal = e.pointB;

            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeOther])
                lastEdgeOtherReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeOther])
                lastEdgeOtherReal = e.pointB;
        }


        // Calculate the ground normal based on coordinates from the last frame of where we should be standing, translated to the new, deformed triangle
        Vector3 groundNormal = GetNormalFromBarycentric(barycentricCoordinate, lastEdgeStartReal, lastEdgeEndReal, lastEdgeOtherReal);

        float turnAngle = 0;

        turnAngle += Input.GetKey(KeyCode.E) ? 100 * Time.deltaTime : 0;
        turnAngle -= Input.GetKey(KeyCode.Q) ? 100 * Time.deltaTime : 0;
        tempForward = Quaternion.Euler(0, turnAngle, 0) * tempForward;

        if (movementMode == MovementMode.Directional)
        {
            if (direction.magnitude > 0)
            {
                tempForward = lastForward;

            }
        }

        // In case of "viewing vector is equal to zero" error
        if (tempForward != Vector3.zero)
        {
            transform.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(tempForward, groundNormal), groundNormal); // set rotation to be towards the forward point.
        }
        else
        {
            Debug.Log("forward zero");
        }


        // Get the direction towards the edge 
        Vector3 triCenter = (cm.Vertices[lastEdgeStart] + cm.Vertices[lastEdgeEnd] + cm.Vertices[lastEdgeOther]) / 3;
        Vector3 closestPointOnEdge = Mathf2.NearestPointOnLine(cm.Vertices[lastEdgeStart], (cm.Vertices[lastEdgeStart] - cm.Vertices[lastEdgeEnd]).normalized, triCenter);
        Vector3 getNewtempForwardDirection = Vector3.zero;
        Vector3 getNewtempForwardDirectionBehind = Vector3.zero;

        if (movementMode == MovementMode.Directional)
        {
            // This check works as intended
            if (direction.magnitude > 0)
            {
                float angleToRotateby = Camera.main.transform.rotation.eulerAngles.y - transform.rotation.eulerAngles.y;
                input = Quaternion.Euler(0, angleToRotateby, 0) * input;

                // if (isFinalPass)
                // {
                // Debug.Log("Final Pass");
                if (!depenetratePass)
                {
                    afterDepenetrateRotation = Quaternion.LookRotation(Quaternion.Euler(0, transform.rotation.eulerAngles.y, 0) * input);
                }
                if (isFinalPass)
                {
                    CharacterModel.rotation = afterDepenetrateRotation;
                }
                Plane NewtempForwardPlane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterModel.right, transform.position);
                // Debug.DrawLine(transform.position, transform.position + NewtempForwardPlane.normal);
                Vector3 PlaneNormalRight = Mathf2.RotateAroundAxis(NewtempForwardPlane.normal, groundNormal, 90);
                // Debug.DrawLine(transform.position, transform.position + PlaneNormalRight, Color.blue);
                getNewtempForwardDirection = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, PlaneNormalRight, transform.position, NewtempForwardPlane, CutType.Test);
                getNewtempForwardDirectionBehind = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, -PlaneNormalRight, transform.position, NewtempForwardPlane, CutType.Test);
                // Debug.DrawLine(getNewtempForwardDirection, getNewtempForwardDirection + Vector3.up, Color.black);
                // Debug.DrawLine(getNewtempForwardDirectionBehind, getNewtempForwardDirectionBehind + Vector3.up, Color.black);

                // Debug.DrawLine(transform.position, transform.position + tempForward, Color.magenta);

                getNewtempForwardDirection = (getNewtempForwardDirection - getNewtempForwardDirectionBehind).normalized;
                tempForward = getNewtempForwardDirection;
                // tempForward = Vector3.forward;
                // Debug.DrawLine(transform.position, transform.position + tempForward, Color.magenta);
            }
            else
            {
                if (!depenetratePass)
                {
                    //     Debug.Log("Final Pass2");
                    afterDepenetrateRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(tempForward, Vector3.up).normalized, Vector3.up);

                }
                if (isFinalPass)
                {
                    if (!goForwardTest)
                    {
                        // EditorApplication.isPaused = true;
                    }
                    CharacterModel.rotation = afterDepenetrateRotation;
                }
            }
            targetSpeed = DirectionalSpeed;

        }
        else
        {
            targetSpeed = ClimbingSpeed;
            CharacterModel.localRotation = Quaternion.identity;
        }

        // This calculates the move direction. What happens if I just rotate this?
        // Get the move direction relative to the player's orientation
        moveDirection = (transform.rotation * input).normalized;

        // moveDirection = Mathf2.RotateAroundAxis(input, Vector3.up, Camera.main.transform.rotation.eulerAngles.y);

        // newDirection - can't remember what this does but it's possible it will change in the loop but we want to keep moveDirection the same.
        Vector3 newDirection = moveDirection;


        // transform.rotation = Quaternion.LookRotation(newDirection);

        //calculate distance to move




        // forceSlide = Physics.SphereCast(transform.position + transform.up * 0.05f, 0.049f, CharacterModel.up, out hit, 0.2f, layerMaskForwardProjection);
        // forceSlideForwardProjection2 = Physics.SphereCast(transform.position + transform.up * 0.1f + transform.right * 0.025f, 0.025f, moveDirection, out forwardProjectionSlideHit2, 0.04f, layerMaskForwardProjection);
        checkPositionStart = CharacterModel.position + CharacterModel.up * 0.05f;
        checkPositionEnd = CharacterModel.position + CharacterModel.up * 0.15f;
        Collider[] colliders = Physics.OverlapCapsule(checkPositionStart, checkPositionEnd, 0.05f, layerMaskForwardProjection);

        bool depenetrate = false;
        bool slide = false;
        float distance = direction.magnitude * targetSpeed * Time.deltaTime;

        if (depenetratePass)
        {
            Vector3 depenetrationDirection = Vector3.zero;
            Vector3 totalDepenetrationDirection = Vector3.zero;

            float depenetrationDistance = 0;

            if (colliders.Length > 0)
            {

                foreach (Collider col in colliders)
                {
                    Debug.Log("Found Collision " + col.name);
                    Physics.ComputePenetration(capsule, capsule.transform.position, capsule.transform.rotation, col, col.transform.position, col.transform.rotation, out depenetrationDirection, out depenetrationDistance);
                    Debug.DrawLine(transform.position, transform.position + depenetrationDirection * depenetrationDistance, Color.yellow);
                    if (depenetrationDistance > 0.00001f)
                    {
                        totalDepenetrationDirection += Vector3.ProjectOnPlane(depenetrationDirection * depenetrationDistance, groundNormal);
                        // Debug.Log(depenetrationDistance);
                        Debug.DrawLine(transform.position, transform.position + totalDepenetrationDirection, Color.red);
                    }
                }
                // Don't need to average this! Additive is better.
                // totalDepenetrationDirection /= (float)colliders.Length;

                Plane wallPlane = new();
                Vector3 wallNormal = depenetrationDirection.normalized;
                wallPlane.SetNormalAndPosition(wallNormal, depenetrationDirection * depenetrationDistance);

                Ray depenetrateRay = new Ray();
                depenetrateRay.direction = totalDepenetrationDirection.normalized;
                depenetrateRay.origin = Vector3.zero;
                wallPlane.Raycast(depenetrateRay, out distance);
                // Vector3 pointWhereDepenetrationDirectionAndGroundIntersect = ;
                distance *= 1f;

                // distance = totalDepenetrationDirection.magnitude * 0.9f;
                tempForward = totalDepenetrationDirection.normalized;

                moveDirection = tempForward;
                newDirection = tempForward;
                input = tempForward;
                depenetrate = true;
                plane = new Plane(Mathf2.RotateAroundAxis(tempForward, transform.up, 90), transform.position);
            }
            else
            {
                // distance = totalDepenetrationDirection.magnitude * 0.9f;
                tempForward = totalDepenetrationDirection.normalized;

                moveDirection = tempForward;
                newDirection = tempForward;
                input = tempForward;
                plane = new Plane(Mathf2.RotateAroundAxis(tempForward, transform.up, 90), transform.position);
                // Debug.Log("Collision not Found");
            }
        }
        else
        {
            // forceSlideForwardProjection = Physics.SphereCast(transform.position + CharacterModel.up * 0.2f - tempForward * 0.05f, 0.05f, tempForward, out forwardProjectionSlideHit, 0.05f, layerMaskForwardProjection);

            // if (forceSlideForwardProjection && Vector3.Dot(moveDirection, forwardProjectionSlideHit.normal) < 0f)
            // {
            //     Vector3 slideDirection = Vector3.Cross(forwardProjectionSlideHit.normal, groundNormal).normalized;

            //     if (Vector3.Dot(slideDirection, moveDirection) < 0)
            //     {
            //         slideDirection = -slideDirection;
            //     }

            //     // transform.position += Vector3.ProjectOnPlane(forwardProjectionSlideHit.normal, groundNormal).normalized * (0.05f - forwardProjectionSlideHit.distance);

            //     slideCoefficient = Mathf.Abs(Vector3.Dot(moveDirection, slideDirection));

            //     distance = direction.magnitude * slideCoefficient * targetSpeed * Time.deltaTime;
            //     tempForward = slideDirection;
            //     moveDirection = tempForward;
            //     newDirection = tempForward;
            //     input = tempForward;
            //     // Create plane to calculate "cuts" from
            //     plane = new Plane(Mathf2.RotateAroundAxis(slideDirection.normalized, CharacterModel.up, 90), transform.position);
            //     slide = true;
            // }

        }
        if (!depenetrate && !slide)
        {
            // Create plane to calculate "cuts" from
            if (movementMode == MovementMode.Car)
            {
                plane = new Plane(CharacterModel.rotation * (Quaternion.Euler(0, 90, 0) * input), transform.position);
            }
            else
            {
                plane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterModel.right, transform.position);
            }

        }

        /// IMPORTANT NOTE. I HAVE NO IDEA WHY THIS WORKS
        cornerPlane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterModel.up, transform.position);

        // total distance checked this frame - if goes over, loop will break
        float totalDistanceChecked = 0;



        // Questional fix for what????
        // QUESTIONABLE FIX... AND STILL BROKEN
        // If there is no input, do nothing
        // if (direction.magnitude == 0 && !Input.GetKeyDown(KeyCode.LeftShift) && !Input.GetKeyUp(KeyCode.LeftShift))
        // {
        //     return;
        // }



        // If on the edge (as of last frame) and holding away from said edge then not on edge any more
        if (onEdge && Vector3.Dot(moveDirection, (closestPointOnEdge - triCenter).normalized) < 0)
        {
            onEdge = false;
        }

        // reset cut found info
        // cut found triggers if any cut is found, even if we're standing on it
        cutFound = false;

        // if not on edge, do a start cut
        // start cut considers all edges
        cut = transform.position;



        if (!onEdge)
        {
            cut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, newDirection, transform.position, plane, CutType.Start);
        }
        else // else do a test cut - test cut considers all edges and returns a "cut" but doesn't change the current edge/index information. this is good because we want to be able to get an edge we are standing directly on.
        {
            cut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, newDirection, transform.position, plane, CutType.Test);
        }

        // Add to the total distance to the cut from the position
        totalDistanceChecked += Vector3.Distance(transform.position, cut);
        // make the currentCheckPosition the same as cut
        currentCheckPosition = cut;

        // i is used to break out of infinite while loops
        int i = 0;

        // newPosition is updated as we walk through the mesh
        newPosition = Vector3.zero;
        // if we didn't reach the next edge with this movement, calculate the new position in the current triangle
        if (totalDistanceChecked >= distance)
        {
            newPosition = transform.position + Vector3.ClampMagnitude(cut - transform.position, distance);

            // Debug.Log("Clamp magnitude " + Vector3.ClampMagnitude(cut - transform.position, distance).magnitude);
            // Debug.Log("Distance " + distance);
        }
        else // else make the next check go from the point on the next edge
        {
            newPosition = cut;
        }


        float remainingDistance = distance - Vector3.Distance(transform.position, newPosition);
        castDirectionTest = transform.forward;

        // Why do we need lastIndex, previousLastIndex and previousIndex?

        // lastIndex is
        // previousLastIndex is
        // previousIndex is

        previousLastIndex = index;
        previousLastEdgeStart = lastEdgeStart;
        previousLastEdgeEnd = lastEdgeEnd;
        previousLastEdgeOther = lastEdgeOther;

        // while we have checked less distance than the edge
        while (totalDistanceChecked < distance)
        {
            // if not on edge / just started moving away from edge
            if (!onEdge)
            {
                // set the last index to be the index - this is because we're about to check the next tri to see if we're on an edge, and we want to compare index to lastIndex
                lastIndex = index;

                // take the edge we just passed, then get the tri that we didn't just check
                // don't worry, index is set inGetNextTri() 
                int[] nextTri = GetNextTri();
                // if GetNextTri() returns same value as index, we are on an edge, else do nothing
                if (index == lastIndex)
                {
                    onEdge = true;
                    Debug.Log(onEdge);
                }
                else
                {
                    cutFound = false;
                    Vector3 lastCut = cut;

                    cut = GetNextCut(nextTri[0], nextTri[1], nextTri[2], newDirection, newPosition, plane, CutType.Next);

                    totalDistanceChecked += Vector3.Distance(currentCheckPosition, cut);

                    Vector3 oldPosition = newPosition;

                    if (totalDistanceChecked >= distance)
                    {
                        newPosition = newPosition + Vector3.ClampMagnitude(cut - newPosition, remainingDistance);
                    }
                    else
                    {
                        newPosition = cut;
                    }

                    newPosition = currentCheckPosition + Vector3.ClampMagnitude(cut - currentCheckPosition, remainingDistance);

                    remainingDistance = remainingDistance - Vector3.Distance(oldPosition, newPosition);

                    // project direction on current normal to get new direction
                    // direction is used to see if cuts are valid/ in the direction if your input

                    if (cut != currentCheckPosition)
                    {
                        newDirection = (cut - currentCheckPosition).normalized;
                        lastCut = currentCheckPosition;
                        currentCheckPosition = cut;
                    }

                    i++;
                    if (i > 100)
                    {
                        lastEdgeStart = previousLastEdgeStart;
                        lastEdgeEnd = previousLastEdgeEnd;
                        lastEdgeOther = previousLastEdgeOther;
                        index = previousLastIndex;
                        totalDistanceChecked = distance;

                        break;
                    }

                    Plane tempPlane = new Plane(-transform.right, transform.position);

                }
            }
            else // if on edge
            {
                // get the point where the character is trying to move, if it moved off the current tri into space
                Vector3 movePositionAttempt = newPosition + newDirection * remainingDistance;
                Vector3 slidePoint = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd]);
                cornerPlane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterModel.up, slidePoint);
                int currentCornerInt = -1;
                int currentCornerIntOther = 0;
                if (slidePoint == cm.Vertices[lastEdgeStart])
                {
                    currentCornerInt = lastEdgeStart;
                    currentCornerIntOther = lastEdgeEnd;
                }
                if (slidePoint == cm.Vertices[lastEdgeEnd])
                {
                    currentCornerInt = lastEdgeEnd;
                    currentCornerIntOther = lastEdgeStart;
                }

                // if we reached a corner
                if (currentCornerInt != -1)
                {
                    // Get the next edge. Since lastEdgeStart and lastEdgeEnd are always the ouside edge when edge-sliding, the next edge is the corner reached -> lastEdgeOther
                    Edge nextEdgeOnThisTriangle = new Edge();
                    if (currentCornerInt < lastEdgeOther)
                    {
                        nextEdgeOnThisTriangle.pointA = currentCornerInt;
                        nextEdgeOnThisTriangle.pointB = lastEdgeOther;
                    }
                    else
                    {
                        nextEdgeOnThisTriangle.pointA = lastEdgeOther;
                        nextEdgeOnThisTriangle.pointB = currentCornerInt;
                    }
                    // If the next edge is another outside edge on the same triangle, switch to that. 
                    if (cm.EdgeAdjacencyInfo.ContainsKey(nextEdgeOnThisTriangle) && cm.EdgeAdjacencyInfo[nextEdgeOnThisTriangle].triangleA == cm.EdgeAdjacencyInfo[nextEdgeOnThisTriangle].triangleB)
                    {

                        lastEdgeOther = currentCornerIntOther;
                        lastEdgeStart = nextEdgeOnThisTriangle.pointA;
                        lastEdgeEnd = nextEdgeOnThisTriangle.pointB;

                        Ray tempRay = CreateRay(newPosition, movePositionAttempt);
                        Plane tempPlane = new Plane((slidePoint - newPosition).normalized, slidePoint);
                        float hitDistance;
                        tempPlane.Raycast(tempRay, out hitDistance);
                        if (hitDistance < 0.001f)
                        {
                            totalDistanceChecked = distance;
                        }
                        else
                        {
                            totalDistanceChecked += hitDistance;
                        }
                    }
                    else // If the next edge isn't an outside edge on the same triangle
                    {

                        Ray tempRay = CreateRay(newPosition, movePositionAttempt);
                        Plane tempPlane = new Plane((slidePoint - newPosition).normalized, slidePoint);

                        float hitDistance;
                        tempPlane.Raycast(tempRay, out hitDistance);
                        if (hitDistance < 0.001f)
                        {
                            totalDistanceChecked = distance;
                        }
                        else
                        {
                            totalDistanceChecked += hitDistance;
                        }

                        if (slidePoint == cm.Vertices[lastEdgeStart])
                        {
                            cornerReached = lastEdgeStart;
                        }
                        else if (slidePoint == cm.Vertices[lastEdgeEnd])
                        {
                            cornerReached = lastEdgeEnd;
                        }

                        int timesLooped = 0;
                        int count = 0;
                        List<Edge> tempEdges = new List<Edge>();

                        Edge checkIsOnEdge = new();

                        int lastEdgeStartRealTemp = -1;
                        int lastEdgeEndRealTemp = -1;
                        Debug.Log(index);
                        // check the next triangle's edges, we're setting lastEdgeStartRealTemp, lastEdgeStartEndTemp, lastEdgeOtherRealTemp because we don't want to override lastEdgeStartReal etc
                        foreach (Edge e in cm.TriangleAdjacencyInfo[index].edges)
                        {

                            // if the positions of the points on the line we crossed match the edge of the triangle being compared currently, store it
                            if (cm.Vertices[lastEdgeStart] == cm.Vertices[e.pointA])
                            {
                                lastEdgeStartRealTemp = e.pointA;
                            }
                            if (cm.Vertices[lastEdgeStart] == cm.Vertices[e.pointB])
                            {
                                lastEdgeStartRealTemp = e.pointB;
                            }
                            if (cm.Vertices[lastEdgeEnd] == cm.Vertices[e.pointA])
                            {
                                lastEdgeEndRealTemp = e.pointA;
                            }
                            if (cm.Vertices[lastEdgeEnd] == cm.Vertices[e.pointB])
                            {
                                lastEdgeEndRealTemp = e.pointB;
                            }
                            if (lastEdgeStartRealTemp == -1)
                            {
                                if (cm.Vertices[lastEdgeOther] == cm.Vertices[e.pointA])
                                {
                                    lastEdgeStartRealTemp = e.pointA;
                                }
                                if (cm.Vertices[lastEdgeOther] == cm.Vertices[e.pointB])
                                {
                                    lastEdgeStartRealTemp = e.pointB;
                                }
                            }
                            if (lastEdgeEndRealTemp == -1)
                            {
                                if (cm.Vertices[lastEdgeOther] == cm.Vertices[e.pointA])
                                {
                                    lastEdgeEndRealTemp = e.pointA;
                                }
                                if (cm.Vertices[lastEdgeOther] == cm.Vertices[e.pointB])
                                {
                                    lastEdgeEndRealTemp = e.pointB;
                                }
                            }
                            // Debug.DrawLine(cm.meshVerts[e.pointA],cm.meshVerts[e.pointB],Color.cyan);

                            // Debug.Log("e.PointA " + cm.meshVerts[e.pointA]);
                            // Debug.Log("e.PointB " + cm.meshVerts[e.pointB]);
                            // Debug.Log("cm.meshVerts[lastEdgeStart] " + cm.meshVerts[lastEdgeStart]);
                            // Debug.Log("cm.meshVerts[lastEdgeEnd] " + cm.meshVerts[lastEdgeEnd]);
                        }
                        if (lastEdgeStartRealTemp == -1 && lastEdgeEndRealTemp == -1)
                        {
                            DebugTriangle(index, Color.blue);
#if UNITY_EDITOR
                            EditorApplication.isPaused = true;
#endif
                        }
                        // Debug.DrawLine(transform.position,transform.position + moveDirection,Color.black);

                        // Debug.DrawLine(cm.meshVerts[lastEdgeStart],cm.meshVerts[lastEdgeOther],Color.green);
                        // Debug.DrawLine(cm.meshVerts[lastEdgeStart],cm.meshVerts[lastEdgeEnd],Color.green);
                        // Debug.DrawLine(cm.meshVerts[lastEdgeOther],cm.meshVerts[lastEdgeEnd],Color.green);
                        // Debug.Log("lastEdgeStartRealTemp " + lastEdgeStartRealTemp);
                        // Debug.Log("lastEdgeEndRealTemp " + lastEdgeEndRealTemp);
                        checkIsOnEdge.pointA = lastEdgeStartRealTemp;
                        checkIsOnEdge.pointB = lastEdgeEndRealTemp;

                        if (!cm.EdgeAdjacencyInfo.ContainsKey(checkIsOnEdge))
                        {
                            checkIsOnEdge.pointA = lastEdgeEndRealTemp;
                            checkIsOnEdge.pointB = lastEdgeStartRealTemp;
                        }

                        if (checkIsOnEdge.pointA == -1 || checkIsOnEdge.pointB == -1)
                        {
#if UNITY_EDITOR
                            EditorApplication.isPaused = true;
#endif
                        }
                        // Debug.Log(checkIsOnEdge.triangleA);
                        // Debug.Log(checkIsOnEdge.triangleB);
                        // Debug.Log(lastEdgeStartRealTemp);
                        // Debug.Log(lastEdgeEndRealTemp);
                        if (cm.EdgeAdjacencyInfo[checkIsOnEdge].triangleA == cm.EdgeAdjacencyInfo[checkIsOnEdge].triangleB)
                        {
                            foreach (Edge e in cm.EdgesAttachedToCorner[cornerReached])
                            {
                                if (!(cm.Vertices[e.pointA] == cm.Vertices[lastEdgeStart] && cm.Vertices[e.pointB] == cm.Vertices[lastEdgeEnd]) &&
                                    !(cm.Vertices[e.pointA] == cm.Vertices[lastEdgeEnd] && cm.Vertices[e.pointB] == cm.Vertices[lastEdgeStart]))
                                {
                                    tempEdges.Add(e);
                                    count++;
                                }
                            }
                        }
                        else
                        {
                            Edge cornerEdge1 = new Edge();
                            Edge cornerEdge2 = new Edge();
                            int cornerEdgeOther1 = -1;
                            int cornerEdgeOther2 = -1;
                            Edge firstEdgeCrossed = new Edge();
                            int triangleCheckCorner = index;
                            int lastTriangleCheckCorner = -1;

                            Vector3 triCenterTemp1 = Vector3.zero;
                            Vector3 triCenterTemp2 = Vector3.zero;
                            Vector3 cornerEdgeNormalTemp1 = Vector3.zero;
                            Vector3 cornerEdgeNormalTemp2 = Vector3.zero;

                            bool firstCheck = false;
                            bool edgeFound = false;

                            while (!edgeFound)
                            {
                                foreach (Edge e in cm.EdgesAttachedToCorner[cornerReached])
                                {
                                    // Debug.DrawLine(cm.meshVerts[e.pointA],cm.meshVerts[e.pointB],Color.magenta);
                                    Vector3 triCenterTemp = (cm.Vertices[triangleCheckCorner] + cm.Vertices[triangleCheckCorner + 1] + cm.Vertices[triangleCheckCorner + 2]) / 3;
                                    Vector3 closestPointTemp = Mathf2.GetClosestPointOnFiniteLine(triCenterTemp, cm.Vertices[e.pointA], cm.Vertices[e.pointB]);
                                    Vector3 edgeNormalTemp = (triCenterTemp - closestPointTemp).normalized;

                                    int edgeTriangleA = cm.EdgeAdjacencyInfo[e].triangleA;
                                    int edgeTriangleB = cm.EdgeAdjacencyInfo[e].triangleB;

                                    if (edgeTriangleA == triangleCheckCorner && !edgeFound)
                                    {
                                        if (edgeTriangleA == edgeTriangleB)
                                        {
                                            cornerEdge1 = e;
                                            edgeFound = true;
                                            if (!firstCheck)
                                            {
                                                firstEdgeCrossed = e;
                                                firstCheck = true;
                                            }
                                        }
                                        else
                                        {
                                            if (lastTriangleCheckCorner != edgeTriangleB)
                                            {
                                                lastTriangleCheckCorner = triangleCheckCorner;
                                                triangleCheckCorner = edgeTriangleB;
                                                if (!firstCheck)
                                                {
                                                    firstEdgeCrossed = e;
                                                    firstCheck = true;
                                                }
                                            }
                                        }
                                    }
                                    else if (edgeTriangleB == triangleCheckCorner && !edgeFound)
                                    {
                                        if (edgeTriangleA == edgeTriangleB)
                                        {
                                            cornerEdge1 = e;
                                            edgeFound = true;
                                            if (!firstCheck)
                                            {
                                                firstEdgeCrossed = e;
                                                firstCheck = true;
                                            }
                                        }
                                        else
                                        {
                                            if (lastTriangleCheckCorner != edgeTriangleA)
                                            {
                                                lastTriangleCheckCorner = triangleCheckCorner;
                                                triangleCheckCorner = edgeTriangleA;
                                            }
                                            if (!firstCheck)
                                            {
                                                firstEdgeCrossed = e;
                                                firstCheck = true;
                                            }
                                        }
                                    }
                                }
                                timesLooped++;
                                if (timesLooped > 100)
                                {
                                    Debug.Log("BAD");
                                    break;
                                }
                            }

                            foreach (Edge e in cm.TriangleAdjacencyInfo[cm.EdgeAdjacencyInfo[cornerEdge1].triangleA].edges)
                            {
                                if (cm.Vertices[e.pointA] != cm.Vertices[lastEdgeStart] && cm.Vertices[e.pointA] != cm.Vertices[lastEdgeEnd])
                                {
                                    cornerEdgeOther1 = e.pointA;
                                }
                                else if (cm.Vertices[e.pointB] != cm.Vertices[lastEdgeStart] && cm.Vertices[e.pointB] != cm.Vertices[lastEdgeEnd])
                                {
                                    cornerEdgeOther1 = e.pointB;
                                }
                            }

                            triCenterTemp1 = (cm.Vertices[cornerEdge1.pointA] + cm.Vertices[cornerEdge1.pointB] + cm.Vertices[cornerEdgeOther1]) / 3;
                            cornerEdgeNormalTemp1 = (triCenterTemp1 - Mathf2.NearestPointOnLine(cm.Vertices[cornerEdge1.pointA], (cm.Vertices[cornerEdge1.pointA] - cm.Vertices[cornerEdge1.pointB]).normalized, triCenterTemp1)).normalized;

                            timesLooped = 0;
                            triangleCheckCorner = index;
                            lastTriangleCheckCorner = -1;

                            firstCheck = false;
                            edgeFound = false;

                            while (!edgeFound)
                            {
                                foreach (Edge e in cm.EdgesAttachedToCorner[cornerReached])
                                {
                                    int edgeTriangleA = cm.EdgeAdjacencyInfo[e].triangleA;
                                    int edgeTriangleB = cm.EdgeAdjacencyInfo[e].triangleB;

                                    if (edgeTriangleA == triangleCheckCorner && !edgeFound)
                                    {
                                        if (!(cm.Vertices[e.pointA] == cm.Vertices[firstEdgeCrossed.pointA] && cm.Vertices[e.pointB] == cm.Vertices[firstEdgeCrossed.pointB] ||
                                            cm.Vertices[e.pointA] == cm.Vertices[firstEdgeCrossed.pointB] && cm.Vertices[e.pointB] == cm.Vertices[firstEdgeCrossed.pointA]))
                                        {
                                            if (edgeTriangleA == edgeTriangleB)
                                            {
                                                cornerEdge2 = e;
                                                edgeFound = true;
                                            }
                                            else
                                            {
                                                if (lastTriangleCheckCorner != edgeTriangleB)
                                                {
                                                    lastTriangleCheckCorner = triangleCheckCorner;
                                                    triangleCheckCorner = edgeTriangleB;
                                                }
                                            }
                                        }
                                    }
                                    else if (edgeTriangleB == triangleCheckCorner && !edgeFound)
                                    {
                                        if (!(cm.Vertices[e.pointA] == cm.Vertices[firstEdgeCrossed.pointA] && cm.Vertices[e.pointB] == cm.Vertices[firstEdgeCrossed.pointB] ||
                                            cm.Vertices[e.pointA] == cm.Vertices[firstEdgeCrossed.pointB] && cm.Vertices[e.pointB] == cm.Vertices[firstEdgeCrossed.pointA]))
                                        {
                                            if (edgeTriangleA == edgeTriangleB)
                                            {
                                                cornerEdge2 = e;
                                                edgeFound = true;
                                            }
                                            else
                                            {
                                                if (lastTriangleCheckCorner != edgeTriangleA)
                                                {
                                                    lastTriangleCheckCorner = triangleCheckCorner;
                                                    triangleCheckCorner = edgeTriangleA;
                                                }
                                            }
                                        }
                                    }
                                }
                                timesLooped++;
                                if (timesLooped > 100)
                                {
                                    Debug.Log("BAD");
                                    break;
                                }
                            }

                            foreach (Edge e in cm.TriangleAdjacencyInfo[cm.EdgeAdjacencyInfo[cornerEdge2].triangleA].edges)
                            {
                                if (cm.Vertices[e.pointA] != cm.Vertices[lastEdgeStart] && cm.Vertices[e.pointA] != cm.Vertices[lastEdgeEnd])
                                {
                                    cornerEdgeOther2 = e.pointA;
                                }
                                else if (cm.Vertices[e.pointB] != cm.Vertices[lastEdgeStart] && cm.Vertices[e.pointB] != cm.Vertices[lastEdgeEnd])
                                {
                                    cornerEdgeOther2 = e.pointB;
                                }
                            }

                            triCenterTemp2 = (cm.Vertices[cornerEdge2.pointA] + cm.Vertices[cornerEdge2.pointB] + cm.Vertices[cornerEdgeOther2]) / 3;
                            cornerEdgeNormalTemp2 = (triCenterTemp2 - Mathf2.NearestPointOnLine(cm.Vertices[cornerEdge2.pointA], (cm.Vertices[cornerEdge2.pointA] - cm.Vertices[cornerEdge2.pointB]).normalized, triCenterTemp2)).normalized;

                            timesLooped = 0;

                            foreach (Edge e in cm.EdgesAttachedToCorner[cornerReached])
                            {
                                tempEdges.Add(e);
                                count++;
                            }

                            if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, cm.Vertices[cornerEdge1.pointA], cm.Vertices[cornerEdge1.pointB]) == cm.Vertices[cornerReached] &&
                                Vector3.Dot(cornerEdgeNormalTemp1, moveDirection) < 0)
                            {
                                tempEdges.Remove(cornerEdge1);
                                index = cm.EdgeAdjacencyInfo[cornerEdge1].triangleA;

                                if (cm.Vertices[cornerEdge1.pointA] == cm.Vertices[cornerReached])
                                {
                                    lastEdgeEnd = cornerEdge1.pointA;
                                    lastEdgeStart = cornerEdge1.pointB;
                                }
                                else if (cm.Vertices[cornerEdge1.pointB] == cm.Vertices[cornerReached])
                                {
                                    lastEdgeEnd = cornerEdge1.pointB;
                                    lastEdgeStart = cornerEdge1.pointA;
                                }


                                lastEdgeOther = cornerEdgeOther1;
                            }
                            else if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, cm.Vertices[cornerEdge2.pointA], cm.Vertices[cornerEdge2.pointB]) == cm.Vertices[cornerReached])
                            {
                                tempEdges.Remove(cornerEdge2);
                                index = cm.EdgeAdjacencyInfo[cornerEdge2].triangleA;

                                if (cm.Vertices[cornerEdge2.pointA] == cm.Vertices[cornerReached])
                                {
                                    lastEdgeEnd = cornerEdge2.pointA;
                                    lastEdgeStart = cornerEdge2.pointB;
                                }
                                else if (cm.Vertices[cornerEdge2.pointB] == cm.Vertices[cornerReached])
                                {
                                    lastEdgeEnd = cornerEdge2.pointB;
                                    lastEdgeStart = cornerEdge2.pointA;
                                }

                                lastEdgeOther = cornerEdgeOther2;
                            }
                        }

                        bool foundNextEdge = false;
                        int x = 0;
                        int tempIndex = index;
                        int tempLastIndex = lastIndex;

                        List<Edge> checkedEdges = new List<Edge>();
                        List<Edge> checkedEdgesCorner = new List<Edge>();
                        // if we haven't found an outer edge

                        int cornerEdgeStart = -1;
                        int cornerEdgeEnd = -1;
                        int cornerEdgeOther = -1;
                        int cornerTriangleIndex = tempIndex;
                        while (cornerEdgeStart == -1)
                        {
                            foreach (Edge e in cm.TriangleAdjacencyInfo[cornerTriangleIndex].edges)
                            {
                                // We need to find the other corner edge
                                bool tempEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in tempEdges)
                                {
                                    if (cm.Vertices[e.pointA] == cm.Vertices[edgeToCheck.pointA] && cm.Vertices[e.pointB] == cm.Vertices[edgeToCheck.pointB]
                                     || cm.Vertices[e.pointA] == cm.Vertices[edgeToCheck.pointB] && cm.Vertices[e.pointB] == cm.Vertices[edgeToCheck.pointA])
                                    {
                                        tempEdgesContainsEdge = true;
                                    }
                                }
                                bool checkedEdgesCornerContainsEdge = false;
                                foreach (Edge edgeToCheck in checkedEdgesCorner)
                                {
                                    if (cm.Vertices[e.pointA] == cm.Vertices[edgeToCheck.pointA] && cm.Vertices[e.pointB] == cm.Vertices[edgeToCheck.pointB]
                                     || cm.Vertices[e.pointA] == cm.Vertices[edgeToCheck.pointB] && cm.Vertices[e.pointB] == cm.Vertices[edgeToCheck.pointA])
                                    {
                                        checkedEdgesCornerContainsEdge = true;
                                    }
                                }
                                // if the edge is one of the edges attached to the current corner
                                if (tempEdgesContainsEdge && !checkedEdgesCornerContainsEdge)
                                {
                                    Debug.Log("A");
                                    // Debug.DrawLine(cm.meshVerts[e.pointA], cm.meshVerts[e.pointB], Color.red);
                                    // and it's an outside edge
                                    if (cm.EdgeAdjacencyInfo[e].triangleA == cm.EdgeAdjacencyInfo[e].triangleB)
                                    {
                                        Debug.Log("A1");
                                        if (cm.Vertices[e.pointA] == cm.Vertices[cornerReached])
                                        {
                                            Debug.Log("A1_1");
                                            cornerEdgeStart = e.pointA;
                                            cornerEdgeEnd = e.pointB;

                                        }
                                        else if (cm.Vertices[e.pointB] == cm.Vertices[cornerReached])
                                        {
                                            Debug.Log("A1_2");
                                            cornerEdgeStart = e.pointB;
                                            cornerEdgeEnd = e.pointA;
                                        }
                                        foreach (Edge e2 in cm.TriangleAdjacencyInfo[cornerTriangleIndex].edges)
                                        {
                                            if (e2.pointA != cornerEdgeStart && e2.pointA != cornerEdgeEnd)
                                            {
                                                Debug.Log("A1_3");
                                                cornerEdgeOther = e2.pointA;
                                            }
                                            if (e2.pointB != cornerEdgeStart && e2.pointB != cornerEdgeEnd)
                                            {
                                                Debug.Log("A1_4");
                                                cornerEdgeOther = e2.pointB;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        // else switch to the tri on the other side of the edge
                                        if (cornerTriangleIndex == cm.EdgeAdjacencyInfo[e].triangleA)
                                        {
                                            cornerTriangleIndex = cm.EdgeAdjacencyInfo[e].triangleB;
                                        }
                                        else
                                        {
                                            cornerTriangleIndex = cm.EdgeAdjacencyInfo[e].triangleA;
                                        }
                                        foreach (Edge e2 in cm.TriangleAdjacencyInfo[cornerTriangleIndex].edges)
                                        {
                                            if (cm.Vertices[e2.pointA] == cm.Vertices[e.pointA] && cm.Vertices[e2.pointB] == cm.Vertices[e.pointB]
                                            || cm.Vertices[e2.pointA] == cm.Vertices[e.pointA] && cm.Vertices[e2.pointB] == cm.Vertices[e.pointB])
                                            {
                                                if (!checkedEdgesCorner.Contains(e2))
                                                {
                                                    checkedEdgesCorner.Add(e2);
                                                }
                                            }
                                        }
                                        if (!checkedEdgesCorner.Contains(e))
                                        {
                                            checkedEdgesCorner.Add(e);
                                        }
                                    }
                                }
                            }

                            timesLooped++;
                            if (timesLooped > 100)
                            {
#if UNITY_EDITOR
                                EditorApplication.isPaused = true;
#endif
                                Debug.Log("Couldn't find corner's end edge");
                                Debug.Log("cornerEdgeStart " + cornerEdgeStart);
                                Debug.Log("cornerEdgeEnd " + cornerEdgeEnd);
                                Debug.Log("cornerEdgeOther " + cornerEdgeOther);
                                Debug.Log("cornerTriangleIndex " + cornerTriangleIndex);
                                Debug.Log("cornerReached " + cornerReached);
                                break;
                            }
                        }
                        while (!foundNextEdge)
                        {
                            // we need to use a different method for contains - checking by their positions, since hard edges has different indices

                            // scan the edges of the current triangle
                            foreach (Edge e in cm.TriangleAdjacencyInfo[tempIndex].edges)
                            {

                                // we need to use a different method for contains - checking by their positions, since hard edges has different indices

                                bool tempEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in tempEdges)
                                {
                                    if (cm.Vertices[e.pointA] == cm.Vertices[edgeToCheck.pointA] && cm.Vertices[e.pointB] == cm.Vertices[edgeToCheck.pointB]
                                     || cm.Vertices[e.pointA] == cm.Vertices[edgeToCheck.pointB] && cm.Vertices[e.pointB] == cm.Vertices[edgeToCheck.pointA])
                                    {
                                        tempEdgesContainsEdge = true;
                                    }
                                }
                                bool checkedEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in checkedEdges)
                                {
                                    if (cm.Vertices[e.pointA] == cm.Vertices[edgeToCheck.pointA] && cm.Vertices[e.pointB] == cm.Vertices[edgeToCheck.pointB]
                                     || cm.Vertices[e.pointA] == cm.Vertices[edgeToCheck.pointB] && cm.Vertices[e.pointB] == cm.Vertices[edgeToCheck.pointA])
                                    {
                                        checkedEdgesContainsEdge = true;
                                    }
                                }
                                // if the edge is one of the edges attached to the current corner
                                if (tempEdgesContainsEdge && !checkedEdgesContainsEdge)
                                {
                                    // and it's an outside edge
                                    if (cm.EdgeAdjacencyInfo[e].triangleA == cm.EdgeAdjacencyInfo[e].triangleB)
                                    {
                                        foundNextEdge = true;
                                        previousLastIndex = lastIndex;
                                        previousIndex = index;

                                        previousLastEdgeStart = lastEdgeStart;
                                        previousLastEdgeEnd = lastEdgeEnd;
                                        previousLastEdgeOther = lastEdgeOther;

                                        lastEdgeStart = e.pointA;
                                        lastEdgeEnd = e.pointB;

                                        lastIndex = tempLastIndex;
                                        index = tempIndex;

                                        foreach (Edge p in cm.TriangleAdjacencyInfo[index].edges)
                                        {
                                            if (cm.Vertices[p.pointA] != cm.Vertices[lastEdgeStart] && cm.Vertices[p.pointA] != cm.Vertices[lastEdgeEnd])
                                            {
                                                lastEdgeOther = p.pointA;
                                            }
                                            else if (cm.Vertices[p.pointB] != cm.Vertices[lastEdgeStart] && cm.Vertices[p.pointB] != cm.Vertices[lastEdgeEnd])
                                            {
                                                lastEdgeOther = p.pointB;
                                            }
                                        }

                                        triCenter = (cm.Vertices[lastEdgeStart] + cm.Vertices[lastEdgeEnd] + cm.Vertices[lastEdgeOther]) / 3;
                                        closestPointOnEdge = Mathf2.NearestPointOnLine(cm.Vertices[lastEdgeStart], (cm.Vertices[lastEdgeStart] - cm.Vertices[lastEdgeEnd]).normalized, triCenter);

                                        Vector3 previousTriCenter = (cm.Vertices[previousLastEdgeStart] + cm.Vertices[previousLastEdgeEnd] + cm.Vertices[previousLastEdgeOther]) / 3;
                                        Vector3 previousClosestPointOnEdge = Mathf2.NearestPointOnLine(cm.Vertices[previousLastEdgeStart], (cm.Vertices[previousLastEdgeStart] - cm.Vertices[previousLastEdgeEnd]).normalized, previousTriCenter);

                                        newPosition = slidePoint;

                                        Vector3 cornerNormal = Vector3.Cross(cm.Vertices[previousLastEdgeStart] - cm.Vertices[previousLastEdgeEnd], cm.Vertices[lastEdgeStart] - cm.Vertices[lastEdgeEnd]).normalized;
                                        if (Vector3.Dot(cornerNormal, transform.up) < 0)
                                        {
                                            cornerNormal = -cornerNormal;
                                        }
                                        Vector3 newEdgeNormal = (closestPointOnEdge - triCenter).normalized;
                                        Vector3 lastEdgeNormal = (previousClosestPointOnEdge - previousTriCenter).normalized;

                                        Vector3 newFarCorner = cm.Vertices[currentCornerInt] == cm.Vertices[lastEdgeStart] ? cm.Vertices[lastEdgeEnd] : cm.Vertices[lastEdgeStart];
                                        Vector3 lastFarCorner = cm.Vertices[currentCornerInt] == cm.Vertices[previousLastEdgeStart] ? cm.Vertices[previousLastEdgeEnd] : cm.Vertices[previousLastEdgeStart];

                                        int shouldInvertNewEdgeNormal = 1;
                                        Vector3 cornerDirection = Vector3.ProjectOnPlane(transform.rotation * input, cornerNormal).normalized;

                                        Vector3 newEdgeNormal2 = Vector3.ProjectOnPlane(newEdgeNormal * shouldInvertNewEdgeNormal, cornerNormal).normalized;
                                        Vector3 lastEdgeNormal2 = Vector3.ProjectOnPlane(lastEdgeNormal, cornerNormal).normalized;
                                        // Debug.DrawLine(closestPointOnEdge, closestPointOnEdge + newEdgeNormal2, Color.yellow);
                                        // Debug.DrawLine(previousClosestPointOnEdge, previousClosestPointOnEdge + lastEdgeNormal2, Color.yellow);
                                        // Debug.DrawLine(transform.position, transform.position + cornerNormal, Color.red);
                                        if (Vector3.Dot((transform.rotation * input).normalized, (closestPointOnEdge - triCenter).normalized) < 0)
                                        {
                                            onEdge = false;
                                            foundNextEdge = true;
                                            Debug.Log("Found Edge");
                                        }

                                        Vector3 slidePoint2 = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, cm.Vertices[previousLastEdgeStart], cm.Vertices[previousLastEdgeEnd]);
                                        Vector3 slidePoint2Clamped = Mathf2.NearestPointOnLine(cm.Vertices[previousLastEdgeStart], cm.Vertices[previousLastEdgeEnd], movePositionAttempt);
                                        Vector3 slidePoint3 = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd]);
                                        Vector3 slidePoint3Clamped = Mathf2.NearestPointOnLine(cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], movePositionAttempt);
                                        // This is for stopping the check from bouncing between edges when stuck in a corner
                                        // Problem is that it's sticking to corners when the GetClosestPointOnLine attemptedMovePosition is outside the line

                                        Ray ray1 = new Ray();
                                        ray1.origin = closestPointOnEdge;
                                        ray1.direction = -newEdgeNormal;
                                        Ray ray2 = new Ray();
                                        ray2.origin = previousClosestPointOnEdge;
                                        ray2.direction = -lastEdgeNormal;

                                        Quaternion angleBetweenTris = Quaternion.FromToRotation(lastEdgeNormal2, newEdgeNormal2);

                                        // In order to determine if we should be able to move round the corner, we need to know if it's a convex corner or not
                                        // DoRaysIntersect of the edge normals determines this.
                                        // If they do, then we check if the closest point on each edge to the attempted move position is equal to the corner - that's the only situation where we should move round the corner  
                                        if (
                                            DoRaysIntersect(ray1, ray2) &&
                                            //  slidePoint2 == cm.meshVerts[cornerReached]
                                            IsTriAfterNextThis(cornerEdgeStart, cornerEdgeEnd, cornerEdgeOther)
                                        )
                                        {
                                            Debug.Log("BUH");
                                            // NOT SWITCHING TO OTHER EDGE SO OF COURSE SLIDEPOINT IS STILL AT THE CORNER
                                            onEdge = true;
                                            lastEdgeStart = previousLastEdgeStart;
                                            lastEdgeEnd = previousLastEdgeEnd;
                                            lastEdgeOther = previousLastEdgeOther;

                                            index = previousIndex;

                                            lastIndex = previousLastIndex;
                                            totalDistanceChecked = distance;
                                            newPosition = slidePoint;
                                            slidePoint = cm.Vertices[cornerReached];
                                            cornerPlane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterModel.up, slidePoint);
                                            foundNextEdge = true;
                                            Debug.Log("Found Edge");
                                        }

                                    }
                                    else
                                    {
                                        int nextTriLastEdgeStart = -1;
                                        int nextTriLastEdgeEnd = -1;
                                        int nextTriLastEdgeOther = -1;
                                        // else switch to the tri on the other side of the edge
                                        tempLastIndex = tempIndex;
                                        if (tempIndex == cm.EdgeAdjacencyInfo[e].triangleA)
                                        {
                                            tempIndex = cm.EdgeAdjacencyInfo[e].triangleB;
                                        }
                                        else
                                        {
                                            tempIndex = cm.EdgeAdjacencyInfo[e].triangleA;
                                        }
                                        checkedEdges.Add(e);
                                        if (cm.Vertices[e.pointA] == cm.Vertices[cornerReached])
                                        {
                                            nextTriLastEdgeStart = e.pointA;
                                            nextTriLastEdgeEnd = e.pointB;
                                        }
                                        else
                                        {
                                            nextTriLastEdgeStart = e.pointB;
                                            nextTriLastEdgeEnd = e.pointA;
                                        }
                                        foreach (Edge e2 in cm.TriangleAdjacencyInfo[tempIndex].edges)
                                        {
                                            if (cm.Vertices[e2.pointA] != cm.Vertices[nextTriLastEdgeStart] && cm.Vertices[e2.pointA] != cm.Vertices[nextTriLastEdgeEnd])
                                            {
                                                nextTriLastEdgeOther = e2.pointA;
                                            }
                                            if (cm.Vertices[e2.pointB] != cm.Vertices[nextTriLastEdgeStart] && cm.Vertices[e2.pointB] != cm.Vertices[nextTriLastEdgeEnd])
                                            {
                                                nextTriLastEdgeOther = e2.pointB;
                                            }
                                        }

                                        // check if we are pointing towards the other edge (not the next edge attached to corner)
                                        // if so, we can come unstuck.

                                        if (GetFarEdgeCut(cornerReached, tempIndex, nextTriLastEdgeStart, nextTriLastEdgeEnd, nextTriLastEdgeOther, cornerEdgeStart, cornerEdgeEnd, cornerEdgeOther, plane))
                                        {
                                            // Debug.DrawLine("BUH");
                                            index = tempIndex;
                                            lastIndex = tempLastIndex;
                                            lastEdgeStart = nextTriLastEdgeEnd;
                                            lastEdgeEnd = nextTriLastEdgeStart;
                                            lastEdgeOther = nextTriLastEdgeOther;
                                            // foreach (Edge p in cm.triangleInfos[index].edges)
                                            // {
                                            //     if (cm.meshVerts[p.pointA] != cm.meshVerts[lastEdgeStart] && cm.meshVerts[p.pointA] != cm.meshVerts[lastEdgeEnd])
                                            //     {
                                            //         lastEdgeOther = p.pointA;
                                            //     } 
                                            //     else if (cm.meshVerts[p.pointB] != cm.meshVerts[lastEdgeStart] && cm.meshVerts[p.pointB] != cm.meshVerts[lastEdgeEnd])
                                            //     {
                                            //         lastEdgeOther = p.pointB;
                                            //     }
                                            // }
                                            // Debug.DrawLine(cm.meshVerts[lastEdgeStart],cm.meshVerts[lastEdgeStart] + Vector3.up, Color.green);
                                            // Debug.DrawLine(cm.meshVerts[lastEdgeEnd],cm.meshVerts[lastEdgeEnd] + Vector3.up, Color.blue);
                                            // Debug.DrawLine(cm.meshVerts[lastEdgeOther],cm.meshVerts[lastEdgeOther] + Vector3.up, Color.yellow);
                                            // EditorApplication.isPaused = true;
                                            onEdge = false;

                                            foundNextEdge = true;
                                            Debug.Log("Found Edge");

                                            totalDistanceChecked = distance;
                                        }
                                        else
                                        {

                                        }
                                    }
                                }
                            }

                            x++;
                            if (x > 100)
                            {
#if UNITY_EDITOR
                                EditorApplication.isPaused = true;
#endif
                                Debug.Log("couldn't find next edge");

                                // Debug.DrawLine(cm.meshVerts[lastEdgeStart], cm.meshVerts[lastEdgeEnd], Color.magenta);

                                return;
                            }
                        }


                    }
                    // Debug.Log(newPosition);
                    newPosition = slidePoint;
                    // Debug.Log(newPosition);
                }
                else
                {
                    totalDistanceChecked = distance;
                    // Debug.Log(newPosition);
                    newPosition = slidePoint;
                    // Debug.Log(newPosition);
                }

                remainingDistance = remainingDistance - Vector3.Distance(newPosition, slidePoint);

                plane = new Plane(transform.rotation * Quaternion.Euler(0, 90, 0) * input, newPosition);

                i++;
                if (i > 1000)
                {
                    Debug.Log("WHOOPS " + index);
#if UNITY_EDITOR
                    EditorApplication.isPaused = true;
#endif
                    break;
                }
            }
        }

        // testing measurements, line should jitter if deltatime is wrong.
        // Debug.DrawLine(transform.position, newPosition, Color.red);
        // newPosition = transform.position;

        barycentricCoordinate = Mathf2.GetBarycentricCoordinates(newPosition, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);

        Vector3 updatedPosition = barycentricCoordinate.x * cm.Vertices[lastEdgeStart] +
                                barycentricCoordinate.y * cm.Vertices[lastEdgeEnd] +
                                barycentricCoordinate.z * cm.Vertices[lastEdgeOther];



        triCenter = (cm.Vertices[lastEdgeStart] + cm.Vertices[lastEdgeEnd] + cm.Vertices[lastEdgeOther]) / 3;

        Vector3 testPosition = barycentricCoordinate.x * cm.Vertices[lastEdgeStart] +
                    barycentricCoordinate.y * cm.Vertices[lastEdgeEnd] +
                    barycentricCoordinate.z * cm.Vertices[lastEdgeOther];

        foreach (Edge e in cm.TriangleAdjacencyInfo[index].edges)
        {
            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeStart])
                lastEdgeStartReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeStart])
                lastEdgeStartReal = e.pointB;

            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeEnd])
                lastEdgeEndReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeEnd])
                lastEdgeEndReal = e.pointB;

            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeOther])
                lastEdgeOtherReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeOther])
                lastEdgeOtherReal = e.pointB;
        }

        groundNormal = GetNormalFromBarycentric(barycentricCoordinate, lastEdgeStartReal, lastEdgeEndReal, lastEdgeOtherReal);

        castDirectionTest = Quaternion.FromToRotation(transform.up, groundNormal) * transform.forward;

        Vector3 v = Vector3.Cross(castDirectionTest, groundNormal).normalized;

        if (movementMode == MovementMode.Car)
        {
            testPlane = new Plane(-(Quaternion.FromToRotation(transform.up, groundNormal) * transform.right), triCenter);
        }
        else
        {
            testPlane = new Plane(-(Quaternion.FromToRotation(transform.up, groundNormal) * transform.right), triCenter);
            if (isFinalPass)
                testPlane = new Plane(-CharacterModel.right, triCenter);
            // Debug.DrawLine(ch)
        }
        testCut = Vector3.zero;
        testCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, castDirectionTest, triCenter, testPlane, CutType.Test);

        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);
        testCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, -castDirectionTest, triCenter, testPlane, CutType.Test);
        barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);
        //    Debug.DrawLine(transform.position, transform.position + plane.normal, Color.blue);


        if (getNewtempForwardDirection != Vector3.zero)
        {
            lastForward = getNewtempForwardDirection;
        }
        else
        {
            lastForward = CharacterModel.forward;
        }
        lastRotation = CharacterModel.rotation;

        // Debug.DrawLine(CharacterModel.position, CharacterModel.position + CharacterModel.forward, Color.red);
    }

    bool IsTriAfterNextThis(int lastEdgeStartReal, int lastEdgeEndReal, int lastEdgeOtherReal)
    {
        int tempLastEdgeStartReal = -1;
        int tempLastEdgeEndReal = -1;
        int tempLastEdgeOtherReal = -1;

        Vector3 tempBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(newPosition, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);

        Vector3 updatedPosition = tempBarycentricCoordinate.x * cm.Vertices[lastEdgeStart] +
                                tempBarycentricCoordinate.y * cm.Vertices[lastEdgeEnd] +
                                tempBarycentricCoordinate.z * cm.Vertices[lastEdgeOther];



        Vector3 tempTriCenter = (cm.Vertices[lastEdgeStart] + cm.Vertices[lastEdgeEnd] + cm.Vertices[lastEdgeOther]) / 3;

        Vector3 testPosition = barycentricCoordinate.x * cm.Vertices[lastEdgeStart] +
                    barycentricCoordinate.y * cm.Vertices[lastEdgeEnd] +
                    barycentricCoordinate.z * cm.Vertices[lastEdgeOther];

        foreach (Edge e in cm.TriangleAdjacencyInfo[index].edges)
        {
            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeStart])
                tempLastEdgeStartReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeStart])
                tempLastEdgeStartReal = e.pointB;

            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeEnd])
                tempLastEdgeEndReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeEnd])
                tempLastEdgeEndReal = e.pointB;

            if (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeOther])
                tempLastEdgeOtherReal = e.pointA;
            else if (cm.Vertices[e.pointB] == cm.Vertices[lastEdgeOther])
                tempLastEdgeOtherReal = e.pointB;
        }

        Vector3 tempGroundNormal = GetNormalFromBarycentric(tempBarycentricCoordinate, tempLastEdgeStartReal, tempLastEdgeEndReal, tempLastEdgeOtherReal);

        Vector3 tempCastDirectionTest = Quaternion.FromToRotation(transform.up, tempGroundNormal) * transform.forward;

        Vector3 v = Vector3.Cross(tempCastDirectionTest, tempGroundNormal).normalized;

        Plane tempTestPlane = new Plane(-(Quaternion.FromToRotation(transform.up, tempGroundNormal) * transform.right), tempTriCenter);

        Vector3 tempTestCut = Vector3.zero;
        tempTestCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, tempCastDirectionTest, tempTriCenter, tempTestPlane, CutType.Test);

        /// ATTEMPTED FIX

        // Vector3 tempCastDirectionTest;

        //         Plane tempTestPlane;

        //         Vector3 tempTestCut = Vector3.zero;
        //         if (movementMode == MovementMode.Car)
        //         {
        //             tempCastDirectionTest = Quaternion.FromToRotation(transform.up, tempGroundNormal) * transform.forward;
        //             tempTestPlane = new Plane(-(Quaternion.FromToRotation(transform.up, tempGroundNormal) * transform.right), tempTriCenter);
        //         }
        //         else
        //         {
        //             tempCastDirectionTest = transform.forward;
        //             tempTestPlane = new Plane(-CharacterModel.right, tempTriCenter);

        //         }


        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        Vector3 tempLastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(tempTestCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);
        tempTestCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, -tempCastDirectionTest, tempTriCenter, tempTestPlane, CutType.Test);
        Vector3 tempBarycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(tempTestCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);
        Vector3 behindPointOnTriangle = GetPositionFromBarycentric(tempBarycentricCoordinateBehind, lastEdgeStart, lastEdgeEnd, lastEdgeOther);
        // At the end of last loop we do a 'test' cut to get the next position in FRONT
        // Here we recalculate it with deformations. This is the forward cut, NOT the movement direction cut 
        // This is also recorded from the tri center
        Vector3 forwardPointOnTriangle = GetPositionFromBarycentric(tempLastBarycentricCoordinate, lastEdgeStart, lastEdgeEnd, lastEdgeOther);

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        Vector3 tempForward = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        Quaternion tempRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(tempForward, tempGroundNormal), tempGroundNormal); // set rotation to be towards the forward point.

        Vector3 tempMovePositionAttempt = cm.Vertices[cornerReached] + (tempRotation * input).normalized;

        Vector3 slidePoint2 = Mathf2.GetClosestPointOnFiniteLine(tempMovePositionAttempt, cm.Vertices[previousLastEdgeStart], cm.Vertices[previousLastEdgeEnd]);
        Vector3 slidePoint3 = Mathf2.GetClosestPointOnFiniteLine(tempMovePositionAttempt, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd]);
        // This is for stopping the check from bouncing between edges when stuck in a corner
        // Problem is that it's sticking to corners when the GetClosestPointOnLine attemptedMovePosition is outside the line

        // In order to determine if we should be able to move round the corner, we need to know if it's a convex corner or not
        // DoRaysIntersect of the edge normals determines this.
        // If they do, then we check if the closest point on each edge to the attempted move position is equal to the corner - that's the only situation where we should move round the corner  


        if (slidePoint3 == cm.Vertices[cornerReached])
        {
            Debug.Log("tri repeat");
            return true;
        }
        else
        {
            Debug.Log("no tri repeat");
            // Debug.DrawLine(cm.meshVerts[cornerReached], tempMovePositionAttempt, Color.magenta);
            return false;
        }


    }

    bool GetFarEdgeCut(int currentCornerInt, int triangleIndex, int nextTriLastEdgeStart, int nextTriLastEdgeEnd, int nextTriLastEdgeOther, int cornerEdgeStart, int cornerEdgeEnd, int cornerEdgeOther, Plane plane)
    {
        Vector3 triCenter = (cm.Vertices[cornerEdgeStart] + cm.Vertices[cornerEdgeEnd] + cm.Vertices[cornerEdgeOther]) / 3;
        Vector3 closestPointOnEdge = Mathf2.NearestPointOnLine(cm.Vertices[cornerEdgeStart], (cm.Vertices[cornerEdgeStart] - cm.Vertices[cornerEdgeEnd]).normalized, triCenter);
        Vector3 nextTriCenter = (cm.Vertices[nextTriLastEdgeStart] + cm.Vertices[nextTriLastEdgeEnd] + cm.Vertices[nextTriLastEdgeOther]) / 3;
        Vector3 nextTriClosestPointOnEdge = Mathf2.NearestPointOnLine(cm.Vertices[nextTriLastEdgeStart], (cm.Vertices[nextTriLastEdgeStart] - cm.Vertices[nextTriLastEdgeEnd]).normalized, nextTriCenter);

        Vector3 previousTriCenter = (cm.Vertices[lastEdgeStart] + cm.Vertices[lastEdgeEnd] + cm.Vertices[lastEdgeOther]) / 3;
        Vector3 previousClosestPointOnEdge = Mathf2.NearestPointOnLine(cm.Vertices[lastEdgeStart], (cm.Vertices[lastEdgeStart] - cm.Vertices[lastEdgeEnd]).normalized, previousTriCenter);

        Vector3 cornerNormal = Vector3.Cross(cm.Vertices[cornerEdgeStart] - cm.Vertices[cornerEdgeOther], cm.Vertices[cornerEdgeStart] - cm.Vertices[cornerEdgeEnd]).normalized;
        if (Vector3.Dot(cornerNormal, transform.up) < 0)
        {
            cornerNormal = -cornerNormal;
        }
        Vector3 edgeNormal = (closestPointOnEdge - triCenter).normalized;
        Vector3 lastEdgeNormal = (previousClosestPointOnEdge - previousTriCenter).normalized;
        Vector3 nextTriEdgeNormal = (nextTriClosestPointOnEdge - nextTriCenter).normalized;
        Vector3 edgeNormal2 = Vector3.ProjectOnPlane(edgeNormal, cornerNormal).normalized;
        Vector3 lastEdgeNormal2 = Vector3.ProjectOnPlane(lastEdgeNormal, cornerNormal).normalized;
        Vector3 tempGroundNormal = GetNormalFromBarycentric(barycentricCoordinate, nextTriLastEdgeStart, nextTriLastEdgeEnd, nextTriLastEdgeOther);
        Vector3 nextTriEdgeNormal2 = Vector3.ProjectOnPlane(nextTriEdgeNormal, transform.up).normalized;
        Vector3 tempRight = Vector3.Cross(tempGroundNormal, nextTriEdgeNormal).normalized;

        bool planeHitsFarEdge = false;

        // if move direction is facing into the wall that we're sliding on
        Vector3 cornerAdjustedMoveDirection = Vector3.ProjectOnPlane((transform.rotation * input).normalized, cornerNormal).normalized;
        if (Vector3.Dot((transform.rotation * input).normalized, nextTriEdgeNormal2) < 0 && Vector3.Dot(cornerAdjustedMoveDirection, edgeNormal2) < 0)
        {
            float farEdgeHitDistance = 0;
            foreach (Edge e in cm.TriangleAdjacencyInfo[triangleIndex].edges)
            {
                if (cm.Vertices[e.pointA] != cm.Vertices[currentCornerInt] && cm.Vertices[e.pointB] != cm.Vertices[currentCornerInt])
                {
                    Ray farEdgeRay = CreateRay(cm.Vertices[e.pointA], cm.Vertices[e.pointB]);
                    if (plane.Raycast(farEdgeRay, out farEdgeHitDistance))
                    {
                        if (farEdgeHitDistance > 0 && farEdgeHitDistance <= Vector3.Distance(cm.Vertices[e.pointA], cm.Vertices[e.pointB]))
                        {
                            planeHitsFarEdge = true;
                        }
                    }
                }
            }
        }

        return planeHitsFarEdge;
    }

    Vector3 GetAdjustedNormal(int normalIndex)
    {
        return cm.transform.TransformDirection(cm.Normals[normalIndex]);
    }

    Vector3 GetNextCut(int p1, int p2, int p3, Vector3 direction, Vector3 position, Plane plane, CutType cutType)
    {
        int p1Temp = p1;
        int p2Temp = p2;
        int p3Temp = p3;

        if (index != lastIndex && cutType != CutType.Test && cutType != CutType.Start)
        {
            if (cm.Vertices[p1] == cm.Vertices[lastEdgeStart])
                lastEdgeStart = p1;
            else if (cm.Vertices[p1] == cm.Vertices[lastEdgeEnd])
                lastEdgeEnd = p1;
            else if (cm.Vertices[p1] == cm.Vertices[lastEdgeOther])
                lastEdgeOther = p1;

            if (cm.Vertices[p2] == cm.Vertices[lastEdgeStart])
                lastEdgeStart = p2;
            else if (cm.Vertices[p2] == cm.Vertices[lastEdgeEnd])
                lastEdgeEnd = p2;
            else if (cm.Vertices[p2] == cm.Vertices[lastEdgeOther])
                lastEdgeOther = p2;

            if (cm.Vertices[p3] == cm.Vertices[lastEdgeStart])
                lastEdgeStart = p3;
            else if (cm.Vertices[p3] == cm.Vertices[lastEdgeEnd])
                lastEdgeEnd = p3;
            else if (cm.Vertices[p3] == cm.Vertices[lastEdgeOther])
                lastEdgeOther = p3;

        }

        // If we're starting or testing, we're doing so from a point within the triangle and we test all edges.
        // Otherwise make the next p1 the START edge, p2 the OTHER edge, and p3 the END edge
        // We do this because the edge that's about to be the one we pass can only be an other edge than the current start->end edge, so we want to be sure not to check start->end
        if (cutType == CutType.Next && lastIndex != index)
        {
            // we want p1 to be lastEdgeStart
            if (cm.Vertices[lastEdgeStart] == cm.Vertices[p1])
                p1Temp = p1;
            if (cm.Vertices[lastEdgeStart] == cm.Vertices[p2])
                p1Temp = p2;
            if (cm.Vertices[lastEdgeStart] == cm.Vertices[p3])
                p1Temp = p3;
            // p2 to be lastEdgeOther
            if (cm.Vertices[p1] != cm.Vertices[lastEdgeStart] && cm.Vertices[p1] != cm.Vertices[lastEdgeEnd])
                p2Temp = p1;
            if (cm.Vertices[p2] != cm.Vertices[lastEdgeStart] && cm.Vertices[p2] != cm.Vertices[lastEdgeEnd])
                p2Temp = p2;
            if (cm.Vertices[p3] != cm.Vertices[lastEdgeStart] && cm.Vertices[p3] != cm.Vertices[lastEdgeEnd])
                p2Temp = p3;
            // we want p3 to be lastEdgeEnd
            if (cm.Vertices[lastEdgeEnd] == cm.Vertices[p1])
                p3Temp = p1;
            if (cm.Vertices[lastEdgeEnd] == cm.Vertices[p2])
                p3Temp = p2;
            if (cm.Vertices[lastEdgeEnd] == cm.Vertices[p3])
                p3Temp = p3;
        }



        // create rays of edges to use with planar casts

        // START >> OTHER edge
        Ray ray1 = CreateRay(cm.Vertices[p1Temp], cm.Vertices[p2Temp]);
        // END >> OTHER edge
        Ray ray2 = CreateRay(cm.Vertices[p2Temp], cm.Vertices[p3Temp]);
        // START >> END edge
        Ray ray3 = CreateRay(cm.Vertices[p1Temp], cm.Vertices[p3Temp]);

        // record ray's lengths, since Ray.direction does not have a magnitude
        // in order of size, because that's how edges are defined.
        float ray1Length = Vector3.Distance(cm.Vertices[p1Temp], cm.Vertices[p2Temp]);
        float ray2Length = Vector3.Distance(cm.Vertices[p2Temp], cm.Vertices[p3Temp]);
        float ray3Length = Vector3.Distance(cm.Vertices[p1Temp], cm.Vertices[p3Temp]);

        // a variable for Plane.Raycast to store distance in
        float hitDistance1 = -1;
        float hitDistance2 = -1;
        float hitDistance3 = -1;
        // temporarily make result: position, so that if all else fails we get put back in the same spot
        Vector3 tempResult = position;
        // OTHER >> START edge
        // if the edge intersects the plane...
        plane.Raycast(ray1, out hitDistance1);
        plane.Raycast(ray2, out hitDistance2);
        plane.Raycast(ray3, out hitDistance3);
        if (firstMoveDone)
        {
            if (hitDistance1 > 0)
            {
                if (cutType == CutType.Start)
                {
                }
                //... and the point along the ray is within the length of the edge...
                if (hitDistance1 <= ray1Length
                 || hitDistance1 > ray1Length && hitDistance2 > ray2Length && hitDistance3 < ray3Length && hitDistance3 > 0
                 || hitDistance1 > ray1Length && hitDistance3 > ray3Length && hitDistance2 < ray2Length && hitDistance2 > 0
                 )
                {
                    // and the cut type is next or
                    // start/test AND the intersection point is in front of the player
                    if (cutType == CutType.Next ||
                       ((cutType == CutType.Start || cutType == CutType.Test) &&
                       (Vector3.Dot(direction.normalized, (position - ray1.GetPoint(hitDistance1)).normalized) <= 0
                 || hitDistance1 > ray1Length && hitDistance2 > ray2Length && hitDistance3 < ray3Length && hitDistance3 > 0
                 || hitDistance1 > ray1Length && hitDistance3 > ray3Length && hitDistance2 < ray2Length && hitDistance2 > 0)))
                    {
                        if (cutType == CutType.Start)
                        {

                        }
                        tempResult = ray1.GetPoint(hitDistance1);

                        // if we're not just doing a test probe, return the relevant edges. The edge we're testing here is p1 to p2, so we make those start and end
                        if (cutType != CutType.Test)
                        {
                            lastEdgeStart = p1Temp;
                            lastEdgeEnd = p2Temp;
                            lastEdgeOther = p3Temp;

                            if (cutType == CutType.Start)
                            {

                            }
                        }

                    }
                }
            }
            else if (hitDistance1 == 0)
            {
                if (cutType != CutType.Test && lastIndex != index)
                {
                    tempResult = cm.Vertices[p1Temp];
                    lastEdgeStart = p1Temp;
                    lastEdgeEnd = p2Temp;
                    lastEdgeOther = p3Temp;
                    cutFound = true;
                }
            }

            // END >> OTHER edge
            if (hitDistance2 > 0)
            {
                // same as last time, it shouldn't be able to hit if the last one already did
                if (hitDistance2 <= ray2Length
                 || hitDistance2 > ray2Length && hitDistance1 > ray1Length && hitDistance3 < ray3Length && hitDistance3 > 0
                 || hitDistance2 > ray2Length && hitDistance3 > ray3Length && hitDistance1 < ray1Length && hitDistance1 > 0
                 )
                {

                    if (cutType == CutType.Next ||
                       ((cutType == CutType.Start || cutType == CutType.Test) &&
                       (Vector3.Dot(direction.normalized, (position - ray2.GetPoint(hitDistance2)).normalized) <= 0
                 || hitDistance2 > ray2Length && hitDistance1 > ray1Length && hitDistance3 < ray3Length && hitDistance3 > 0
                 || hitDistance2 > ray2Length && hitDistance3 > ray3Length && hitDistance1 < ray1Length && hitDistance1 > 0)))
                    {
                        if (cutType == CutType.Start)
                        {
                        }
                        tempResult = ray2.GetPoint(hitDistance2);
                        if (cutType != CutType.Test)
                        {
                            lastEdgeStart = p2Temp;
                            lastEdgeEnd = p3Temp;
                            lastEdgeOther = p1Temp;
                            cutFound = true;

                            if (cutType == CutType.Start)
                            {
                            }
                        }
                    }
                }
            }
            else if (hitDistance2 == 0 && !cutFound)
            {
                if (cutType != CutType.Test && lastIndex != index)
                {
                    tempResult = cm.Vertices[p2Temp];
                    lastEdgeStart = p2Temp;
                    lastEdgeEnd = p3Temp;
                    lastEdgeOther = p1Temp;
                }
            }


            // we only test the START >> END edge if it's the start of a new loop or a test because we just passed through this
            if (hitDistance3 > 0)
            {
                if (cutType == CutType.Start)
                {
                }
                if (hitDistance3 <= ray3Length
                 || hitDistance3 > ray3Length && hitDistance1 > ray1Length && hitDistance2 < ray2Length && hitDistance2 > 0
                 || hitDistance3 > ray3Length && hitDistance2 > ray2Length && hitDistance1 < ray1Length && hitDistance1 > 0
                 )
                {
                    if (((cutType == CutType.Start || cutType == CutType.Test) && (
                    (Vector3.Dot(direction.normalized, (position - ray3.GetPoint(hitDistance3)).normalized) <= 0
                     || hitDistance3 > ray3Length && hitDistance1 > ray1Length && hitDistance2 < ray2Length && hitDistance2 > 0
                     || hitDistance3 > ray3Length && hitDistance2 > ray2Length && hitDistance1 < ray1Length && hitDistance1 > 0)))
                    )
                    {
                        if (cutType == CutType.Start)
                        {
                        }
                        tempResult = ray3.GetPoint(hitDistance3);
                        if (cutType != CutType.Test)
                        {
                            lastEdgeStart = p1Temp;
                            lastEdgeEnd = p3Temp;
                            lastEdgeOther = p2Temp;
                            cutFound = true;
                        }
                    }
                }
            }
            else if (hitDistance3 == 0 && !cutFound)
            {
                if (cutType != CutType.Test && lastIndex != index)
                {
                    tempResult = cm.Vertices[p1Temp];
                    lastEdgeStart = p1Temp;
                    lastEdgeEnd = p3Temp;
                    lastEdgeOther = p2Temp;
                    cutFound = true;
                }
            }
        }

        firstMoveDone = true;
        return tempResult;
    }

    Ray CreateRay(Vector3 point1, Vector3 point2) => new Ray { origin = point1, direction = (point2 - point1).normalized };

    int[] GetNextTri()
    {
        // Get the three adjacent edges of the triangle we're currently checking
        Edge[] adjacentEdges = cm.TriangleAdjacencyInfo[index].edges;

        foreach (Edge e in adjacentEdges)
        {
            // Debug.Log(e.triangleA);
            // Debug.Log(e.triangleB);
            // Get the edge of the three that is equal to the edge that has just been passed
            bool edgeMatches = (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeStart] && cm.Vertices[e.pointB] == cm.Vertices[lastEdgeEnd]) ||
                (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeEnd] && cm.Vertices[e.pointB] == cm.Vertices[lastEdgeStart]);

            if (edgeMatches)
            {
                int nextTriangle = cm.EdgeAdjacencyInfo[e].triangleA == cm.EdgeAdjacencyInfo[e].triangleB
                    ? cm.EdgeAdjacencyInfo[e].triangleA // If adjacent triangles are the same, return the same triangle
                    : cm.EdgeAdjacencyInfo[e].triangleA != index
                        ? cm.EdgeAdjacencyInfo[e].triangleA // If edge has two triangles and the first is not equal to the current triangle, return that
                        : cm.EdgeAdjacencyInfo[e].triangleB; // Otherwise, return the second, that's all that's left

                // Set index to the new triangle
                index = nextTriangle;

                return new int[3]
                {
                    cm.Triangles[nextTriangle],
                    cm.Triangles[nextTriangle + 1],
                    cm.Triangles[nextTriangle + 2]
                };
            }
        }

        return default;
    }

    void TryStartClimb()
    {
        firstMoveDone = false;
        RaycastHit hit;
        if (Physics.Raycast(transform.position + transform.up * 0.1f, -transform.up, out hit, 0.2f, layerMask)
        || Physics.Raycast(previousRayCastPosition, (previousRayCastPosition - transform.position).normalized, out hit, Vector3.Distance(previousRayCastPosition, transform.position), layerMask))
        {
            GameObject temp = hit.collider.gameObject;
            cm = temp.GetComponent<ClimbableMesh>();

            climbing = true;
            transform.position = hit.point;
            index = hit.triangleIndex * 3;
            Debug.Log(hit.triangleIndex);
            Debug.Log(index);
            barycentricCoordinate = hit.barycentricCoordinate;

            lastEdgeStart = cm.Triangles[index];
            lastEdgeEnd = cm.Triangles[index + 1];
            lastEdgeOther = cm.Triangles[index + 2];

            Vector3 triCenter = (cm.Vertices[lastEdgeStart] + cm.Vertices[lastEdgeEnd] + cm.Vertices[lastEdgeOther]) / 3;

            Plane plane = new Plane(-transform.right, triCenter);

            testCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, transform.forward, triCenter, plane, CutType.Start);
            lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);
            testCut = GetNextCut(lastEdgeStart, lastEdgeEnd, lastEdgeOther, -transform.forward, triCenter, plane, CutType.Test);
            barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[lastEdgeStart], cm.Vertices[lastEdgeEnd], cm.Vertices[lastEdgeOther]);
        }
        previousRayCastPosition = transform.position + transform.up * 0.1f;
    }
    void LeaveClimbableMesh()
    {
        // MYSTERY
        transform.position = CharacterModel.position;
        rb.position = CharacterModel.position;
        rb.GetComponent<KinematicCharacterMotor>().SetPositionAndRotation(CharacterModel.position, CharacterModel.rotation);
        climbing = false;
    }

    Vector3 Tr_(Vector3 v)
    {
        return cm.transform.TransformPoint(v);
    }

    Vector3 GetPositionFromBarycentric(Vector3 barycentricCoordinate, int lastEdgeStart, int lastEdgeEnd, int lastEdgeOther)
    {
        return barycentricCoordinate.x * cm.Vertices[lastEdgeStart] +
               barycentricCoordinate.y * cm.Vertices[lastEdgeEnd] +
               barycentricCoordinate.z * cm.Vertices[lastEdgeOther];
    }
    Vector3 GetNormalFromBarycentric(Vector3 barycentricCoordinate, int lastEdgeStart, int lastEdgeEnd, int lastEdgeOther)
    {
        return (barycentricCoordinate.x * cm.Normals[lastEdgeStart] +
                barycentricCoordinate.y * cm.Normals[lastEdgeEnd] +
                barycentricCoordinate.z * cm.Normals[lastEdgeOther]).normalized;
    }

    void DebugTriangle(int triangle, Color color)
    {
        if (cm.EdgeAdjacencyInfo[cm.TriangleAdjacencyInfo[triangle].edges[0]].triangleB != -1)
            Debug.DrawLine(cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[0].pointA], cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[0].pointB], color, 0);

        if (cm.EdgeAdjacencyInfo[cm.TriangleAdjacencyInfo[triangle].edges[1]].triangleB != -1)
            Debug.DrawLine(cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[1].pointA], cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[1].pointB], color, 0);

        if (cm.EdgeAdjacencyInfo[cm.TriangleAdjacencyInfo[triangle].edges[2]].triangleB != -1)
            Debug.DrawLine(cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[2].pointA], cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[2].pointB], color, 0);
    }

    void OnGUI()
    {
        // Display the frame rate on the screen
        GUIStyle style = new GUIStyle();
        style.normal.textColor = Color.white;
        style.fontSize = 20;
        Rect rect = new Rect(10, 10, 200, 30);
        GUI.Label(rect, "Frame Rate: " + frameRate.ToString("F2"), style);
    }

    enum CutType
    {
        Start,
        Next,
        Test
    }

    bool DoRaysIntersect(Ray ray1, Ray ray2)
    {
        Vector3 v1 = ray1.direction.normalized;
        Vector3 v2 = ray2.direction.normalized;
        Vector3 cross = Vector3.Cross(v1, v2);
        float denominator = cross.magnitude;
        if (denominator == 0f)
        {
            return false; // rays are parallel
        }
        // if (Vector3.Dot(v1,v2) > 0)
        // {
        //     return false;
        // }
        Vector3 p1 = ray1.origin;
        Vector3 p2 = ray2.origin;
        Vector3 p1_to_p2 = p2 - p1;
        float t1 = Vector3.Dot(Vector3.Cross(p1_to_p2, v2), cross) / denominator;
        float t2 = Vector3.Dot(Vector3.Cross(p1_to_p2, v1), cross) / denominator;
        return (t1 >= 0f) && (t2 >= 0f);
    }

    public enum MovementMode
    {
        Car,
        Directional
    }

}
public class EdgePoints
{
    public int Start = -1;
    public int End = -1;
    public int Other = -1;

    public void Set(int start, int end, int other)
    {
        Start = start;
        End = end;
        Other = other;
    }

}

public static class EdgePointsExtensions
{
    public static void SetEdges(this EdgePoints edgePoints, int start, int end, int other)
    {
        edgePoints.Start = start;
        edgePoints.End = end;
        edgePoints.Other = other;
    }
}