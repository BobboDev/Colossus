using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Overhang;
using KinematicCharacterController;
using VinforlabTeam.FurClimbingSystem;
using System.Linq;

public class ClimbShape : MonoBehaviour
{
    // 🧱 CORE COMPONENTS

    [Header("Core Components")]
    public Transform CharacterModel;
    public Collider capsule;
    public MovementMode movementMode;

    Rigidbody rb;
    ClimbableMesh _cm;

    // 🧗 CLIMBING STATE & FLAGS

    [Header("Climbing State")]
    public bool isClimbing;
    public bool onEdge;
    public bool forceSlide;
    public bool forceSlideForwardProjection;

    bool firstMoveDone;
    bool cutFound;
    bool goForwardTest = false;
    int indexOfCornerReached;


    // 🚀 MOVEMENT & INPUT

    [Header("Movement & Input")]
    [Min(0)]
    public float DirectionalSpeed;
    [Min(0)]
    public float ClimbingSpeed;

    Vector3 input;
    Vector3 moveDirection;
    Vector3 newPosition;
    Vector3 lastForward;
    float targetSpeed;

    // ⏱️ TIMING

    [Header("Timing")]
    float deltaTime;
    float frameRate;


    // 🧩 TRIANGLE INDEXING

    [Header("Triangle Indexing")]
    int currentTriangleIndex;
    int lastTriangleIndex;
    int previousIndex;
    int previousLastIndex;


    // 🔺 BARYCENTRIC COORDINATES

    [Header("Barycentric Coordinates")]
    Vector3 barycentricCoordinate;
    Vector3 lastBarycentricCoordinate;
    Vector3 barycentricCoordinateBehind;


    // 📏 EDGE TRACKING

    [Header("Edge Tracking")]
    EdgePoints currentEdgePoints = new();
    EdgePoints lastEdge = new();


    // 🛠️ PLANE & CUT LOGIC

    [Header("Plane & Cut Logic")]
    Plane plane;
    Plane testPlane;
    Vector3 cut;
    Vector3 testCut;


    // 🎯 RAYCASTING & DETECTION

    [Header("Raycasting")]
    public LayerMask layerMask;
    public LayerMask layerMaskForwardProjection;

    Vector3 previousRayCastOrigin;
    Vector3 castDirectionTest;

    // 📌 POSITION CHECKING / TRACING

    [Header("Position Checking")]
    Vector3 currentCheckPosition;
    Vector3 checkPositionStart;
    Vector3 checkPositionEnd;
    Quaternion afterDepenetrateRotation;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        // This is used to ensure accuracy with collisions/depenetration, but it has a performance cost. Better to do it manually in the future.
        Physics.autoSyncTransforms = true;
        UnityEngine.Cursor.visible = false;
        UnityEngine.Cursor.lockState = CursorLockMode.Locked;

        // Don’t limit framerate
        Application.targetFrameRate = 300;
        // set the plane used for pathfinding to be oriented to the character
        plane = new Plane(-transform.right, transform.position);
    }

    void LateUpdate()
    {
        // Calculate the time it took to render the last frame
        deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;

        // Calculate the frame rate in frames per second
        frameRate = 1.0f / deltaTime;

        // if not on a mesh, raycast to find a mesh
        if (!isClimbing)
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

            _cm.RecalculateMesh(false);

            // Get input
            Vector3 tempInput;
            tempInput = new Vector3(Input.GetAxisRaw("Horizontal"), 0, Input.GetAxisRaw("Vertical")).normalized;

            // This is for debugging - makes the character go forwards and backwards over an edge rapidly, so I don't have to hammer the keyboard to test. 
            if (Input.GetKeyDown(KeyCode.K))
            {
                goForwardTest = !goForwardTest;
            }
            if (goForwardTest)
            {
                tempInput = new Vector3(0, 0, 1);
            }

            // if input has magnitude then use it
            if (tempInput.magnitude > 0)
            {
                input = tempInput;
            }

            // calculate movement on the mesh based on input
            Travel(tempInput, false, false);
            Vector3 pointOnTriangle = _cm.GetPositionFromBarycentric(barycentricCoordinate, currentEdgePoints.Start, currentEdgePoints.End, currentEdgePoints.Other);
            transform.position = pointOnTriangle;

            int depenetrationIterations = 1;

            for (int i = 0; i < depenetrationIterations; i++)
            {
                bool isFinalPass = i == depenetrationIterations - 1;
                Depenetrate(isFinalPass);
                Physics.SyncTransforms();
            }

            // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
            pointOnTriangle = _cm.GetPositionFromBarycentric(barycentricCoordinate, currentEdgePoints.Start, currentEdgePoints.End, currentEdgePoints.Other);
            transform.position = pointOnTriangle;
        }
    }

    void Depenetrate(bool isFinalPass)
    {
        Travel(Vector3.zero, true, isFinalPass);
    }

    void SetEdges(Vector3 previousLastEdgeStartPosition, Vector3 previousLastEdgeEndPosition, Vector3 previousLastEdgeOtherPosition, int index)
    {
        currentEdgePoints.Start = _cm.Triangles
                            .Skip(index)
                            .Take(3)
                            .FirstOrDefault(tri => _cm.Vertices[tri] == previousLastEdgeStartPosition);

        currentEdgePoints.End = _cm.Triangles
                            .Skip(index)
                            .Take(3)
                            .FirstOrDefault(tri => _cm.Vertices[tri] == previousLastEdgeEndPosition);

        currentEdgePoints.Other = _cm.Triangles
                            .Skip(index)
                            .Take(3)
                            .FirstOrDefault(tri => _cm.Vertices[tri] == previousLastEdgeOtherPosition);
    }

    void Travel(Vector3 direction, bool depenetratePass, bool isFinalPass)
    {
        // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
        Vector3 pointOnTriangle = _cm.GetPositionFromBarycentric(barycentricCoordinate, currentEdgePoints.Start, currentEdgePoints.End, currentEdgePoints.Other);
        // At the end of last loop we do a 'test' cut to get the next position BEHIND
        // This is recorded from the tri center
        Vector3 behindPointOnTriangle = _cm.GetPositionFromBarycentric(barycentricCoordinateBehind, currentEdgePoints.Start, currentEdgePoints.End, currentEdgePoints.Other);
        // Quaternion rotationLast = transform.rotation;
        // At the end of last loop we do a 'test' cut to get the next position in FRONT
        // Here we recalculate it with deformations. This is the forward cut, NOT the movement direction cut 
        // This is also recorded from the tri center
        Vector3 forwardPointOnTriangle = _cm.GetPositionFromBarycentric(lastBarycentricCoordinate, currentEdgePoints.Start, currentEdgePoints.End, currentEdgePoints.Other);

        transform.position = pointOnTriangle;

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        Vector3 tempForward = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        if (Input.GetKey(KeyCode.LeftShift))
        {
            if (movementMode != MovementMode.Car)
            {
                Vector3 previousLastEdgeStartPosition = _cm.Vertices[currentEdgePoints.Start];
                Vector3 previousLastEdgeEndPosition = _cm.Vertices[currentEdgePoints.End];
                Vector3 previousLastEdgeOtherPosition = _cm.Vertices[currentEdgePoints.Other];

                int indexTemp = _cm.GetArea(currentTriangleIndex / 3) * 3;

                if (indexTemp != -3)
                {
                    currentTriangleIndex = indexTemp;
                    SetEdges(previousLastEdgeStartPosition, previousLastEdgeEndPosition, previousLastEdgeOtherPosition, currentTriangleIndex);

                    Vector3 triCenterTemp = (_cm.Vertices[currentEdgePoints.Start] + _cm.Vertices[currentEdgePoints.End] + _cm.Vertices[currentEdgePoints.Other]) / 3;

                    Plane plane = new Plane(-transform.right, triCenterTemp);

                    barycentricCoordinate = Mathf2.GetBarycentricCoordinates(newPosition, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);

                    testCut = GetNextCut(currentEdgePoints, transform.forward, triCenterTemp, plane, CutType.Start);
                    lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);
                    testCut = GetNextCut(currentEdgePoints, -transform.forward, triCenterTemp, plane, CutType.Test);
                    barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);

                    movementMode = MovementMode.Car;
                }
            }
        }
        else
        {
            if (movementMode != MovementMode.Directional)
            {
                Vector3 previousLastEdgeStartPosition = _cm.Vertices[currentEdgePoints.Start];
                Vector3 previousLastEdgeEndPosition = _cm.Vertices[currentEdgePoints.End];
                Vector3 previousLastEdgeOtherPosition = _cm.Vertices[currentEdgePoints.Other];

                int indexTemp = _cm.GetMainBody(currentTriangleIndex / 3) * 3;

                _cm.RecalculateMesh(false);

                if (indexTemp != -3)
                {
                    currentTriangleIndex = indexTemp;
                    onEdge = false;

                    SetEdges(previousLastEdgeStartPosition, previousLastEdgeEndPosition, previousLastEdgeOtherPosition, currentTriangleIndex);
                    Vector3 triCenterTemp = (_cm.Vertices[currentEdgePoints.Start] + _cm.Vertices[currentEdgePoints.End] + _cm.Vertices[currentEdgePoints.Other]) / 3;

                    Plane plane = new Plane(-transform.right, triCenterTemp);

                    testCut = GetNextCut(currentEdgePoints, transform.forward, triCenterTemp, plane, CutType.Start);
                    lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);
                    testCut = GetNextCut(currentEdgePoints, -transform.forward, triCenterTemp, plane, CutType.Test);
                    barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);

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
        if (!_cm.TriangleInfos.ContainsKey(currentTriangleIndex))
        {
            Debug.Log("Index: " + currentTriangleIndex);
            foreach (var item in _cm.TriangleInfos)
            {
                Debug.Log(item.Key);
#if UNITY_EDITOR
                EditorApplication.isPlaying = false;
#endif
            }
        }

        foreach (Edge e in _cm.TriangleInfos[currentTriangleIndex].edges)
        {
            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.Start])
                lastEdgeStartReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.Start])
                lastEdgeStartReal = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.End])
                lastEdgeEndReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.End])
                lastEdgeEndReal = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.Other])
                lastEdgeOtherReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.Other])
                lastEdgeOtherReal = e.pointB;
        }

        // Calculate the ground normal based on coordinates from the last frame of where we should be standing, translated to the new, deformed triangle
        Vector3 groundNormal = _cm.GetNormalFromBarycentric(barycentricCoordinate, lastEdgeStartReal, lastEdgeEndReal, lastEdgeOtherReal);

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
        Vector3 triCenter = (_cm.Vertices[currentEdgePoints.Start] + _cm.Vertices[currentEdgePoints.End] + _cm.Vertices[currentEdgePoints.Other]) / 3;
        Vector3 closestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[currentEdgePoints.Start], (_cm.Vertices[currentEdgePoints.Start] - _cm.Vertices[currentEdgePoints.End]).normalized, triCenter);
        Vector3 getNewtempForwardDirection = Vector3.zero;
        Vector3 getNewtempForwardDirectionBehind = Vector3.zero;

        if (movementMode == MovementMode.Directional)
        {
            // This check works as intended
            if (direction.magnitude > 0)
            {
                float angleToRotateby = Camera.main.transform.rotation.eulerAngles.y - transform.rotation.eulerAngles.y;
                input = Quaternion.Euler(0, angleToRotateby, 0) * input;

                if (!depenetratePass)
                {
                    afterDepenetrateRotation = Quaternion.LookRotation(Quaternion.Euler(0, transform.rotation.eulerAngles.y, 0) * input);
                }
                if (isFinalPass)
                {
                    CharacterModel.rotation = afterDepenetrateRotation;
                }
                Plane NewtempForwardPlane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterModel.right, transform.position);
                Vector3 PlaneNormalRight = Mathf2.RotateAroundAxis(NewtempForwardPlane.normal, groundNormal, 90);
                getNewtempForwardDirection = GetNextCut(currentEdgePoints, PlaneNormalRight, transform.position, NewtempForwardPlane, CutType.Test);
                getNewtempForwardDirectionBehind = GetNextCut(currentEdgePoints, -PlaneNormalRight, transform.position, NewtempForwardPlane, CutType.Test);

                getNewtempForwardDirection = (getNewtempForwardDirection - getNewtempForwardDirectionBehind).normalized;
                tempForward = getNewtempForwardDirection;
            }
            else
            {
                if (!depenetratePass)
                {
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

        // newDirection - can't remember what this does but it's possible it will change in the loop but we want to keep moveDirection the same.
        Vector3 newDirection = moveDirection;

        //calculate distance to move

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
                        Debug.DrawLine(transform.position, transform.position + totalDepenetrationDirection, Color.red);
                    }
                }

                Plane wallPlane = new();
                Vector3 wallNormal = depenetrationDirection.normalized;
                wallPlane.SetNormalAndPosition(wallNormal, depenetrationDirection * depenetrationDistance);

                Ray depenetrateRay = new Ray();
                depenetrateRay.direction = totalDepenetrationDirection.normalized;
                depenetrateRay.origin = Vector3.zero;
                wallPlane.Raycast(depenetrateRay, out distance);
                distance *= 1f;

                tempForward = totalDepenetrationDirection.normalized;

                moveDirection = tempForward;
                newDirection = tempForward;
                input = tempForward;
                depenetrate = true;
                plane = new Plane(Mathf2.RotateAroundAxis(tempForward, transform.up, 90), transform.position);
            }
            else
            {
                tempForward = totalDepenetrationDirection.normalized;

                moveDirection = tempForward;
                newDirection = tempForward;
                input = tempForward;
                plane = new Plane(Mathf2.RotateAroundAxis(tempForward, transform.up, 90), transform.position);
            }
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

        // total distance checked this frame - if goes over, loop will break
        float totalDistanceChecked = 0;

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
            cut = GetNextCut(currentEdgePoints, newDirection, transform.position, plane, CutType.Start);
        }
        else // else do a test cut - test cut considers all edges and returns a "cut" but doesn't change the current edge/index information. this is good because we want to be able to get an edge we are standing directly on.
        {
            cut = GetNextCut(currentEdgePoints, newDirection, transform.position, plane, CutType.Test);
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
        }
        else // else make the next check go from the point on the next edge
        {
            newPosition = cut;
        }

        float remainingDistance = distance - Vector3.Distance(transform.position, newPosition);
        castDirectionTest = transform.forward;

        // Why do we need lastIndex, previousLastIndex and previousIndex?

        previousLastIndex = currentTriangleIndex;
        lastEdge.SetEdges(currentEdgePoints.Start, currentEdgePoints.End, currentEdgePoints.Other);

        // while we have checked less distance than the edge
        while (totalDistanceChecked < distance)
        {
            // if not on edge / just started moving away from edge
            if (!onEdge)
            {
                // set the last index to be the index - this is because we're about to check the next tri to see if we're on an edge, and we want to compare index to lastIndex
                lastTriangleIndex = currentTriangleIndex;

                // take the edge we just passed, then get the tri that we didn't just check
                // don't worry, index is set inGetNextTri() 
                EdgePoints nextTri = _cm.GetNextTri(ref currentTriangleIndex, currentEdgePoints.Start, currentEdgePoints.End);
                // if GetNextTri() returns same value as index, we are on an edge, else do nothing
                if (currentTriangleIndex == lastTriangleIndex)
                {
                    onEdge = true;
                    Debug.Log(onEdge);
                }
                else
                {
                    cutFound = false;
                    Vector3 lastCut = cut;

                    cut = GetNextCut(nextTri, newDirection, newPosition, plane, CutType.NextTriangle);

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
                        currentEdgePoints.Start = lastEdge.Start;
                        currentEdgePoints.End = lastEdge.End;
                        currentEdgePoints.Other = lastEdge.Other;
                        currentTriangleIndex = previousLastIndex;
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
                Vector3 slidePoint = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End]);
                int currentCornerInt = -1;
                int currentCornerIntOther = 0;
                if (slidePoint == _cm.Vertices[currentEdgePoints.Start])
                {
                    currentCornerInt = currentEdgePoints.Start;
                    currentCornerIntOther = currentEdgePoints.End;
                }
                if (slidePoint == _cm.Vertices[currentEdgePoints.End])
                {
                    currentCornerInt = currentEdgePoints.End;
                    currentCornerIntOther = currentEdgePoints.Start;
                }

                // if we reached a corner
                if (currentCornerInt != -1)
                {
                    // Get the next edge. Since lastEdge.Start and lastEdge.End are always the ouside edge when edge-sliding, the next edge is the corner reached -> lastEdgeOther
                    Edge nextEdgeOnThisTriangle = new Edge();
                    if (currentCornerInt < currentEdgePoints.Other)
                    {
                        nextEdgeOnThisTriangle.pointA = currentCornerInt;
                        nextEdgeOnThisTriangle.pointB = currentEdgePoints.Other;
                    }
                    else
                    {
                        nextEdgeOnThisTriangle.pointA = currentEdgePoints.Other;
                        nextEdgeOnThisTriangle.pointB = currentCornerInt;
                    }
                    // If the next edge is another outside edge on the same triangle, switch to that. 
                    if (_cm.edgeInfos.ContainsKey(nextEdgeOnThisTriangle) && nextEdgeOnThisTriangle.EdgeIsOutsideEdge())
                    {
                        currentEdgePoints.SetEdges(nextEdgeOnThisTriangle.pointA, currentCornerIntOther, nextEdgeOnThisTriangle.pointB);

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

                        if (slidePoint == _cm.Vertices[currentEdgePoints.Start])
                        {
                            indexOfCornerReached = currentEdgePoints.Start;
                        }
                        else if (slidePoint == _cm.Vertices[currentEdgePoints.End])
                        {
                            indexOfCornerReached = currentEdgePoints.End;
                        }

                        int timesLooped = 0;
                        int count = 0;

                        GetEdgeOnSpecificTriangle(currentTriangleIndex, ref lastEdgeStartReal, ref lastEdgeEndReal);




                        ////////////////////////
                        //                    //
                        //   START Refactor   //
                        //   START Refactor   //
                        //   START Refactor   //
                        //   START Refactor   //
                        //   START Refactor   //
                        //   START Refactor   //
                        //   START Refactor   //
                        //   START Refactor   //
                        //   START Refactor   //
                        //   START Refactor   //
                        //                    //
                        ////////////////////////


                        List<Edge> previouslyCheckedEdges = new List<Edge>();
                        Edge lastEdgePassedReal = _cm.GetEdgeFromVertexIndices(lastEdgeStartReal, lastEdgeEndReal);
                        Edge lastEdgePassed = _cm.GetEdgeFromVertexIndices(currentEdgePoints.Start, currentEdgePoints.End);

                        // WARNING - changed from using lastEdge.Start and lastEdge.End to lastEdgeStartRealTemp and lastEdgeEndRealTemp. Could cause issues.

                        // If on outside
                        if (lastEdgePassed.EdgeIsOutsideEdge())
                        {
                            foreach (Edge e in _cm.EdgesAttachedToCorner[indexOfCornerReached])
                            {
                                if (!e.IsIdenticalToByPosition(lastEdgePassedReal))
                                {
                                    previouslyCheckedEdges.Add(e);
                                    count++;
                                }
                            }
                        }
                        else
                        {

                            Edge cornerWall;
                            int currentCheckingTriangleInCorner = currentTriangleIndex;

                            cornerWall = _cm.GetOppositeWallOnCorner(indexOfCornerReached, currentTriangleIndex, lastEdgePassed);
                            int cornerEdgeOther2 = -1;

                            AdjacentEdges cornerWallAdjacentEdges = _cm.TriangleInfos[_cm.edgeInfos[cornerWall].triangleA];

                            // Get the vertex of the wall triangle that's not on the wall

                            cornerEdgeOther2 = _cm.FindVertexNotOnEdge(cornerWallAdjacentEdges, lastEdgePassed);

                            timesLooped = 0;

                            foreach (Edge e in _cm.EdgesAttachedToCorner[indexOfCornerReached])
                            {
                                previouslyCheckedEdges.Add(e);
                                count++;
                            }

                            if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[cornerWall.pointA], _cm.Vertices[cornerWall.pointB]) == _cm.Vertices[indexOfCornerReached])
                            {
                                previouslyCheckedEdges.Remove(cornerWall);
                                currentTriangleIndex = _cm.edgeInfos[cornerWall].triangleA;

                                if (_cm.Vertices[cornerWall.pointA] == _cm.Vertices[indexOfCornerReached])
                                {
                                    currentEdgePoints.End = cornerWall.pointA;
                                    currentEdgePoints.Start = cornerWall.pointB;
                                }
                                else if (_cm.Vertices[cornerWall.pointB] == _cm.Vertices[indexOfCornerReached])
                                {
                                    currentEdgePoints.End = cornerWall.pointB;
                                    currentEdgePoints.Start = cornerWall.pointA;
                                }

                                currentEdgePoints.Other = cornerEdgeOther2;
                            }
                        }



                        //////////////////////
                        //                  //
                        //   END Refactor   //
                        //   END Refactor   //
                        //   END Refactor   //
                        //   END Refactor   //
                        //   END Refactor   //
                        //   END Refactor   //
                        //   END Refactor   //
                        //   END Refactor   //
                        //   END Refactor   //
                        //   END Refactor   //
                        //                  //
                        //////////////////////



                        bool foundNextEdge = false;
                        int x = 0;
                        int tempIndex = currentTriangleIndex;
                        int tempLastIndex = lastTriangleIndex;

                        List<Edge> checkedEdges = new List<Edge>();
                        List<Edge> checkedEdgesCorner = new List<Edge>();
                        // if we haven't found an outer edge

                        int cornerEdgeStart = -1;
                        int cornerEdgeEnd = -1;
                        int cornerEdgeOther = -1;
                        int cornerTriangleIndex = tempIndex;
                        while (cornerEdgeStart == -1)
                        {
                            foreach (Edge e in _cm.TriangleInfos[cornerTriangleIndex].edges)
                            {
                                // We need to find the other corner edge
                                bool tempEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in previouslyCheckedEdges)
                                {
                                    if (_cm.Vertices[e.pointA] == _cm.Vertices[edgeToCheck.pointA] && _cm.Vertices[e.pointB] == _cm.Vertices[edgeToCheck.pointB]
                                     || _cm.Vertices[e.pointA] == _cm.Vertices[edgeToCheck.pointB] && _cm.Vertices[e.pointB] == _cm.Vertices[edgeToCheck.pointA])
                                    {
                                        tempEdgesContainsEdge = true;
                                    }
                                }
                                bool checkedEdgesCornerContainsEdge = false;
                                foreach (Edge edgeToCheck in checkedEdgesCorner)
                                {
                                    if (_cm.Vertices[e.pointA] == _cm.Vertices[edgeToCheck.pointA] && _cm.Vertices[e.pointB] == _cm.Vertices[edgeToCheck.pointB]
                                     || _cm.Vertices[e.pointA] == _cm.Vertices[edgeToCheck.pointB] && _cm.Vertices[e.pointB] == _cm.Vertices[edgeToCheck.pointA])
                                    {
                                        checkedEdgesCornerContainsEdge = true;
                                    }
                                }
                                // if the edge is one of the edges attached to the current corner
                                if (tempEdgesContainsEdge && !checkedEdgesCornerContainsEdge)
                                {
                                    Debug.Log("A");
                                    // and it's an outside edge
                                    if (_cm.edgeInfos[e].triangleA == _cm.edgeInfos[e].triangleB)
                                    {
                                        Debug.Log("A1");
                                        if (_cm.Vertices[e.pointA] == _cm.Vertices[indexOfCornerReached])
                                        {
                                            Debug.Log("A1_1");
                                            cornerEdgeStart = e.pointA;
                                            cornerEdgeEnd = e.pointB;

                                        }
                                        else if (_cm.Vertices[e.pointB] == _cm.Vertices[indexOfCornerReached])
                                        {
                                            Debug.Log("A1_2");
                                            cornerEdgeStart = e.pointB;
                                            cornerEdgeEnd = e.pointA;
                                        }
                                        foreach (Edge e2 in _cm.TriangleInfos[cornerTriangleIndex].edges)
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
                                        if (cornerTriangleIndex == _cm.edgeInfos[e].triangleA)
                                        {
                                            cornerTriangleIndex = _cm.edgeInfos[e].triangleB;
                                        }
                                        else
                                        {
                                            cornerTriangleIndex = _cm.edgeInfos[e].triangleA;
                                        }
                                        foreach (Edge e2 in _cm.TriangleInfos[cornerTriangleIndex].edges)
                                        {
                                            if (_cm.Vertices[e2.pointA] == _cm.Vertices[e.pointA] && _cm.Vertices[e2.pointB] == _cm.Vertices[e.pointB]
                                            || _cm.Vertices[e2.pointA] == _cm.Vertices[e.pointA] && _cm.Vertices[e2.pointB] == _cm.Vertices[e.pointB])
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
                                break;
                            }
                        }
                        while (!foundNextEdge)
                        {
                            // we need to use a different method for contains - checking by their positions, since hard edges has different indices
                            // scan the edges of the current triangle
                            foreach (Edge e in _cm.TriangleInfos[tempIndex].edges)
                            {
                                // we need to use a different method for contains - checking by their positions, since hard edges has different indices
                                bool tempEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in previouslyCheckedEdges)
                                {
                                    if (_cm.Vertices[e.pointA] == _cm.Vertices[edgeToCheck.pointA] && _cm.Vertices[e.pointB] == _cm.Vertices[edgeToCheck.pointB]
                                     || _cm.Vertices[e.pointA] == _cm.Vertices[edgeToCheck.pointB] && _cm.Vertices[e.pointB] == _cm.Vertices[edgeToCheck.pointA])
                                    {
                                        tempEdgesContainsEdge = true;
                                    }
                                }
                                bool checkedEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in checkedEdges)
                                {
                                    if (_cm.Vertices[e.pointA] == _cm.Vertices[edgeToCheck.pointA] && _cm.Vertices[e.pointB] == _cm.Vertices[edgeToCheck.pointB]
                                     || _cm.Vertices[e.pointA] == _cm.Vertices[edgeToCheck.pointB] && _cm.Vertices[e.pointB] == _cm.Vertices[edgeToCheck.pointA])
                                    {
                                        checkedEdgesContainsEdge = true;
                                    }
                                }
                                // if the edge is one of the edges attached to the current corner
                                if (tempEdgesContainsEdge && !checkedEdgesContainsEdge)
                                {
                                    // and it's an outside edge
                                    if (_cm.edgeInfos[e].triangleA == _cm.edgeInfos[e].triangleB)
                                    {
                                        foundNextEdge = true;
                                        previousLastIndex = lastTriangleIndex;
                                        previousIndex = currentTriangleIndex;

                                        lastEdge.Start = currentEdgePoints.Start;
                                        lastEdge.End = currentEdgePoints.End;
                                        lastEdge.Other = currentEdgePoints.Other;

                                        currentEdgePoints.Start = e.pointA;
                                        currentEdgePoints.End = e.pointB;

                                        lastTriangleIndex = tempLastIndex;
                                        currentTriangleIndex = tempIndex;

                                        foreach (Edge p in _cm.TriangleInfos[currentTriangleIndex].edges)
                                        {
                                            if (_cm.Vertices[p.pointA] != _cm.Vertices[currentEdgePoints.Start] && _cm.Vertices[p.pointA] != _cm.Vertices[currentEdgePoints.End])
                                            {
                                                currentEdgePoints.Other = p.pointA;
                                            }
                                            else if (_cm.Vertices[p.pointB] != _cm.Vertices[currentEdgePoints.Start] && _cm.Vertices[p.pointB] != _cm.Vertices[currentEdgePoints.End])
                                            {
                                                currentEdgePoints.Other = p.pointB;
                                            }
                                        }

                                        triCenter = (_cm.Vertices[currentEdgePoints.Start] + _cm.Vertices[currentEdgePoints.End] + _cm.Vertices[currentEdgePoints.Other]) / 3;
                                        closestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[currentEdgePoints.Start], (_cm.Vertices[currentEdgePoints.Start] - _cm.Vertices[currentEdgePoints.End]).normalized, triCenter);

                                        Vector3 previousTriCenter = (_cm.Vertices[lastEdge.Start] + _cm.Vertices[lastEdge.End] + _cm.Vertices[lastEdge.Other]) / 3;
                                        Vector3 previousClosestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[lastEdge.Start], (_cm.Vertices[lastEdge.Start] - _cm.Vertices[lastEdge.End]).normalized, previousTriCenter);

                                        newPosition = slidePoint;

                                        Vector3 cornerNormal = Vector3.Cross(_cm.Vertices[lastEdge.Start] - _cm.Vertices[lastEdge.End], _cm.Vertices[currentEdgePoints.Start] - _cm.Vertices[currentEdgePoints.End]).normalized;
                                        if (Vector3.Dot(cornerNormal, transform.up) < 0)
                                        {
                                            cornerNormal = -cornerNormal;
                                        }
                                        Vector3 newEdgeNormal = (closestPointOnEdge - triCenter).normalized;
                                        Vector3 lastEdgeNormal = (previousClosestPointOnEdge - previousTriCenter).normalized;

                                        Vector3 newFarCorner = _cm.Vertices[currentCornerInt] == _cm.Vertices[currentEdgePoints.Start] ? _cm.Vertices[currentEdgePoints.End] : _cm.Vertices[currentEdgePoints.Start];
                                        Vector3 lastFarCorner = _cm.Vertices[currentCornerInt] == _cm.Vertices[lastEdge.Start] ? _cm.Vertices[lastEdge.End] : _cm.Vertices[lastEdge.Start];

                                        int shouldInvertNewEdgeNormal = 1;
                                        Vector3 cornerDirection = Vector3.ProjectOnPlane(transform.rotation * input, cornerNormal).normalized;

                                        Vector3 newEdgeNormal2 = Vector3.ProjectOnPlane(newEdgeNormal * shouldInvertNewEdgeNormal, cornerNormal).normalized;
                                        Vector3 lastEdgeNormal2 = Vector3.ProjectOnPlane(lastEdgeNormal, cornerNormal).normalized;

                                        if (Vector3.Dot((transform.rotation * input).normalized, (closestPointOnEdge - triCenter).normalized) < 0)
                                        {
                                            onEdge = false;
                                            foundNextEdge = true;
                                            Debug.Log("Found Edge");
                                        }

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
                                            VectorUtils.DoRaysIntersect(ray1, ray2) &&
                                            IsTriAfterNextThis()
                                        )
                                        {
                                            // NOT SWITCHING TO OTHER EDGE SO OF COURSE SLIDEPOINT IS STILL AT THE CORNER
                                            onEdge = true;
                                            currentEdgePoints.Start = lastEdge.Start;
                                            currentEdgePoints.End = lastEdge.End;
                                            currentEdgePoints.Other = lastEdge.Other;

                                            currentTriangleIndex = previousIndex;

                                            lastTriangleIndex = previousLastIndex;
                                            totalDistanceChecked = distance;
                                            newPosition = slidePoint;
                                            slidePoint = _cm.Vertices[indexOfCornerReached];
                                            foundNextEdge = true;
                                            Debug.Log("Found Edge");
                                        }

                                    }
                                    else
                                    {
                                        EdgePoints nextTriLastEdge = new();
                                        // else switch to the tri on the other side of the edge
                                        tempLastIndex = tempIndex;
                                        if (tempIndex == _cm.edgeInfos[e].triangleA)
                                        {
                                            tempIndex = _cm.edgeInfos[e].triangleB;
                                        }
                                        else
                                        {
                                            tempIndex = _cm.edgeInfos[e].triangleA;
                                        }
                                        checkedEdges.Add(e);
                                        if (_cm.Vertices[e.pointA] == _cm.Vertices[indexOfCornerReached])
                                        {
                                            nextTriLastEdge.Start = e.pointA;
                                            nextTriLastEdge.End = e.pointB;
                                        }
                                        else
                                        {
                                            nextTriLastEdge.Start = e.pointB;
                                            nextTriLastEdge.End = e.pointA;
                                        }
                                        foreach (Edge e2 in _cm.TriangleInfos[tempIndex].edges)
                                        {
                                            if (_cm.Vertices[e2.pointA] != _cm.Vertices[nextTriLastEdge.Start] && _cm.Vertices[e2.pointA] != _cm.Vertices[nextTriLastEdge.End])
                                            {
                                                nextTriLastEdge.Other = e2.pointA;
                                            }
                                            if (_cm.Vertices[e2.pointB] != _cm.Vertices[nextTriLastEdge.Start] && _cm.Vertices[e2.pointB] != _cm.Vertices[nextTriLastEdge.End])
                                            {
                                                nextTriLastEdge.Other = e2.pointB;
                                            }
                                        }

                                        // check if we are pointing towards the other edge (not the next edge attached to corner)
                                        // if so, we can come unstuck.

                                        if (GetFarEdgeCut(indexOfCornerReached, tempIndex, nextTriLastEdge.Start, nextTriLastEdge.End, nextTriLastEdge.Other, cornerEdgeStart, cornerEdgeEnd, cornerEdgeOther, plane))
                                        {
                                            currentTriangleIndex = tempIndex;
                                            lastTriangleIndex = tempLastIndex;
                                            currentEdgePoints.SetEdges(nextTriLastEdge.End, nextTriLastEdge.Start, nextTriLastEdge.Other);

                                            onEdge = false;

                                            foundNextEdge = true;
                                            Debug.Log("Found Edge");

                                            totalDistanceChecked = distance;
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

                                return;
                            }
                        }
                    }
                    newPosition = slidePoint;
                }
                else
                {
                    totalDistanceChecked = distance;
                    newPosition = slidePoint;
                }

                remainingDistance = remainingDistance - Vector3.Distance(newPosition, slidePoint);
                plane = new Plane(transform.rotation * Quaternion.Euler(0, 90, 0) * input, newPosition);

                i++;
                if (i > 1000)
                {
                    Debug.Log("WHOOPS " + currentTriangleIndex);
#if UNITY_EDITOR
                    EditorApplication.isPaused = true;
#endif
                    break;
                }
            }
        }

        barycentricCoordinate = Mathf2.GetBarycentricCoordinates(newPosition, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);

        triCenter = (_cm.Vertices[currentEdgePoints.Start] + _cm.Vertices[currentEdgePoints.End] + _cm.Vertices[currentEdgePoints.Other]) / 3;

        foreach (Edge e in _cm.TriangleInfos[currentTriangleIndex].edges)
        {
            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.Start])
                lastEdgeStartReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.Start])
                lastEdgeStartReal = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.End])
                lastEdgeEndReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.End])
                lastEdgeEndReal = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.Other])
                lastEdgeOtherReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.Other])
                lastEdgeOtherReal = e.pointB;
        }

        groundNormal = _cm.GetNormalFromBarycentric(barycentricCoordinate, lastEdgeStartReal, lastEdgeEndReal, lastEdgeOtherReal);

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
        }
        testCut = Vector3.zero;
        testCut = GetNextCut(currentEdgePoints, castDirectionTest, triCenter, testPlane, CutType.Test);

        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);
        testCut = GetNextCut(currentEdgePoints, -castDirectionTest, triCenter, testPlane, CutType.Test);
        barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);


        if (getNewtempForwardDirection != Vector3.zero)
        {
            lastForward = getNewtempForwardDirection;
        }
        else
        {
            lastForward = CharacterModel.forward;
        }
    }

    bool IsTriAfterNextThis()
    {
        int tempLastEdgeStartReal = -1;
        int tempLastEdgeEndReal = -1;
        int tempLastEdgeOtherReal = -1;

        Vector3 tempBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(newPosition, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);

        Vector3 tempTriCenter = (_cm.Vertices[currentEdgePoints.Start] + _cm.Vertices[currentEdgePoints.End] + _cm.Vertices[currentEdgePoints.Other]) / 3;

        foreach (Edge e in _cm.TriangleInfos[currentTriangleIndex].edges)
        {
            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.Start])
                tempLastEdgeStartReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.Start])
                tempLastEdgeStartReal = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.End])
                tempLastEdgeEndReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.End])
                tempLastEdgeEndReal = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[currentEdgePoints.Other])
                tempLastEdgeOtherReal = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[currentEdgePoints.Other])
                tempLastEdgeOtherReal = e.pointB;
        }

        Vector3 tempGroundNormal = _cm.GetNormalFromBarycentric(tempBarycentricCoordinate, tempLastEdgeStartReal, tempLastEdgeEndReal, tempLastEdgeOtherReal);

        Vector3 tempCastDirectionTest = Quaternion.FromToRotation(transform.up, tempGroundNormal) * transform.forward;

        Plane tempTestPlane = new Plane(-(Quaternion.FromToRotation(transform.up, tempGroundNormal) * transform.right), tempTriCenter);

        Vector3 tempTestCut = GetNextCut(currentEdgePoints, tempCastDirectionTest, tempTriCenter, tempTestPlane, CutType.Test);

        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        Vector3 tempLastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(tempTestCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);
        tempTestCut = GetNextCut(currentEdgePoints, -tempCastDirectionTest, tempTriCenter, tempTestPlane, CutType.Test);
        Vector3 tempBarycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(tempTestCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);
        Vector3 behindPointOnTriangle = _cm.GetPositionFromBarycentric(tempBarycentricCoordinateBehind, currentEdgePoints.Start, currentEdgePoints.End, currentEdgePoints.Other);
        // At the end of last loop we do a 'test' cut to get the next position in FRONT
        // Here we recalculate it with deformations. This is the forward cut, NOT the movement direction cut 
        // This is also recorded from the tri center
        Vector3 forwardPointOnTriangle = _cm.GetPositionFromBarycentric(tempLastBarycentricCoordinate, currentEdgePoints.Start, currentEdgePoints.End, currentEdgePoints.Other);

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        Vector3 tempForward = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        Quaternion tempRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(tempForward, tempGroundNormal), tempGroundNormal); // set rotation to be towards the forward point.

        Vector3 tempMovePositionAttempt = _cm.Vertices[indexOfCornerReached] + (tempRotation * input).normalized;

        Vector3 slidePoint3 = Mathf2.GetClosestPointOnFiniteLine(tempMovePositionAttempt, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End]);
        // This is for stopping the check from bouncing between edges when stuck in a corner
        // Problem is that it's sticking to corners when the GetClosestPointOnLine attemptedMovePosition is outside the line

        // In order to determine if we should be able to move round the corner, we need to know if it's a convex corner or not
        // DoRaysIntersect of the edge normals determines this.
        // If they do, then we check if the closest point on each edge to the attempted move position is equal to the corner - that's the only situation where we should move round the corner  

        if (slidePoint3 == _cm.Vertices[indexOfCornerReached])
        {
            Debug.Log("tri repeat");
            return true;
        }
        else
        {
            Debug.Log("no tri repeat");
            return false;
        }
    }

    bool GetFarEdgeCut(int currentCornerInt, int triangleIndex, int nextTriLastEdgeStart, int nextTriLastEdgeEnd, int nextTriLastEdgeOther, int cornerEdgeStart, int cornerEdgeEnd, int cornerEdgeOther, Plane plane)
    {
        Vector3 triCenter = (_cm.Vertices[cornerEdgeStart] + _cm.Vertices[cornerEdgeEnd] + _cm.Vertices[cornerEdgeOther]) / 3;
        Vector3 closestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[cornerEdgeStart], (_cm.Vertices[cornerEdgeStart] - _cm.Vertices[cornerEdgeEnd]).normalized, triCenter);
        Vector3 nextTriCenter = (_cm.Vertices[nextTriLastEdgeStart] + _cm.Vertices[nextTriLastEdgeEnd] + _cm.Vertices[nextTriLastEdgeOther]) / 3;
        Vector3 nextTriClosestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[nextTriLastEdgeStart], (_cm.Vertices[nextTriLastEdgeStart] - _cm.Vertices[nextTriLastEdgeEnd]).normalized, nextTriCenter);

        Vector3 cornerNormal = Vector3.Cross(_cm.Vertices[cornerEdgeStart] - _cm.Vertices[cornerEdgeOther], _cm.Vertices[cornerEdgeStart] - _cm.Vertices[cornerEdgeEnd]).normalized;
        if (Vector3.Dot(cornerNormal, transform.up) < 0)
        {
            cornerNormal = -cornerNormal;
        }
        Vector3 edgeNormal = (closestPointOnEdge - triCenter).normalized;
        Vector3 nextTriEdgeNormal = (nextTriClosestPointOnEdge - nextTriCenter).normalized;
        Vector3 edgeNormal2 = Vector3.ProjectOnPlane(edgeNormal, cornerNormal).normalized;
        Vector3 nextTriEdgeNormal2 = Vector3.ProjectOnPlane(nextTriEdgeNormal, transform.up).normalized;

        bool planeHitsFarEdge = false;

        // if move direction is facing into the wall that we're sliding on
        Vector3 cornerAdjustedMoveDirection = Vector3.ProjectOnPlane((transform.rotation * input).normalized, cornerNormal).normalized;
        if (Vector3.Dot((transform.rotation * input).normalized, nextTriEdgeNormal2) < 0 && Vector3.Dot(cornerAdjustedMoveDirection, edgeNormal2) < 0)
        {
            foreach (Edge e in _cm.TriangleInfos[triangleIndex].edges)
            {
                if (_cm.Vertices[e.pointA] != _cm.Vertices[currentCornerInt] && _cm.Vertices[e.pointB] != _cm.Vertices[currentCornerInt])
                {
                    float farEdgeHitDistance = 0;
                    Ray farEdgeRay = CreateRay(_cm.Vertices[e.pointA], _cm.Vertices[e.pointB]);
                    if (plane.Raycast(farEdgeRay, out farEdgeHitDistance))
                    {
                        if (farEdgeHitDistance > 0 && farEdgeHitDistance <= Vector3.Distance(_cm.Vertices[e.pointA], _cm.Vertices[e.pointB]))
                        {
                            planeHitsFarEdge = true;
                        }
                    }
                }
            }
        }

        return planeHitsFarEdge;
    }

    Vector3 GetNextCut(EdgePoints edgePoints, Vector3 direction, Vector3 origin, Plane plane, CutType cutType)
    {
        // If we are on a new triangle and want to get the next triangle
        if (currentTriangleIndex != lastTriangleIndex && cutType == CutType.NextTriangle)
        {
            //Check each point in the next triangle’s points
            foreach (int point in new[] { edgePoints.Start, edgePoints.End, edgePoints.Other })
            {
                // Match Start and End with the current edge based on position
                // Assume that the left over one is the “other”
                if (_cm.Vertices[point] == _cm.Vertices[currentEdgePoints.Start]) currentEdgePoints.Start = point;
                else if (_cm.Vertices[point] == _cm.Vertices[currentEdgePoints.End]) currentEdgePoints.End = point;
                else currentEdgePoints.Other = point;
            }


            // Since we're checking the far side, we want the edgePoints to be the start -> other, other -> end
            // Usually we would just get the edge in front, but here, we're crossing the edge in front and then going onto the next triangle and testing the other edges
            edgePoints.Start = currentEdgePoints.Start;
            edgePoints.Other = currentEdgePoints.End;
            edgePoints.End = currentEdgePoints.Other;
        }

        // Create rays of edges to use with planar casts
        // START >> END edge
        Ray startEndRay = CreateRay(_cm.Vertices[edgePoints.Start], _cm.Vertices[edgePoints.End]);
        // END >> OTHER edge
        Ray endOtherRay = CreateRay(_cm.Vertices[edgePoints.End], _cm.Vertices[edgePoints.Other]);
        // START >> OTHER edge
        Ray startOtherRay = CreateRay(_cm.Vertices[edgePoints.Start], _cm.Vertices[edgePoints.Other]);

        // record ray's lengths, since Ray.direction does not have a magnitude
        // In order of size, because that's how edges are defined.
        float startEndRayLength = Vector3.Distance(_cm.Vertices[edgePoints.Start], _cm.Vertices[edgePoints.End]);
        float endOtherRayLength = Vector3.Distance(_cm.Vertices[edgePoints.End], _cm.Vertices[edgePoints.Other]);
        float startOtherRayLength = Vector3.Distance(_cm.Vertices[edgePoints.Start], _cm.Vertices[edgePoints.Other]);

        // A variable for Plane.Raycast to store distance in
        float startEndRayHitDistance = -1;
        float endOtherRayHitDistance = -1;
        float startOtherRayHitDistance = -1;
        // Temporarily make result: origin, so that if all else fails we get put back in the same spot
        Vector3 result = origin;
        // OTHER >> START edge

        // Get distances along each edge that plane intersects
        plane.Raycast(startEndRay, out startEndRayHitDistance);
        plane.Raycast(endOtherRay, out endOtherRayHitDistance);
        plane.Raycast(startOtherRay, out startOtherRayHitDistance);

        // If this isn’t the first move since being on the mesh
        if (firstMoveDone)
        {
            // And we hit on the positive side of the startEndRay (somewhere beyond the origin and in the direction of the ray.

            if (startEndRayHitDistance > 0)
            {
                //... and the point along the ray is within the length of the edge...
                if (startEndRayHitDistance <= startEndRayLength
                 || startEndRayHitDistance > startEndRayLength && endOtherRayHitDistance > endOtherRayLength && startOtherRayHitDistance < startOtherRayLength && startOtherRayHitDistance > 0
                 || startEndRayHitDistance > startEndRayLength && startOtherRayHitDistance > startOtherRayLength && endOtherRayHitDistance < endOtherRayLength && endOtherRayHitDistance > 0
                 )
                {
                    // and the cut type is next or
                    // start/test AND the intersection point is in front of the player
                    if (cutType == CutType.NextTriangle ||
                       ((cutType == CutType.Start || cutType == CutType.Test) &&
                       (Vector3.Dot(direction.normalized, (origin - startEndRay.GetPoint(startEndRayHitDistance)).normalized) <= 0
                 || startEndRayHitDistance > startEndRayLength && endOtherRayHitDistance > endOtherRayLength && startOtherRayHitDistance < startOtherRayLength && startOtherRayHitDistance > 0
                 || startEndRayHitDistance > startEndRayLength && startOtherRayHitDistance > startOtherRayLength && endOtherRayHitDistance < endOtherRayLength && endOtherRayHitDistance > 0)))
                    {
                        result = startEndRay.GetPoint(startEndRayHitDistance);

                        // if we're not just doing a test probe, return the relevant edges. The edge we're testing here is p1 to p2, so we make those start and end
                        if (cutType != CutType.Test)
                        {
                            currentEdgePoints.SetEdges(edgePoints.Start, edgePoints.End, edgePoints.Other);
                        }
                    }
                }
            }
            else if (startEndRayHitDistance == 0)
            {
                if (cutType != CutType.Test && lastTriangleIndex != currentTriangleIndex)
                {
                    result = _cm.Vertices[edgePoints.Start];
                    currentEdgePoints.SetEdges(edgePoints.Start, edgePoints.End, edgePoints.Other);
                    cutFound = true;
                }
            }

            // END >> OTHER edge
            if (endOtherRayHitDistance > 0)
            {
                // same as last time, it shouldn't be able to hit if the last one already did
                if (endOtherRayHitDistance <= endOtherRayLength
                 || endOtherRayHitDistance > endOtherRayLength && startEndRayHitDistance > startEndRayLength && startOtherRayHitDistance < startOtherRayLength && startOtherRayHitDistance > 0
                 || endOtherRayHitDistance > endOtherRayLength && startOtherRayHitDistance > startOtherRayLength && startEndRayHitDistance < startEndRayLength && startEndRayHitDistance > 0
                 )
                {

                    if (cutType == CutType.NextTriangle ||
                       ((cutType == CutType.Start || cutType == CutType.Test) &&
                       (Vector3.Dot(direction.normalized, (origin - endOtherRay.GetPoint(endOtherRayHitDistance)).normalized) <= 0
                 || endOtherRayHitDistance > endOtherRayLength && startEndRayHitDistance > startEndRayLength && startOtherRayHitDistance < startOtherRayLength && startOtherRayHitDistance > 0
                 || endOtherRayHitDistance > endOtherRayLength && startOtherRayHitDistance > startOtherRayLength && startEndRayHitDistance < startEndRayLength && startEndRayHitDistance > 0)))
                    {
                        result = endOtherRay.GetPoint(endOtherRayHitDistance);
                        if (cutType != CutType.Test)
                        {
                            currentEdgePoints.SetEdges(edgePoints.End, edgePoints.Other, edgePoints.Start);
                            cutFound = true;
                        }
                    }
                }
            }
            else if (endOtherRayHitDistance == 0 && !cutFound)
            {
                if (cutType != CutType.Test && lastTriangleIndex != currentTriangleIndex)
                {
                    result = _cm.Vertices[edgePoints.End];
                    currentEdgePoints.SetEdges(edgePoints.End, edgePoints.Other, edgePoints.Start);
                }
            }

            // we only test the START >> END edge if it's the start of a new loop or a test because we just passed through this
            if (startOtherRayHitDistance > 0)
            {
                if (startOtherRayHitDistance <= startOtherRayLength
                 || startOtherRayHitDistance > startOtherRayLength && startEndRayHitDistance > startEndRayLength && endOtherRayHitDistance < endOtherRayLength && endOtherRayHitDistance > 0
                 || startOtherRayHitDistance > startOtherRayLength && endOtherRayHitDistance > endOtherRayLength && startEndRayHitDistance < startEndRayLength && startEndRayHitDistance > 0
                 )
                {
                    if ((cutType == CutType.Start || cutType == CutType.Test) && (
                    (Vector3.Dot(direction.normalized, (origin - startOtherRay.GetPoint(startOtherRayHitDistance)).normalized) <= 0
                     || startOtherRayHitDistance > startOtherRayLength && startEndRayHitDistance > startEndRayLength && endOtherRayHitDistance < endOtherRayLength && endOtherRayHitDistance > 0
                     || startOtherRayHitDistance > startOtherRayLength && endOtherRayHitDistance > endOtherRayLength && startEndRayHitDistance < startEndRayLength && startEndRayHitDistance > 0))
                    )
                    {
                        result = startOtherRay.GetPoint(startOtherRayHitDistance);
                        if (cutType != CutType.Test)
                        {
                            currentEdgePoints.SetEdges(edgePoints.Start, edgePoints.Other, edgePoints.End);
                            cutFound = true;
                        }
                    }
                }
            }
            else if (startOtherRayHitDistance == 0 && !cutFound)
            {
                if (cutType != CutType.Test && lastTriangleIndex != currentTriangleIndex)
                {
                    result = _cm.Vertices[edgePoints.Start];
                    currentEdgePoints.SetEdges(edgePoints.Start, edgePoints.Other, edgePoints.End);
                    cutFound = true;
                }
            }
        }

        firstMoveDone = true;
        return result;
    }

    Ray CreateRay(Vector3 point1, Vector3 point2) => new Ray { origin = point1, direction = (point2 - point1).normalized };

    void TryStartClimb()
    {
        // firstMoveDone represents whether we are moving on the mesh for the first time since not climbing.
        // This is because we will just want to check which edge is in front.
        firstMoveDone = false;

        if (ClimbDetectionUtility.TryGetClimbableMesh(transform, previousRayCastOrigin, layerMask, out RaycastHit raycastHit, out _cm))
        {
            isClimbing = true;
            transform.position = raycastHit.point;
            currentTriangleIndex = raycastHit.triangleIndex * 3;
            barycentricCoordinate = raycastHit.barycentricCoordinate;

            currentEdgePoints.Set(
                start: _cm.Triangles[currentTriangleIndex],
                end: _cm.Triangles[currentTriangleIndex + 1],
                other: _cm.Triangles[currentTriangleIndex + 2]
            );

            Vector3 triCenter = (_cm.Vertices[currentEdgePoints.Start] + _cm.Vertices[currentEdgePoints.End] + _cm.Vertices[currentEdgePoints.Other]) / 3;

            Plane plane = new Plane(-transform.right, triCenter);

            testCut = GetNextCut(currentEdgePoints, transform.forward, triCenter, plane, CutType.Start);
            lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);
            testCut = GetNextCut(currentEdgePoints, -transform.forward, triCenter, plane, CutType.Test);
            barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, _cm.Vertices[currentEdgePoints.Start], _cm.Vertices[currentEdgePoints.End], _cm.Vertices[currentEdgePoints.Other]);
        }
        previousRayCastOrigin = transform.position + transform.up * 0.1f;
    }

    void LeaveClimbableMesh()
    {
        // MYSTERY
        transform.position = CharacterModel.position;
        rb.position = CharacterModel.position;
        rb.rotation = CharacterModel.rotation;
        rb.GetComponent<KinematicCharacterMotor>().SetPositionAndRotation(CharacterModel.position, CharacterModel.rotation);
        isClimbing = false;
        CharacterModel.localRotation = Quaternion.identity;
        Physics.SyncTransforms();
    }

    void GetEdgeOnSpecificTriangle(int index, ref int start, ref int end)
    {
        // check the next triangle's edges, we're setting lastEdgeStartRealTemp, lastEdgeEndRealTemp, lastEdgeOtherRealTemp because we don't want to override lastEdgeStartReal etc
        foreach (Edge e in _cm.TriangleInfos[index].edges)
        {
            // if the positions of the points on the line we crossed match the edge of the triangle being compared currently, store it
            if (_cm.Vertices[currentEdgePoints.Start] == _cm.Vertices[e.pointA])
            {
                start = e.pointA;
            }

            if (_cm.Vertices[currentEdgePoints.Start] == _cm.Vertices[e.pointB])
            {
                start = e.pointB;
            }

            if (_cm.Vertices[currentEdgePoints.End] == _cm.Vertices[e.pointA])
            {
                start = e.pointA;
            }

            if (_cm.Vertices[currentEdgePoints.End] == _cm.Vertices[e.pointB])
            {
                end = e.pointB;
            }

            if (start == -1)
            {
                if (_cm.Vertices[currentEdgePoints.Other] == _cm.Vertices[e.pointA])
                {
                    start = e.pointA;
                }
                if (_cm.Vertices[currentEdgePoints.Other] == _cm.Vertices[e.pointB])
                {
                    start = e.pointB;
                }
            }

            if (end == -1)
            {
                if (_cm.Vertices[currentEdgePoints.Other] == _cm.Vertices[e.pointA])
                {
                    end = e.pointA;
                }
                if (_cm.Vertices[currentEdgePoints.Other] == _cm.Vertices[e.pointB])
                {
                    end = e.pointB;
                }
            }
        }
        if (start == -1 && end == -1)
        {
            DebugUtils.DebugTriangle(_cm, index, Color.blue);
#if UNITY_EDITOR
            EditorApplication.isPaused = true;
#endif
        }
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
        NextTriangle,
        Test
    }

    public enum MovementMode
    {
        Car,
        Directional
    }
}

public struct EdgePoints
{
    public int Start;
    public int End;
    public int Other;

    public EdgePoints(bool initialize = true)
    {
        Start = -1;
        End = -1;
        Other = -1;
    }

    public void Set(int start, int end, int other)
    {
        Start = start;
        End = end;
        Other = other;
    }

}

public static class EdgePointsExtensions
{
    public static void SetEdges(ref this EdgePoints edgePoints, int start, int end, int other)
    {
        edgePoints.Start = start;
        edgePoints.End = end;
        edgePoints.Other = other;
    }
}