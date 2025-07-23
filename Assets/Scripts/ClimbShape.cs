using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Overhang;
using KinematicCharacterController;
using System;
using System.Linq;

public class ClimbShape : MonoBehaviour
{
    ClimbableMesh _cm;

    public Transform CharacterModel;
    Rigidbody _rb;
    int _index, _lastIndex, _previousLastIndex, _previousIndex;

    Vector3 _barycentricCoordinate, _lastBarycentricCoordinate, _barycentricCoordinateBehind;

    Plane _plane, _testPlane;

    Vector3 _cut, _testCut;

    EdgePoints _currentEdge;
    EdgePoints _lastEdge;

    int _previousLastEdgeStart;
    int _previousLastEdgeEnd;
    int _previousLastEdgeOther;

    float _targetSpeed;

    [Min(0)]
    public float DirectionalSpeed, ClimbingSpeed;

    Vector3 _currentCheckPosition;

    Vector3 _moveDirection;
    Vector3 _castDirectionTest;

    public LayerMask _layerMask;
    public LayerMask _layerMaskForwardProjection;

    Vector3 _input;
    Vector3 _newPosition;

    bool _isClimbing;
    bool _onEdge;

    int _cornerReached;
    bool _firstMoveDone;
    bool _goForwardTest = false;

    float _deltaTime;
    float _frameRate;

    public MovementMode _movementMode;
    Vector3 _lastForward;

    public bool ForceSlide;
    public bool ForceSlideForwardProjection;
    float _slideCoefficient = 1;

    public Collider DepenetrationCapsule;

    Quaternion _afterDepenetrateRotation;

    Vector3 _previousRayCastPosition;

    void Awake()
    {
        Physics.autoSyncTransforms = true;
        UnityEngine.Cursor.visible = false;
        UnityEngine.Cursor.lockState = CursorLockMode.Locked;
        Application.targetFrameRate = 300;
        // set the plane used for pathfinding to be oriented to the character
        _plane = new Plane(-transform.right, transform.position);
        _rb = GetComponent<Rigidbody>();
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
        _deltaTime += (Time.unscaledDeltaTime - _deltaTime) * 0.1f;

        // Calculate the frame rate in frames per second
        _frameRate = 1.0f / _deltaTime;

        // if not on a mesh, raycast to find a mesh
        if (!_isClimbing)
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
            // tempInput = Mathf2.RotateAroundAxis(tempInput, Vector3.up, Camera.main.transform.rotation.eulerAngles.y);

            // This is for debugging - makes the character go forwards and backwards over an edge rapidly, so I don't have to hammer the keyboard to test. 
            if (Input.GetKeyDown(KeyCode.K))
            {
                _goForwardTest = !_goForwardTest;
                // testPosition = transform.position.x;
            }
            if (_goForwardTest)
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
                _input = tempInput;
            }

            // calculate movement on the mesh based on input
            Travel(tempInput, false, false);
            Vector3 pointOnTriangle = GetPositionFromBarycentric(_barycentricCoordinate, _currentEdge);
            transform.position = pointOnTriangle;


            int depenetrationIterations = 1;
            for (int i = 0; i < depenetrationIterations; i++)
            {
                bool isFinalPass = i == depenetrationIterations - 1;
                Depenetrate(isFinalPass);
                Physics.SyncTransforms();
            }
            // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
            pointOnTriangle = GetPositionFromBarycentric(_barycentricCoordinate, _currentEdge);
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
        if (_cm.Vertices[_cm.Triangles[index]] == previousLastEdgeStartPosition)
        {
            _currentEdge.Start = _cm.Triangles[index];
        }
        if (_cm.Vertices[_cm.Triangles[index + 1]] == previousLastEdgeStartPosition)
        {
            _currentEdge.Start = _cm.Triangles[index + 1];
        }
        if (_cm.Vertices[_cm.Triangles[index + 2]] == previousLastEdgeStartPosition)
        {
            _currentEdge.Start = _cm.Triangles[index + 2];
        }

        if (_cm.Vertices[_cm.Triangles[index]] == previousLastEdgeEndPosition)
        {
            _currentEdge.End = _cm.Triangles[index];
        }
        if (_cm.Vertices[_cm.Triangles[index + 1]] == previousLastEdgeEndPosition)
        {
            _currentEdge.End = _cm.Triangles[index + 1];
        }
        if (_cm.Vertices[_cm.Triangles[index + 2]] == previousLastEdgeEndPosition)
        {
            _currentEdge.End = _cm.Triangles[index + 2];
        }

        if (_cm.Vertices[_cm.Triangles[index]] == previousLastEdgeOtherPosition)
        {
            _currentEdge.Other = _cm.Triangles[index];
        }
        if (_cm.Vertices[_cm.Triangles[index + 1]] == previousLastEdgeOtherPosition)
        {
            _currentEdge.Other = _cm.Triangles[index + 1];
        }
        if (_cm.Vertices[_cm.Triangles[index + 2]] == previousLastEdgeOtherPosition)
        {
            _currentEdge.Other = _cm.Triangles[index + 2];
        }
    }

    void Travel(Vector3 direction, bool depenetratePass, bool isFinalPass)
    {
        // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
        Vector3 pointOnTriangle = GetPositionFromBarycentric(_barycentricCoordinate, _currentEdge);
        // At the end of last loop we do a 'test' cut to get the next position BEHIND
        // This is recorded from the tri center
        Vector3 behindPointOnTriangle = GetPositionFromBarycentric(_barycentricCoordinateBehind, _currentEdge);
        // Quaternion rotationLast = transform.rotation;
        // At the end of last loop we do a 'test' cut to get the next position in FRONT
        // Here we recalculate it with deformations. This is the forward cut, NOT the movement direction cut 
        // This is also recorded from the tri center
        Vector3 forwardPointOnTriangle = GetPositionFromBarycentric(_lastBarycentricCoordinate, _currentEdge);

        transform.position = pointOnTriangle;

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        Vector3 tempForward = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        if (Input.GetKey(KeyCode.LeftShift))
        {
            if (_movementMode != MovementMode.Car)
            {

                Vector3 previousLastEdgeStartPosition = _cm.Vertices[_currentEdge.Start];
                Vector3 previousLastEdgeEndPosition = _cm.Vertices[_currentEdge.End];
                Vector3 previousLastEdgeOtherPosition = _cm.Vertices[_currentEdge.Other];

                int indexTemp = _cm.GetArea(_index / 3) * 3;

                if (indexTemp != -3)
                {
                    _index = indexTemp;
                    SetEdges(previousLastEdgeStartPosition, previousLastEdgeEndPosition, previousLastEdgeOtherPosition, _index);

                    Vector3 triCenterTemp = (_cm.Vertices[_currentEdge.Start] + _cm.Vertices[_currentEdge.End] + _cm.Vertices[_currentEdge.Other]) / 3;

                    Plane plane = new Plane(-transform.right, triCenterTemp);

                    _barycentricCoordinate = Mathf2.GetBarycentricCoordinates(_newPosition, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);

                    _testCut = FindTrianglePlaneIntersection(_currentEdge, transform.forward, triCenterTemp, plane, CutType.Start);
                    _lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);
                    _testCut = FindTrianglePlaneIntersection(_currentEdge, -transform.forward, triCenterTemp, plane, CutType.Test);
                    _barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);

                    _movementMode = MovementMode.Car;
                }
            }
        }
        else
        {
            if (_movementMode != MovementMode.Directional)
            {
                Vector3 previousLastEdgeStartPosition = _cm.Vertices[_currentEdge.Start];
                Vector3 previousLastEdgeEndPosition = _cm.Vertices[_currentEdge.End];
                Vector3 previousLastEdgeOtherPosition = _cm.Vertices[_currentEdge.Other];

                int indexTemp = _cm.GetMainBody(_index / 3) * 3;

                _cm.RecalculateMesh(false);

                if (indexTemp != -3)
                {
                    _index = indexTemp;
                    _onEdge = false;

                    SetEdges(previousLastEdgeStartPosition, previousLastEdgeEndPosition, previousLastEdgeOtherPosition, _index);
                    Vector3 triCenterTemp = (_cm.Vertices[_currentEdge.Start] + _cm.Vertices[_currentEdge.End] + _cm.Vertices[_currentEdge.Other]) / 3;

                    Plane plane = new Plane(-transform.right, triCenterTemp);

                    _testCut = FindTrianglePlaneIntersection(_currentEdge, transform.forward, triCenterTemp, plane, CutType.Start);
                    _lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);
                    _testCut = FindTrianglePlaneIntersection(_currentEdge, -transform.forward, triCenterTemp, plane, CutType.Test);
                    _barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);

                    _movementMode = MovementMode.Directional;
                }
            }
        }


        EdgePoints currentEdgeReal = new();
        currentEdgeReal.Set(0, 0, 0);

        // Because of hard edges, the current triangle will have different vertices to what's currently stores in lastEdgeStart, lastEdgeEnd, lastEdgeOther
        // We need to check this triangle's edge positions against those and update.
        // lastEdgeStartReal,lastEdgeEndReal, lastEdgeOtherReal are the real indices of the current triangle
        if (!_cm.TriangleAdjacencyInfo.ContainsKey(_index))
        {
            Debug.Log("Index: " + _index);
            foreach (var item in _cm.TriangleAdjacencyInfo)
            {
                Debug.Log(item.Key);
#if UNITY_EDITOR
                EditorApplication.isPlaying = false;
#endif
            }
        }
        foreach (Edge e in _cm.TriangleAdjacencyInfo[_index].edges)
        {
            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.Start])
                currentEdgeReal.Start = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.Start])
                currentEdgeReal.Start = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.End])
                currentEdgeReal.End = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.End])
                currentEdgeReal.End = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.Other])
                currentEdgeReal.Other = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.Other])
                currentEdgeReal.Other = e.pointB;
        }


        // Calculate the ground normal based on coordinates from the last frame of where we should be standing, translated to the new, deformed triangle
        Vector3 groundNormal = GetNormalFromBarycentric(_barycentricCoordinate, currentEdgeReal);

        float turnAngle = 0;

        turnAngle += Input.GetKey(KeyCode.E) ? 100 * Time.deltaTime : 0;
        turnAngle -= Input.GetKey(KeyCode.Q) ? 100 * Time.deltaTime : 0;
        tempForward = Quaternion.Euler(0, turnAngle, 0) * tempForward;

        if (_movementMode == MovementMode.Directional)
        {
            if (direction.magnitude > 0)
            {
                tempForward = _lastForward;

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
        Vector3 triCenter = (_cm.Vertices[_currentEdge.Start] + _cm.Vertices[_currentEdge.End] + _cm.Vertices[_currentEdge.Other]) / 3;
        Vector3 closestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[_currentEdge.Start], (_cm.Vertices[_currentEdge.Start] - _cm.Vertices[_currentEdge.End]).normalized, triCenter);
        Vector3 getNewtempForwardDirection = Vector3.zero;
        Vector3 getNewtempForwardDirectionBehind = Vector3.zero;

        if (_movementMode == MovementMode.Directional)
        {
            // This check works as intended
            if (direction.magnitude > 0)
            {
                float angleToRotateby = Camera.main.transform.rotation.eulerAngles.y - transform.rotation.eulerAngles.y;
                _input = Quaternion.Euler(0, angleToRotateby, 0) * _input;

                // if (isFinalPass)
                // {
                // Debug.Log("Final Pass");
                if (!depenetratePass)
                {
                    _afterDepenetrateRotation = Quaternion.LookRotation(Quaternion.Euler(0, transform.rotation.eulerAngles.y, 0) * _input);
                }
                if (isFinalPass)
                {
                    CharacterModel.rotation = _afterDepenetrateRotation;
                }
                Plane NewtempForwardPlane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterModel.right, transform.position);
                // Debug.DrawLine(transform.position, transform.position + NewtempForwardPlane.normal);
                Vector3 PlaneNormalRight = Mathf2.RotateAroundAxis(NewtempForwardPlane.normal, groundNormal, 90);
                // Debug.DrawLine(transform.position, transform.position + PlaneNormalRight, Color.blue);
                getNewtempForwardDirection = FindTrianglePlaneIntersection(_currentEdge, PlaneNormalRight, transform.position, NewtempForwardPlane, CutType.Test);
                getNewtempForwardDirectionBehind = FindTrianglePlaneIntersection(_currentEdge, -PlaneNormalRight, transform.position, NewtempForwardPlane, CutType.Test);
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
                    _afterDepenetrateRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(tempForward, Vector3.up).normalized, Vector3.up);

                }
                if (isFinalPass)
                {
                    if (!_goForwardTest)
                    {
                        // EditorApplication.isPaused = true;
                    }
                    CharacterModel.rotation = _afterDepenetrateRotation;
                }
            }
            _targetSpeed = DirectionalSpeed;

        }
        else
        {
            _targetSpeed = ClimbingSpeed;
            CharacterModel.localRotation = Quaternion.identity;
        }

        // This calculates the move direction. What happens if I just rotate this?
        // Get the move direction relative to the player's orientation
        _moveDirection = (transform.rotation * _input).normalized;

        // moveDirection = Mathf2.RotateAroundAxis(input, Vector3.up, Camera.main.transform.rotation.eulerAngles.y);
        // newDirection - can't remember what this does but it's possible it will change in the loop but we want to keep moveDirection the same.
        Vector3 newDirection = _moveDirection;


        // transform.rotation = Quaternion.LookRotation(newDirection);
        //calculate distance to move

        Vector3 _checkPositionStart = CharacterModel.position + CharacterModel.up * 0.05f;
        Vector3 _checkPositionEnd = CharacterModel.position + CharacterModel.up * 0.15f;
        Collider[] colliders = Physics.OverlapCapsule(_checkPositionStart, _checkPositionEnd, 0.05f, _layerMaskForwardProjection);

        bool depenetrate = false;
        bool slide = false;
        float distance = direction.magnitude * _targetSpeed * Time.deltaTime;

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
                    Physics.ComputePenetration(DepenetrationCapsule, DepenetrationCapsule.transform.position, DepenetrationCapsule.transform.rotation, col, col.transform.position, col.transform.rotation, out depenetrationDirection, out depenetrationDistance);
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

                _moveDirection = tempForward;
                newDirection = tempForward;
                _input = tempForward;
                depenetrate = true;
                _plane = new Plane(Mathf2.RotateAroundAxis(tempForward, transform.up, 90), transform.position);
            }
            else
            {
                // distance = totalDepenetrationDirection.magnitude * 0.9f;
                tempForward = totalDepenetrationDirection.normalized;

                _moveDirection = tempForward;
                newDirection = tempForward;
                _input = tempForward;
                _plane = new Plane(Mathf2.RotateAroundAxis(tempForward, transform.up, 90), transform.position);
                // Debug.Log("Collision not Found");
            }
        }
        else
        {

        }
        if (!depenetrate && !slide)
        {
            // Create plane to calculate "cuts" from
            if (_movementMode == MovementMode.Car)
            {
                _plane = new Plane(CharacterModel.rotation * (Quaternion.Euler(0, 90, 0) * _input), transform.position);
            }
            else
            {
                _plane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterModel.right, transform.position);
            }

        }

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
        if (_onEdge && Vector3.Dot(_moveDirection, (closestPointOnEdge - triCenter).normalized) < 0)
        {
            _onEdge = false;
        }

        // if not on edge, do a start cut
        // start cut considers all edges
        _cut = transform.position;



        if (!_onEdge)
        {
            _cut = FindTrianglePlaneIntersection(_currentEdge, newDirection, transform.position, _plane, CutType.Start);
        }
        else // else do a test cut - test cut considers all edges and returns a "cut" but doesn't change the current edge/index information. this is good because we want to be able to get an edge we are standing directly on.
        {
            _cut = FindTrianglePlaneIntersection(_currentEdge, newDirection, transform.position, _plane, CutType.Test);
        }

        // Add to the total distance to the cut from the position
        totalDistanceChecked += Vector3.Distance(transform.position, _cut);
        // make the currentCheckPosition the same as cut
        _currentCheckPosition = _cut;

        // i is used to break out of infinite while loops
        int i = 0;

        // newPosition is updated as we walk through the mesh
        _newPosition = Vector3.zero;
        // if we didn't reach the next edge with this movement, calculate the new position in the current triangle
        if (totalDistanceChecked >= distance)
        {
            _newPosition = transform.position + Vector3.ClampMagnitude(_cut - transform.position, distance);

            // Debug.Log("Clamp magnitude " + Vector3.ClampMagnitude(cut - transform.position, distance).magnitude);
            // Debug.Log("Distance " + distance);
        }
        else // else make the next check go from the point on the next edge
        {
            _newPosition = _cut;
        }


        float remainingDistance = distance - Vector3.Distance(transform.position, _newPosition);
        _castDirectionTest = transform.forward;

        // Why do we need lastIndex, previousLastIndex and previousIndex?

        // lastIndex is
        // previousLastIndex is
        // previousIndex is

        _previousLastIndex = _index;
        _previousLastEdgeStart = _currentEdge.Start;
        _previousLastEdgeEnd = _currentEdge.End;
        _previousLastEdgeOther = _currentEdge.Other;

        // while we have checked less distance than the edge
        while (totalDistanceChecked < distance)
        {
            // if not on edge / just started moving away from edge
            if (!_onEdge)
            {
                // set the last index to be the index - this is because we're about to check the next tri to see if we're on an edge, and we want to compare index to lastIndex
                _lastIndex = _index;

                // take the edge we just passed, then get the tri that we didn't just check
                // don't worry, index is set inGetNextTri() 
                int[] nextTriIndices = GetNextTri();
                EdgePoints nextTri = new();
                nextTri.Set(nextTriIndices[0], nextTriIndices[1], nextTriIndices[2]);
                // if GetNextTri() returns same value as index, we are on an edge, else do nothing
                if (_index == _lastIndex)
                {
                    _onEdge = true;
                    Debug.Log(_onEdge);
                }
                else
                {
                    _cut = FindTrianglePlaneIntersection(nextTri, newDirection, _newPosition, _plane, CutType.Next);

                    totalDistanceChecked += Vector3.Distance(_currentCheckPosition, _cut);

                    Vector3 oldPosition = _newPosition;

                    if (totalDistanceChecked >= distance)
                    {
                        _newPosition = _newPosition + Vector3.ClampMagnitude(_cut - _newPosition, remainingDistance);
                    }
                    else
                    {
                        _newPosition = _cut;
                    }

                    _newPosition = _currentCheckPosition + Vector3.ClampMagnitude(_cut - _currentCheckPosition, remainingDistance);

                    remainingDistance = remainingDistance - Vector3.Distance(oldPosition, _newPosition);

                    // project direction on current normal to get new direction
                    // direction is used to see if cuts are valid/ in the direction if your input

                    if (_cut != _currentCheckPosition)
                    {
                        newDirection = (_cut - _currentCheckPosition).normalized;
                        _currentCheckPosition = _cut;
                    }

                    i++;
                    if (i > 100)
                    {
                        _currentEdge.Start = _previousLastEdgeStart;
                        _currentEdge.End = _previousLastEdgeEnd;
                        _currentEdge.Other = _previousLastEdgeOther;
                        _index = _previousLastIndex;
                        totalDistanceChecked = distance;

                        break;
                    }

                    Plane tempPlane = new Plane(-transform.right, transform.position);

                }
            }
            else // if on edge
            {
                // get the point where the character is trying to move, if it moved off the current tri into space
                Vector3 movePositionAttempt = _newPosition + newDirection * remainingDistance;
                Vector3 slidePoint = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End]);

                int currentCornerInt = -1;
                int currentCornerIntOther = 0;
                if (slidePoint == _cm.Vertices[_currentEdge.Start])
                {
                    currentCornerInt = _currentEdge.Start;
                    currentCornerIntOther = _currentEdge.End;
                }
                if (slidePoint == _cm.Vertices[_currentEdge.End])
                {
                    currentCornerInt = _currentEdge.End;
                    currentCornerIntOther = _currentEdge.Start;
                }

                // if we reached a corner
                if (currentCornerInt != -1)
                {
                    // Get the next edge. Since lastEdgeStart and lastEdgeEnd are always the ouside edge when edge-sliding, the next edge is the corner reached -> lastEdgeOther
                    Edge nextEdgeOnThisTriangle = new Edge();
                    if (currentCornerInt < _currentEdge.Other)
                    {
                        nextEdgeOnThisTriangle.pointA = currentCornerInt;
                        nextEdgeOnThisTriangle.pointB = _currentEdge.Other;
                    }
                    else
                    {
                        nextEdgeOnThisTriangle.pointA = _currentEdge.Other;
                        nextEdgeOnThisTriangle.pointB = currentCornerInt;
                    }
                    // If the next edge is another outside edge on the same triangle, switch to that. 
                    if (_cm.EdgeAdjacencyInfo.ContainsKey(nextEdgeOnThisTriangle) && _cm.EdgeAdjacencyInfo[nextEdgeOnThisTriangle].triangleA == _cm.EdgeAdjacencyInfo[nextEdgeOnThisTriangle].triangleB)
                    {

                        _currentEdge.Other = currentCornerIntOther;
                        _currentEdge.Start = nextEdgeOnThisTriangle.pointA;
                        _currentEdge.End = nextEdgeOnThisTriangle.pointB;

                        Ray tempRay = CreateRay(_newPosition, movePositionAttempt);
                        Plane tempPlane = new Plane((slidePoint - _newPosition).normalized, slidePoint);
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

                        Ray tempRay = CreateRay(_newPosition, movePositionAttempt);
                        Plane tempPlane = new Plane((slidePoint - _newPosition).normalized, slidePoint);

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

                        if (slidePoint == _cm.Vertices[_currentEdge.Start])
                        {
                            _cornerReached = _currentEdge.Start;
                        }
                        else if (slidePoint == _cm.Vertices[_currentEdge.End])
                        {
                            _cornerReached = _currentEdge.End;
                        }

                        int timesLooped = 0;
                        int count = 0;
                        List<Edge> tempEdges = new List<Edge>();

                        Edge checkIsOnEdge = new();

                        int lastEdgeStartRealTemp = -1;
                        int lastEdgeEndRealTemp = -1;
                        Debug.Log(_index);
                        // check the next triangle's edges, we're setting lastEdgeStartRealTemp, lastEdgeStartEndTemp, lastEdgeOtherRealTemp because we don't want to override lastEdgeStartReal etc
                        foreach (Edge e in _cm.TriangleAdjacencyInfo[_index].edges)
                        {

                            // if the positions of the points on the line we crossed match the edge of the triangle being compared currently, store it
                            if (_cm.Vertices[_currentEdge.Start] == _cm.Vertices[e.pointA])
                            {
                                lastEdgeStartRealTemp = e.pointA;
                            }
                            if (_cm.Vertices[_currentEdge.Start] == _cm.Vertices[e.pointB])
                            {
                                lastEdgeStartRealTemp = e.pointB;
                            }
                            if (_cm.Vertices[_currentEdge.End] == _cm.Vertices[e.pointA])
                            {
                                lastEdgeEndRealTemp = e.pointA;
                            }
                            if (_cm.Vertices[_currentEdge.End] == _cm.Vertices[e.pointB])
                            {
                                lastEdgeEndRealTemp = e.pointB;
                            }
                            if (lastEdgeStartRealTemp == -1)
                            {
                                if (_cm.Vertices[_currentEdge.Other] == _cm.Vertices[e.pointA])
                                {
                                    lastEdgeStartRealTemp = e.pointA;
                                }
                                if (_cm.Vertices[_currentEdge.Other] == _cm.Vertices[e.pointB])
                                {
                                    lastEdgeStartRealTemp = e.pointB;
                                }
                            }
                            if (lastEdgeEndRealTemp == -1)
                            {
                                if (_cm.Vertices[_currentEdge.Other] == _cm.Vertices[e.pointA])
                                {
                                    lastEdgeEndRealTemp = e.pointA;
                                }
                                if (_cm.Vertices[_currentEdge.Other] == _cm.Vertices[e.pointB])
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
                            DebugTriangle(_index, Color.blue);
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

                        if (!_cm.EdgeAdjacencyInfo.ContainsKey(checkIsOnEdge))
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
                        if (_cm.EdgeAdjacencyInfo[checkIsOnEdge].triangleA == _cm.EdgeAdjacencyInfo[checkIsOnEdge].triangleB)
                        {
                            foreach (Edge e in _cm.EdgesAttachedToCorner[_cornerReached])
                            {
                                if (!(_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.Start] && _cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.End]) &&
                                    !(_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.End] && _cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.Start]))
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
                            int triangleCheckCorner = _index;
                            int lastTriangleCheckCorner = -1;

                            Vector3 triCenterTemp1 = Vector3.zero;
                            Vector3 triCenterTemp2 = Vector3.zero;
                            Vector3 cornerEdgeNormalTemp1 = Vector3.zero;
                            Vector3 cornerEdgeNormalTemp2 = Vector3.zero;

                            bool firstCheck = false;
                            bool edgeFound = false;

                            while (!edgeFound)
                            {
                                foreach (Edge e in _cm.EdgesAttachedToCorner[_cornerReached])
                                {
                                    // Debug.DrawLine(cm.meshVerts[e.pointA],cm.meshVerts[e.pointB],Color.magenta);
                                    Vector3 triCenterTemp = (_cm.Vertices[triangleCheckCorner] + _cm.Vertices[triangleCheckCorner + 1] + _cm.Vertices[triangleCheckCorner + 2]) / 3;
                                    Vector3 closestPointTemp = Mathf2.GetClosestPointOnFiniteLine(triCenterTemp, _cm.Vertices[e.pointA], _cm.Vertices[e.pointB]);
                                    Vector3 edgeNormalTemp = (triCenterTemp - closestPointTemp).normalized;

                                    int edgeTriangleA = _cm.EdgeAdjacencyInfo[e].triangleA;
                                    int edgeTriangleB = _cm.EdgeAdjacencyInfo[e].triangleB;

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

                            foreach (Edge e in _cm.TriangleAdjacencyInfo[_cm.EdgeAdjacencyInfo[cornerEdge1].triangleA].edges)
                            {
                                if (_cm.Vertices[e.pointA] != _cm.Vertices[_currentEdge.Start] && _cm.Vertices[e.pointA] != _cm.Vertices[_currentEdge.End])
                                {
                                    cornerEdgeOther1 = e.pointA;
                                }
                                else if (_cm.Vertices[e.pointB] != _cm.Vertices[_currentEdge.Start] && _cm.Vertices[e.pointB] != _cm.Vertices[_currentEdge.End])
                                {
                                    cornerEdgeOther1 = e.pointB;
                                }
                            }

                            triCenterTemp1 = (_cm.Vertices[cornerEdge1.pointA] + _cm.Vertices[cornerEdge1.pointB] + _cm.Vertices[cornerEdgeOther1]) / 3;
                            cornerEdgeNormalTemp1 = (triCenterTemp1 - Mathf2.NearestPointOnLine(_cm.Vertices[cornerEdge1.pointA], (_cm.Vertices[cornerEdge1.pointA] - _cm.Vertices[cornerEdge1.pointB]).normalized, triCenterTemp1)).normalized;

                            timesLooped = 0;
                            triangleCheckCorner = _index;
                            lastTriangleCheckCorner = -1;

                            firstCheck = false;
                            edgeFound = false;

                            while (!edgeFound)
                            {
                                foreach (Edge e in _cm.EdgesAttachedToCorner[_cornerReached])
                                {
                                    int edgeTriangleA = _cm.EdgeAdjacencyInfo[e].triangleA;
                                    int edgeTriangleB = _cm.EdgeAdjacencyInfo[e].triangleB;

                                    if (edgeTriangleA == triangleCheckCorner && !edgeFound)
                                    {
                                        if (!(_cm.Vertices[e.pointA] == _cm.Vertices[firstEdgeCrossed.pointA] && _cm.Vertices[e.pointB] == _cm.Vertices[firstEdgeCrossed.pointB] ||
                                            _cm.Vertices[e.pointA] == _cm.Vertices[firstEdgeCrossed.pointB] && _cm.Vertices[e.pointB] == _cm.Vertices[firstEdgeCrossed.pointA]))
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
                                        if (!(_cm.Vertices[e.pointA] == _cm.Vertices[firstEdgeCrossed.pointA] && _cm.Vertices[e.pointB] == _cm.Vertices[firstEdgeCrossed.pointB] ||
                                            _cm.Vertices[e.pointA] == _cm.Vertices[firstEdgeCrossed.pointB] && _cm.Vertices[e.pointB] == _cm.Vertices[firstEdgeCrossed.pointA]))
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

                            foreach (Edge e in _cm.TriangleAdjacencyInfo[_cm.EdgeAdjacencyInfo[cornerEdge2].triangleA].edges)
                            {
                                if (_cm.Vertices[e.pointA] != _cm.Vertices[_currentEdge.Start] && _cm.Vertices[e.pointA] != _cm.Vertices[_currentEdge.End])
                                {
                                    cornerEdgeOther2 = e.pointA;
                                }
                                else if (_cm.Vertices[e.pointB] != _cm.Vertices[_currentEdge.Start] && _cm.Vertices[e.pointB] != _cm.Vertices[_currentEdge.End])
                                {
                                    cornerEdgeOther2 = e.pointB;
                                }
                            }

                            triCenterTemp2 = (_cm.Vertices[cornerEdge2.pointA] + _cm.Vertices[cornerEdge2.pointB] + _cm.Vertices[cornerEdgeOther2]) / 3;
                            cornerEdgeNormalTemp2 = (triCenterTemp2 - Mathf2.NearestPointOnLine(_cm.Vertices[cornerEdge2.pointA], (_cm.Vertices[cornerEdge2.pointA] - _cm.Vertices[cornerEdge2.pointB]).normalized, triCenterTemp2)).normalized;

                            timesLooped = 0;

                            foreach (Edge e in _cm.EdgesAttachedToCorner[_cornerReached])
                            {
                                tempEdges.Add(e);
                                count++;
                            }

                            if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[cornerEdge1.pointA], _cm.Vertices[cornerEdge1.pointB]) == _cm.Vertices[_cornerReached] &&
                                Vector3.Dot(cornerEdgeNormalTemp1, _moveDirection) < 0)
                            {
                                tempEdges.Remove(cornerEdge1);
                                _index = _cm.EdgeAdjacencyInfo[cornerEdge1].triangleA;

                                if (_cm.Vertices[cornerEdge1.pointA] == _cm.Vertices[_cornerReached])
                                {
                                    _currentEdge.End = cornerEdge1.pointA;
                                    _currentEdge.Start = cornerEdge1.pointB;
                                }
                                else if (_cm.Vertices[cornerEdge1.pointB] == _cm.Vertices[_cornerReached])
                                {
                                    _currentEdge.End = cornerEdge1.pointB;
                                    _currentEdge.Start = cornerEdge1.pointA;
                                }


                                _currentEdge.Other = cornerEdgeOther1;
                            }
                            else if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[cornerEdge2.pointA], _cm.Vertices[cornerEdge2.pointB]) == _cm.Vertices[_cornerReached])
                            {
                                tempEdges.Remove(cornerEdge2);
                                _index = _cm.EdgeAdjacencyInfo[cornerEdge2].triangleA;

                                if (_cm.Vertices[cornerEdge2.pointA] == _cm.Vertices[_cornerReached])
                                {
                                    _currentEdge.End = cornerEdge2.pointA;
                                    _currentEdge.Start = cornerEdge2.pointB;
                                }
                                else if (_cm.Vertices[cornerEdge2.pointB] == _cm.Vertices[_cornerReached])
                                {
                                    _currentEdge.End = cornerEdge2.pointB;
                                    _currentEdge.Start = cornerEdge2.pointA;
                                }

                                _currentEdge.Other = cornerEdgeOther2;
                            }
                        }

                        bool foundNextEdge = false;
                        int x = 0;
                        int tempIndex = _index;
                        int tempLastIndex = _lastIndex;

                        List<Edge> checkedEdges = new List<Edge>();
                        List<Edge> checkedEdgesCorner = new List<Edge>();
                        // if we haven't found an outer edge

                        int cornerEdgeStart = -1;
                        int cornerEdgeEnd = -1;
                        int cornerEdgeOther = -1;
                        int cornerTriangleIndex = tempIndex;
                        while (cornerEdgeStart == -1)
                        {
                            foreach (Edge e in _cm.TriangleAdjacencyInfo[cornerTriangleIndex].edges)
                            {
                                // We need to find the other corner edge
                                bool tempEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in tempEdges)
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
                                    // Debug.DrawLine(cm.meshVerts[e.pointA], cm.meshVerts[e.pointB], Color.red);
                                    // and it's an outside edge
                                    if (_cm.EdgeAdjacencyInfo[e].triangleA == _cm.EdgeAdjacencyInfo[e].triangleB)
                                    {
                                        Debug.Log("A1");
                                        if (_cm.Vertices[e.pointA] == _cm.Vertices[_cornerReached])
                                        {
                                            Debug.Log("A1_1");
                                            cornerEdgeStart = e.pointA;
                                            cornerEdgeEnd = e.pointB;

                                        }
                                        else if (_cm.Vertices[e.pointB] == _cm.Vertices[_cornerReached])
                                        {
                                            Debug.Log("A1_2");
                                            cornerEdgeStart = e.pointB;
                                            cornerEdgeEnd = e.pointA;
                                        }
                                        foreach (Edge e2 in _cm.TriangleAdjacencyInfo[cornerTriangleIndex].edges)
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
                                        if (cornerTriangleIndex == _cm.EdgeAdjacencyInfo[e].triangleA)
                                        {
                                            cornerTriangleIndex = _cm.EdgeAdjacencyInfo[e].triangleB;
                                        }
                                        else
                                        {
                                            cornerTriangleIndex = _cm.EdgeAdjacencyInfo[e].triangleA;
                                        }
                                        foreach (Edge e2 in _cm.TriangleAdjacencyInfo[cornerTriangleIndex].edges)
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
                                Debug.Log("Couldn't find corner's end edge");
                                Debug.Log("cornerEdgeStart " + cornerEdgeStart);
                                Debug.Log("cornerEdgeEnd " + cornerEdgeEnd);
                                Debug.Log("cornerEdgeOther " + cornerEdgeOther);
                                Debug.Log("cornerTriangleIndex " + cornerTriangleIndex);
                                Debug.Log("cornerReached " + _cornerReached);
                                break;
                            }
                        }
                        while (!foundNextEdge)
                        {
                            // we need to use a different method for contains - checking by their positions, since hard edges has different indices

                            // scan the edges of the current triangle
                            foreach (Edge e in _cm.TriangleAdjacencyInfo[tempIndex].edges)
                            {

                                // we need to use a different method for contains - checking by their positions, since hard edges has different indices

                                bool tempEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in tempEdges)
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
                                    if (_cm.EdgeAdjacencyInfo[e].triangleA == _cm.EdgeAdjacencyInfo[e].triangleB)
                                    {
                                        foundNextEdge = true;
                                        _previousLastIndex = _lastIndex;
                                        _previousIndex = _index;

                                        _previousLastEdgeStart = _currentEdge.Start;
                                        _previousLastEdgeEnd = _currentEdge.End;
                                        _previousLastEdgeOther = _currentEdge.Other;

                                        _currentEdge.Start = e.pointA;
                                        _currentEdge.End = e.pointB;

                                        _lastIndex = tempLastIndex;
                                        _index = tempIndex;

                                        foreach (Edge p in _cm.TriangleAdjacencyInfo[_index].edges)
                                        {
                                            if (_cm.Vertices[p.pointA] != _cm.Vertices[_currentEdge.Start] && _cm.Vertices[p.pointA] != _cm.Vertices[_currentEdge.End])
                                            {
                                                _currentEdge.Other = p.pointA;
                                            }
                                            else if (_cm.Vertices[p.pointB] != _cm.Vertices[_currentEdge.Start] && _cm.Vertices[p.pointB] != _cm.Vertices[_currentEdge.End])
                                            {
                                                _currentEdge.Other = p.pointB;
                                            }
                                        }

                                        triCenter = (_cm.Vertices[_currentEdge.Start] + _cm.Vertices[_currentEdge.End] + _cm.Vertices[_currentEdge.Other]) / 3;
                                        closestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[_currentEdge.Start], (_cm.Vertices[_currentEdge.Start] - _cm.Vertices[_currentEdge.End]).normalized, triCenter);

                                        Vector3 previousTriCenter = (_cm.Vertices[_previousLastEdgeStart] + _cm.Vertices[_previousLastEdgeEnd] + _cm.Vertices[_previousLastEdgeOther]) / 3;
                                        Vector3 previousClosestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[_previousLastEdgeStart], (_cm.Vertices[_previousLastEdgeStart] - _cm.Vertices[_previousLastEdgeEnd]).normalized, previousTriCenter);

                                        _newPosition = slidePoint;

                                        Vector3 cornerNormal = Vector3.Cross(_cm.Vertices[_previousLastEdgeStart] - _cm.Vertices[_previousLastEdgeEnd], _cm.Vertices[_currentEdge.Start] - _cm.Vertices[_currentEdge.End]).normalized;
                                        if (Vector3.Dot(cornerNormal, transform.up) < 0)
                                        {
                                            cornerNormal = -cornerNormal;
                                        }
                                        Vector3 newEdgeNormal = (closestPointOnEdge - triCenter).normalized;
                                        Vector3 lastEdgeNormal = (previousClosestPointOnEdge - previousTriCenter).normalized;

                                        Vector3 newFarCorner = _cm.Vertices[currentCornerInt] == _cm.Vertices[_currentEdge.Start] ? _cm.Vertices[_currentEdge.End] : _cm.Vertices[_currentEdge.Start];
                                        Vector3 lastFarCorner = _cm.Vertices[currentCornerInt] == _cm.Vertices[_previousLastEdgeStart] ? _cm.Vertices[_previousLastEdgeEnd] : _cm.Vertices[_previousLastEdgeStart];

                                        int shouldInvertNewEdgeNormal = 1;
                                        Vector3 cornerDirection = Vector3.ProjectOnPlane(transform.rotation * _input, cornerNormal).normalized;

                                        Vector3 newEdgeNormal2 = Vector3.ProjectOnPlane(newEdgeNormal * shouldInvertNewEdgeNormal, cornerNormal).normalized;
                                        Vector3 lastEdgeNormal2 = Vector3.ProjectOnPlane(lastEdgeNormal, cornerNormal).normalized;
                                        // Debug.DrawLine(closestPointOnEdge, closestPointOnEdge + newEdgeNormal2, Color.yellow);
                                        // Debug.DrawLine(previousClosestPointOnEdge, previousClosestPointOnEdge + lastEdgeNormal2, Color.yellow);
                                        // Debug.DrawLine(transform.position, transform.position + cornerNormal, Color.red);
                                        if (Vector3.Dot((transform.rotation * _input).normalized, (closestPointOnEdge - triCenter).normalized) < 0)
                                        {
                                            _onEdge = false;
                                            foundNextEdge = true;
                                            Debug.Log("Found Edge");
                                        }

                                        Vector3 slidePoint2 = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[_previousLastEdgeStart], _cm.Vertices[_previousLastEdgeEnd]);
                                        Vector3 slidePoint2Clamped = Mathf2.NearestPointOnLine(_cm.Vertices[_previousLastEdgeStart], _cm.Vertices[_previousLastEdgeEnd], movePositionAttempt);
                                        Vector3 slidePoint3 = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End]);
                                        Vector3 slidePoint3Clamped = Mathf2.NearestPointOnLine(_cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], movePositionAttempt);
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
                                            _onEdge = true;
                                            _currentEdge.Start = _previousLastEdgeStart;
                                            _currentEdge.End = _previousLastEdgeEnd;
                                            _currentEdge.Other = _previousLastEdgeOther;

                                            _index = _previousIndex;

                                            _lastIndex = _previousLastIndex;
                                            totalDistanceChecked = distance;
                                            _newPosition = slidePoint;
                                            slidePoint = _cm.Vertices[_cornerReached];
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
                                        if (tempIndex == _cm.EdgeAdjacencyInfo[e].triangleA)
                                        {
                                            tempIndex = _cm.EdgeAdjacencyInfo[e].triangleB;
                                        }
                                        else
                                        {
                                            tempIndex = _cm.EdgeAdjacencyInfo[e].triangleA;
                                        }
                                        checkedEdges.Add(e);
                                        if (_cm.Vertices[e.pointA] == _cm.Vertices[_cornerReached])
                                        {
                                            nextTriLastEdgeStart = e.pointA;
                                            nextTriLastEdgeEnd = e.pointB;
                                        }
                                        else
                                        {
                                            nextTriLastEdgeStart = e.pointB;
                                            nextTriLastEdgeEnd = e.pointA;
                                        }
                                        foreach (Edge e2 in _cm.TriangleAdjacencyInfo[tempIndex].edges)
                                        {
                                            if (_cm.Vertices[e2.pointA] != _cm.Vertices[nextTriLastEdgeStart] && _cm.Vertices[e2.pointA] != _cm.Vertices[nextTriLastEdgeEnd])
                                            {
                                                nextTriLastEdgeOther = e2.pointA;
                                            }
                                            if (_cm.Vertices[e2.pointB] != _cm.Vertices[nextTriLastEdgeStart] && _cm.Vertices[e2.pointB] != _cm.Vertices[nextTriLastEdgeEnd])
                                            {
                                                nextTriLastEdgeOther = e2.pointB;
                                            }
                                        }

                                        // check if we are pointing towards the other edge (not the next edge attached to corner)
                                        // if so, we can come unstuck.

                                        if (GetFarEdgeCut(_cornerReached, tempIndex, nextTriLastEdgeStart, nextTriLastEdgeEnd, nextTriLastEdgeOther, cornerEdgeStart, cornerEdgeEnd, cornerEdgeOther, _plane))
                                        {
                                            // Debug.DrawLine("BUH");
                                            _index = tempIndex;
                                            _lastIndex = tempLastIndex;
                                            _currentEdge.Start = nextTriLastEdgeEnd;
                                            _currentEdge.End = nextTriLastEdgeStart;
                                            _currentEdge.Other = nextTriLastEdgeOther;
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
                                            _onEdge = false;

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
                    _newPosition = slidePoint;
                    // Debug.Log(newPosition);
                }
                else
                {
                    totalDistanceChecked = distance;
                    // Debug.Log(newPosition);
                    _newPosition = slidePoint;
                    // Debug.Log(newPosition);
                }

                remainingDistance = remainingDistance - Vector3.Distance(_newPosition, slidePoint);

                _plane = new Plane(transform.rotation * Quaternion.Euler(0, 90, 0) * _input, _newPosition);

                i++;
                if (i > 1000)
                {
                    Debug.Log("WHOOPS " + _index);
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

        _barycentricCoordinate = Mathf2.GetBarycentricCoordinates(_newPosition, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);

        Vector3 updatedPosition = _barycentricCoordinate.x * _cm.Vertices[_currentEdge.Start] +
                                _barycentricCoordinate.y * _cm.Vertices[_currentEdge.End] +
                                _barycentricCoordinate.z * _cm.Vertices[_currentEdge.Other];



        triCenter = (_cm.Vertices[_currentEdge.Start] + _cm.Vertices[_currentEdge.End] + _cm.Vertices[_currentEdge.Other]) / 3;

        Vector3 testPosition = _barycentricCoordinate.x * _cm.Vertices[_currentEdge.Start] +
                    _barycentricCoordinate.y * _cm.Vertices[_currentEdge.End] +
                    _barycentricCoordinate.z * _cm.Vertices[_currentEdge.Other];

        foreach (Edge e in _cm.TriangleAdjacencyInfo[_index].edges)
        {
            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.Start])
                currentEdgeReal.Start = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.Start])
                currentEdgeReal.Start = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.End])
                currentEdgeReal.End = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.End])
                currentEdgeReal.End = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.Other])
                currentEdgeReal.Other = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.Other])
                currentEdgeReal.Other = e.pointB;
        }

        groundNormal = GetNormalFromBarycentric(_barycentricCoordinate, currentEdgeReal);

        _castDirectionTest = Quaternion.FromToRotation(transform.up, groundNormal) * transform.forward;

        Vector3 v = Vector3.Cross(_castDirectionTest, groundNormal).normalized;

        if (_movementMode == MovementMode.Car)
        {
            _testPlane = new Plane(-(Quaternion.FromToRotation(transform.up, groundNormal) * transform.right), triCenter);
        }
        else
        {
            _testPlane = new Plane(-(Quaternion.FromToRotation(transform.up, groundNormal) * transform.right), triCenter);
            if (isFinalPass)
                _testPlane = new Plane(-CharacterModel.right, triCenter);
            // Debug.DrawLine(ch)
        }
        _testCut = Vector3.zero;
        _testCut = FindTrianglePlaneIntersection(_currentEdge, _castDirectionTest, triCenter, _testPlane, CutType.Test);

        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        _lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);
        _testCut = FindTrianglePlaneIntersection(_currentEdge, -_castDirectionTest, triCenter, _testPlane, CutType.Test);
        _barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);
        //    Debug.DrawLine(transform.position, transform.position + plane.normal, Color.blue);


        if (getNewtempForwardDirection != Vector3.zero)
        {
            _lastForward = getNewtempForwardDirection;
        }
        else
        {
            _lastForward = CharacterModel.forward;
        }

        // Debug.DrawLine(CharacterModel.position, CharacterModel.position + CharacterModel.forward, Color.red);
    }

    bool IsTriAfterNextThis(int lastEdgeStartReal, int lastEdgeEndReal, int lastEdgeOtherReal)
    {
        EdgePoints tempCurrentEdgeReal = new();
        tempCurrentEdgeReal.Set(-1, -1, -1);

        Vector3 tempBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(_newPosition, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);

        Vector3 tempTriCenter = (_cm.Vertices[_currentEdge.Start] + _cm.Vertices[_currentEdge.End] + _cm.Vertices[_currentEdge.Other]) / 3;

        foreach (Edge e in _cm.TriangleAdjacencyInfo[_index].edges)
        {
            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.Start])
                tempCurrentEdgeReal.Start = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.Start])
                tempCurrentEdgeReal.Start = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.End])
                tempCurrentEdgeReal.End = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.End])
                tempCurrentEdgeReal.End = e.pointB;

            if (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.Other])
                tempCurrentEdgeReal.Other = e.pointA;
            else if (_cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.Other])
                tempCurrentEdgeReal.Other = e.pointB;
        }

        Vector3 tempGroundNormal = GetNormalFromBarycentric(tempBarycentricCoordinate, tempCurrentEdgeReal);

        Vector3 tempCastDirectionTest = Quaternion.FromToRotation(transform.up, tempGroundNormal) * transform.forward;

        Plane tempTestPlane = new Plane(-(Quaternion.FromToRotation(transform.up, tempGroundNormal) * transform.right), tempTriCenter);

        Vector3 tempTestCut = Vector3.zero;
        tempTestCut = FindTrianglePlaneIntersection(_currentEdge, tempCastDirectionTest, tempTriCenter, tempTestPlane, CutType.Test);

        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        Vector3 tempLastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(tempTestCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);
        tempTestCut = FindTrianglePlaneIntersection(_currentEdge, -tempCastDirectionTest, tempTriCenter, tempTestPlane, CutType.Test);
        Vector3 tempBarycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(tempTestCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);
        Vector3 behindPointOnTriangle = GetPositionFromBarycentric(tempBarycentricCoordinateBehind, _currentEdge);
        // At the end of last loop we do a 'test' cut to get the next position in FRONT
        // Here we recalculate it with deformations. This is the forward cut, NOT the movement direction cut 
        // This is also recorded from the tri center
        Vector3 forwardPointOnTriangle = GetPositionFromBarycentric(tempLastBarycentricCoordinate, _currentEdge);

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        Vector3 tempForward = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        Quaternion tempRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(tempForward, tempGroundNormal), tempGroundNormal); // set rotation to be towards the forward point.

        Vector3 tempMovePositionAttempt = _cm.Vertices[_cornerReached] + (tempRotation * _input).normalized;

        Vector3 slidePoint = Mathf2.GetClosestPointOnFiniteLine(tempMovePositionAttempt, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End]);
        // This is for stopping the check from bouncing between edges when stuck in a corner
        // Problem is that it's sticking to corners when the GetClosestPointOnLine attemptedMovePosition is outside the line

        // In order to determine if we should be able to move round the corner, we need to know if it's a convex corner or not
        // DoRaysIntersect of the edge normals determines this.
        // If they do, then we check if the closest point on each edge to the attempted move position is equal to the corner - that's the only situation where we should move round the corner  

        if (slidePoint == _cm.Vertices[_cornerReached])
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
        Vector3 triCenter = (_cm.Vertices[cornerEdgeStart] + _cm.Vertices[cornerEdgeEnd] + _cm.Vertices[cornerEdgeOther]) / 3;
        Vector3 closestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[cornerEdgeStart], (_cm.Vertices[cornerEdgeStart] - _cm.Vertices[cornerEdgeEnd]).normalized, triCenter);
        Vector3 nextTriCenter = (_cm.Vertices[nextTriLastEdgeStart] + _cm.Vertices[nextTriLastEdgeEnd] + _cm.Vertices[nextTriLastEdgeOther]) / 3;
        Vector3 nextTriClosestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[nextTriLastEdgeStart], (_cm.Vertices[nextTriLastEdgeStart] - _cm.Vertices[nextTriLastEdgeEnd]).normalized, nextTriCenter);

        Vector3 previousTriCenter = (_cm.Vertices[_currentEdge.Start] + _cm.Vertices[_currentEdge.End] + _cm.Vertices[_currentEdge.Other]) / 3;

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
        Vector3 cornerAdjustedMoveDirection = Vector3.ProjectOnPlane((transform.rotation * _input).normalized, cornerNormal).normalized;
        if (Vector3.Dot((transform.rotation * _input).normalized, nextTriEdgeNormal2) < 0 && Vector3.Dot(cornerAdjustedMoveDirection, edgeNormal2) < 0)
        {
            foreach (Edge e in _cm.TriangleAdjacencyInfo[triangleIndex].edges)
            {
                if (_cm.Vertices[e.pointA] != _cm.Vertices[currentCornerInt] && _cm.Vertices[e.pointB] != _cm.Vertices[currentCornerInt])
                {
                    Ray farEdgeRay = CreateRay(_cm.Vertices[e.pointA], _cm.Vertices[e.pointB]);
                    if (plane.Raycast(farEdgeRay, out var farEdgeHitDistance))
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

    Vector3 GetAdjustedNormal(int normalIndex)
    {
        return _cm.transform.TransformDirection(_cm.Normals[normalIndex]);
    }

    ////////////////
    // REFACTOR Start //
    ////////////////

    /// <summary>
    /// Finds the next intersection point between a plane and a triangle's edges as part of a cutting operation.
    /// </summary>
    /// <param name="p1">First vertex index of the triangle.</param>
    /// <param name="p2">Second vertex index of the triangle.</param>
    /// <param name="p3">Third vertex index of the triangle.</param>
    /// <param name="direction">Direction vector used to validate intersection points.</param>
    /// <param name="position">Current position for directional checks.</param>
    /// <param name="plane">Plane to intersect with the triangle's edges.</param>
    /// <param name="cutType">Type of cut operation: Start (initial cut), Next (subsequent cut), or Test (check without state update).</param>
    /// <returns>The intersection point if found; otherwise, the current position.</returns>
    Vector3 FindTrianglePlaneIntersection(EdgePoints edgePoints, Vector3 direction, Vector3 position, Plane plane, CutType cutType)
    {
        // Update tracking of the previous edge when moving to a new triangle (except in Test or Start modes)
        if (_index != _lastIndex && cutType != CutType.Test && cutType != CutType.Start)
        {
            UpdateEdgeIndices(edgePoints.Start, edgePoints.End, edgePoints.Other);
        }

        // Reorder vertices for CutType.Next to ensure continuity with the previous edge
        (int vertexA, int vertexB, int vertexC) = OrderVerticesForCut(edgePoints.Start, edgePoints.End, edgePoints.Other, cutType);

        // Define the triangle's edges: each tuple represents (start vertex, end vertex, opposite vertex)
        var edges = new[]
        {
        (start: vertexA, end: vertexB, other: vertexC), // Edge A -> B, opposite C
        (start: vertexB, end: vertexC, other: vertexA), // Edge B -> C, opposite A
        (start: vertexA, end: vertexC, other: vertexB)  // Edge A -> C, opposite B
    };

        // Precompute ray data for each edge (ray, length, and plane intersection distance)
        var edgeRayData = ComputeRayDataForEdges(edges, plane);

        // Select edges to process: only the first two for Next (new edges), all three otherwise
        int edgesToProcessCount = cutType == CutType.Next ? 2 : 3;

        Vector3 result = position; // Default return value if no intersection is found
        bool hasIntersection = false;

        // In GetNextCut method, replace the loop inside the if (firstMoveDone) block:
        if (_firstMoveDone)
        {
            for (int i = 0; i < edgesToProcessCount; i++)
            {
                ProcessEdgeIntersection(edges[i], edgeRayData[i], edges, edgeRayData,
                                       direction, position, cutType,
                                       ref result, ref hasIntersection, isFirstEdge: i == 0);
            }
        }

        _firstMoveDone = true; // Mark that the first move has occurred for subsequent calls
        return result;
    }

    /// <summary>
    /// Orders triangle vertices based on the cut type, aligning with the previous edge for CutType.Next.
    /// </summary>
    private (int vertexA, int vertexB, int vertexC) OrderVerticesForCut(int p1, int p2, int p3, CutType cutType)
    {
        if (cutType == CutType.Next && _lastIndex != _index)
        {
            int vertexA = MatchVertex(_currentEdge.Start, p1, p2, p3); // Start matches previous edge start
            int vertexC = MatchVertex(_currentEdge.End, p1, p2, p3);   // End matches previous edge end
            int vertexB = (p1 != vertexA && p1 != vertexC) ? p1 :
                          (p2 != vertexA && p2 != vertexC) ? p2 : p3; // Middle is the remaining vertex
            return (vertexA, vertexB, vertexC);
        }
        return (p1, p2, p3); // Default order for Start or Test
    }

    /// <summary>
    /// Computes ray data (ray, length, intersection distance) for each edge against the plane.
    /// </summary>
    private (Ray ray, float length, float hitDistance)[] ComputeRayDataForEdges((int start, int end, int other)[] edges, Plane plane)
    {
        var rayData = new (Ray ray, float length, float hitDistance)[3];
        for (int i = 0; i < 3; i++)
        {
            Vector3 startPos = _cm.Vertices[edges[i].start];
            Vector3 endPos = _cm.Vertices[edges[i].end];
            Vector3 rayDirection = (endPos - startPos).normalized;
            Ray ray = new Ray(startPos, rayDirection);
            float edgeLength = Vector3.Distance(startPos, endPos);
            plane.Raycast(ray, out float hitDistance);
            rayData[i] = (ray, edgeLength, hitDistance);
        }
        return rayData;
    }

    /// <summary>
    /// Processes an edge to find and validate an intersection with the plane, updating the result if accepted.
    /// </summary>
    private void ProcessEdgeIntersection((int start, int end, int other) edge,
                                        (Ray ray, float length, float hitDistance) rayData,
                                        (int start, int end, int other)[] edges, // Added parameter
                                        (Ray ray, float length, float hitDistance)[] edgeRayData, // Added parameter
                                        Vector3 direction, Vector3 position, CutType cutType,
                                        ref Vector3 result, ref bool hasIntersection, bool isFirstEdge)
    {
        var (ray, edgeLength, hitDistance) = rayData;

        // Case 1: Ray intersects the plane beyond the starting point
        if (hitDistance > 0)
        {
            int edgeIndex = Array.IndexOf(edges, edge); // Now uses the passed 'edges' parameter
            bool isValid = IsIntersectionValid(hitDistance, edgeLength, edgeRayData, edgeIndex); // Now uses the passed 'edgeRayData'
            if (isValid)
            {
                Vector3 intersectionPoint = ray.GetPoint(hitDistance);
                if (ShouldAcceptIntersection(cutType, direction, position, intersectionPoint))
                {
                    result = intersectionPoint;
                    UpdateEdgeState(edge, cutType, ref hasIntersection, isFirstEdge);
                }
            }
        }
        // Case 2: Ray starts on the plane (hitDistance = 0), use start vertex under specific conditions
        else if (hitDistance == 0 && cutType != CutType.Test && _lastIndex != _index &&
                 (isFirstEdge || !hasIntersection))
        {
            result = _cm.Vertices[edge.start];
            UpdateEdgeState(edge, cutType, ref hasIntersection, isFirstEdge);
        }
    }

    /// <summary>
    /// Updates the last edge state variables if not in Test mode.
    /// </summary>
    private void UpdateEdgeState((int start, int end, int other) edge, CutType cutType,
                                ref bool hasIntersection, bool isFirstEdge)
    {
        if (cutType != CutType.Test)
        {
            _currentEdge.Start = edge.start;
            _currentEdge.End = edge.end;
            _currentEdge.Other = edge.other;
            if (!isFirstEdge) hasIntersection = true;
        }
    }

    /// <summary>
    /// Updates last edge indices to match the current triangle's vertices.
    /// </summary>
    private void UpdateEdgeIndices(int p1, int p2, int p3)
    {
        _currentEdge.Start = MatchVertex(_currentEdge.Start, p1, p2, p3);
        _currentEdge.End = MatchVertex(_currentEdge.End, p1, p2, p3);
        _currentEdge.Other = MatchVertex(_currentEdge.Other, p1, p2, p3);
    }

    /// <summary>
    /// Finds the vertex index among p1, p2, p3 that matches the target vertex by position.
    /// </summary>
    private int MatchVertex(int target, int p1, int p2, int p3)
    {
        if (_cm.Vertices[p1] == _cm.Vertices[target]) return p1;
        if (_cm.Vertices[p2] == _cm.Vertices[target]) return p2;
        if (_cm.Vertices[p3] == _cm.Vertices[target]) return p3;
        return p1; // Fallback to p1 if no match (assumes a match should exist)
    }

    /// <summary>
    /// Validates an intersection, including extended conditions beyond the edge length.
    /// </summary>
    private bool IsIntersectionValid(float hitDistance, float rayLength,
                                    (Ray ray, float length, float hitDistance)[] rayData, int index)
    {
        if (hitDistance <= rayLength) return true;

        int i1 = (index + 1) % 3;
        int i2 = (index + 2) % 3;
        float hit1 = rayData[i1].hitDistance;
        float len1 = rayData[i1].length;
        float hit2 = rayData[i2].hitDistance;
        float len2 = rayData[i2].length;

        return (hitDistance > rayLength && hit1 > len1 && hit2 < len2 && hit2 > 0) ||
               (hitDistance > rayLength && hit2 > len2 && hit1 < len1 && hit1 > 0);
    }

    /// <summary>
    /// Determines if an intersection point should be accepted based on cut type and direction.
    /// </summary>
    private bool ShouldAcceptIntersection(CutType cutType, Vector3 direction,
                                         Vector3 position, Vector3 intersectionPoint)
    {
        if (cutType == CutType.Next) return true;
        if (cutType == CutType.Start || cutType == CutType.Test)
        {
            // Accept if the intersection is not in the forward direction from the position
            return Vector3.Dot(direction.normalized, (position - intersectionPoint).normalized) <= 0;
        }
        return false;
    }

    //////////////
    // REFACTOR END //
    //////////////

    Ray CreateRay(Vector3 point1, Vector3 point2) => new Ray { origin = point1, direction = (point2 - point1).normalized };

    int[] GetNextTri()
    {
        // Get the three adjacent edges of the triangle we're currently checking
        Edge[] adjacentEdges = _cm.TriangleAdjacencyInfo[_index].edges;

        foreach (Edge e in adjacentEdges)
        {
            // Debug.Log(e.triangleA);
            // Debug.Log(e.triangleB);
            // Get the edge of the three that is equal to the edge that has just been passed
            bool edgeMatches = (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.Start] && _cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.End]) ||
                (_cm.Vertices[e.pointA] == _cm.Vertices[_currentEdge.End] && _cm.Vertices[e.pointB] == _cm.Vertices[_currentEdge.Start]);

            if (edgeMatches)
            {
                int nextTriangle = _cm.EdgeAdjacencyInfo[e].triangleA == _cm.EdgeAdjacencyInfo[e].triangleB
                    ? _cm.EdgeAdjacencyInfo[e].triangleA // If adjacent triangles are the same, return the same triangle
                    : _cm.EdgeAdjacencyInfo[e].triangleA != _index
                        ? _cm.EdgeAdjacencyInfo[e].triangleA // If edge has two triangles and the first is not equal to the current triangle, return that
                        : _cm.EdgeAdjacencyInfo[e].triangleB; // Otherwise, return the second, that's all that's left

                // Set index to the new triangle
                _index = nextTriangle;

                return new int[3]
                {
                    _cm.Triangles[nextTriangle],
                    _cm.Triangles[nextTriangle + 1],
                    _cm.Triangles[nextTriangle + 2]
                };
            }
        }

        return default;
    }

    void TryStartClimb()
    {
        _firstMoveDone = false;
        RaycastHit hit;
        if (Physics.Raycast(transform.position + transform.up * 0.1f, -transform.up, out hit, 0.2f, _layerMask)
        || Physics.Raycast(_previousRayCastPosition, (_previousRayCastPosition - transform.position).normalized, out hit, Vector3.Distance(_previousRayCastPosition, transform.position), _layerMask))
        {
            GameObject temp = hit.collider.gameObject;
            _cm = temp.GetComponent<ClimbableMesh>();

            _isClimbing = true;
            transform.position = hit.point;
            _index = hit.triangleIndex * 3;
            Debug.Log(hit.triangleIndex);
            Debug.Log(_index);
            _barycentricCoordinate = hit.barycentricCoordinate;

            _currentEdge.Start = _cm.Triangles[_index];
            _currentEdge.End = _cm.Triangles[_index + 1];
            _currentEdge.Other = _cm.Triangles[_index + 2];

            Vector3 triCenter = (_cm.Vertices[_currentEdge.Start] + _cm.Vertices[_currentEdge.End] + _cm.Vertices[_currentEdge.Other]) / 3;

            Plane plane = new Plane(-transform.right, triCenter);

            _testCut = FindTrianglePlaneIntersection(_currentEdge, transform.forward, triCenter, plane, CutType.Start);
            _lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);
            _testCut = FindTrianglePlaneIntersection(_currentEdge, -transform.forward, triCenter, plane, CutType.Test);
            _barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdge.Start], _cm.Vertices[_currentEdge.End], _cm.Vertices[_currentEdge.Other]);
        }
        _previousRayCastPosition = transform.position + transform.up * 0.1f;
    }

    void LeaveClimbableMesh()
    {
        // MYSTERY
        transform.position = CharacterModel.position;
        _rb.position = CharacterModel.position;
        _rb.GetComponent<KinematicCharacterMotor>().SetPositionAndRotation(CharacterModel.position, CharacterModel.rotation);
        _isClimbing = false;
    }

    Vector3 Tr_(Vector3 v)
    {
        return _cm.transform.TransformPoint(v);
    }

    Vector3 GetPositionFromBarycentric(Vector3 barycentricCoordinate, EdgePoints edgePoints)
    {
        return barycentricCoordinate.x * _cm.Vertices[edgePoints.Start] +
               barycentricCoordinate.y * _cm.Vertices[edgePoints.End] +
               barycentricCoordinate.z * _cm.Vertices[edgePoints.Other];
    }
    Vector3 GetNormalFromBarycentric(Vector3 barycentricCoordinate, EdgePoints edgePoints)
    {
        return (barycentricCoordinate.x * _cm.Normals[edgePoints.Start] +
                barycentricCoordinate.y * _cm.Normals[edgePoints.End] +
                barycentricCoordinate.z * _cm.Normals[edgePoints.Other]).normalized;
    }

    void DebugTriangle(int triangle, Color color)
    {
        if (_cm.EdgeAdjacencyInfo[_cm.TriangleAdjacencyInfo[triangle].edges[0]].triangleB != -1)
            Debug.DrawLine(_cm.Vertices[_cm.TriangleAdjacencyInfo[triangle].edges[0].pointA], _cm.Vertices[_cm.TriangleAdjacencyInfo[triangle].edges[0].pointB], color, 0);

        if (_cm.EdgeAdjacencyInfo[_cm.TriangleAdjacencyInfo[triangle].edges[1]].triangleB != -1)
            Debug.DrawLine(_cm.Vertices[_cm.TriangleAdjacencyInfo[triangle].edges[1].pointA], _cm.Vertices[_cm.TriangleAdjacencyInfo[triangle].edges[1].pointB], color, 0);

        if (_cm.EdgeAdjacencyInfo[_cm.TriangleAdjacencyInfo[triangle].edges[2]].triangleB != -1)
            Debug.DrawLine(_cm.Vertices[_cm.TriangleAdjacencyInfo[triangle].edges[2].pointA], _cm.Vertices[_cm.TriangleAdjacencyInfo[triangle].edges[2].pointB], color, 0);
    }

    void OnGUI()
    {
        // Display the frame rate on the screen
        GUIStyle style = new GUIStyle();
        style.normal.textColor = Color.white;
        style.fontSize = 20;
        Rect rect = new Rect(10, 10, 200, 30);
        GUI.Label(rect, "Frame Rate: " + _frameRate.ToString("F2"), style);
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
    public static void SetEdges(this EdgePoints edgePoints, int start, int end, int other)
    {
        edgePoints.Start = start;
        edgePoints.End = end;
        edgePoints.Other = other;
    }
}