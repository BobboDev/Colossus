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

    EdgePoints _currentEdgePoints;
    EdgePoints _lastEdgePoints;

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
    Vector3 _forwardLastFrame;

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
            ClimbUtils.TryStartClimb(ref _cm, transform, ref _currentEdgePoints, ref _index, ref _lastIndex, ref _firstMoveDone, ref _testCut, ref _barycentricCoordinate, ref _lastBarycentricCoordinate, ref _barycentricCoordinateBehind, ref _previousRayCastPosition, _layerMask, ref _isClimbing);
        }
        else
        {
            if (Input.GetKey(KeyCode.Space))
            {
                ClimbUtils.LeaveClimbableMesh(transform, CharacterModel, _rb, ref _isClimbing);
                return;
            }

            _cm.RecalculateMesh(false);

            // Get input
            Vector3 tempInput;
            tempInput = new Vector3(Input.GetAxisRaw("Horizontal"), 0, Input.GetAxisRaw("Vertical")).normalized;

            // This is for debugging - makes the character go forwards and backwards over an edge rapidly, so I don't have to hammer the keyboard to test. 
            if (Input.GetKeyDown(KeyCode.K))
                _goForwardTest = !_goForwardTest;

            if (_goForwardTest)
                tempInput = new Vector3(0, 0, 1);

            // if input has magnitude then use it
            if (tempInput.magnitude > 0)
            {
                _input = tempInput;
            }

            // calculate movement on the mesh based on input
            Travel(directionInput: tempInput, depenetratePass: false, isFinalPass: false);
            Vector3 pointOnTriangle = EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinate, _currentEdgePoints);
            transform.position = pointOnTriangle;


            int depenetrationIterations = 1;
            for (int i = 0; i < depenetrationIterations; i++)
            {
                bool isFinalPass = i == depenetrationIterations - 1;
                Depenetrate(isFinalPass);
                Physics.SyncTransforms();
            }
            // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
            pointOnTriangle = EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinate, _currentEdgePoints);
            transform.position = pointOnTriangle;
        }

    }

    void Depenetrate(bool isFinalPass)
    {
        Travel(Vector3.zero, true, isFinalPass);
    }

    void Travel(Vector3 directionInput, bool depenetratePass, bool isFinalPass)
    {
        // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
        Vector3 pointOnTriangle = EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinate, _currentEdgePoints);
        // At the end of last loop we do a 'test' cut to get the next position BEHIND
        // This is recorded from the tri center
        Vector3 behindPointOnTriangle = EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinateBehind, _currentEdgePoints);
        // At the end of last loop we do a 'test' cut to get the next position in FRONT
        // Here we recalculate it with deformations. This is the forward cut, NOT the movement direction cut 
        // This is also recorded from the tri center
        Vector3 forwardPointOnTriangle = EdgeUtils.GetPositionFromBarycentric(_cm, _lastBarycentricCoordinate, _currentEdgePoints);

        transform.position = pointOnTriangle;

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        Vector3 forwardFromRecordedBarycentric = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        if (Input.GetKey(KeyCode.LeftShift))
        {
            ClimbUtils.HandleMovementModeSwitch(_cm, transform, ref _currentEdgePoints, ref _index, ref _lastIndex, ref _firstMoveDone, ref _testCut, ref _barycentricCoordinate, ref _lastBarycentricCoordinate, ref _barycentricCoordinateBehind, MovementMode.Car, ref _movementMode, ref _newPosition, ref _onEdge);
        }
        else
        {
            ClimbUtils.HandleMovementModeSwitch(_cm, transform, ref _currentEdgePoints, ref _index, ref _lastIndex, ref _firstMoveDone, ref _testCut, ref _barycentricCoordinate, ref _lastBarycentricCoordinate, ref _barycentricCoordinateBehind, MovementMode.Directional, ref _movementMode, ref _newPosition, ref _onEdge);
        }

        // Because of hard edges, the current triangle will have different vertices to what's currently stores in lastEdgeStart, lastEdgeEnd, lastEdgeOther
        // We need to check this triangle's edge positions against those and update.
        // lastEdgeStartReal,lastEdgeEndReal, lastEdgeOtherReal are the real indices of the current triangle
        EdgeUtils.GetMatchingEdgeOnAdjacentTriangle(_cm, out var edgeAdjacentToCurrent, _currentEdgePoints, _index);

        // Calculate the ground normal based on coordinates from the last frame of where we should be standing, translated to the new, deformed triangle
        Vector3 groundNormal = EdgeUtils.GetNormalFromBarycentric(_cm, _barycentricCoordinate, edgeAdjacentToCurrent);

        float turnAngle = 0;

        turnAngle += Input.GetKey(KeyCode.E) ? 100 * Time.deltaTime : 0;
        turnAngle -= Input.GetKey(KeyCode.Q) ? 100 * Time.deltaTime : 0;
        forwardFromRecordedBarycentric = Quaternion.Euler(
            Vector3.Dot(groundNormal, Vector3.right) * turnAngle,
            Vector3.Dot(groundNormal, Vector3.up) * turnAngle,
            Vector3.Dot(groundNormal, Vector3.forward) * turnAngle)

            * forwardFromRecordedBarycentric;

        if (_movementMode == MovementMode.Directional)
        {
            if (directionInput.magnitude > 0)
            {
                forwardFromRecordedBarycentric = _forwardLastFrame;
            }
        }

        // In case of "viewing vector is equal to zero" error
        if (forwardFromRecordedBarycentric != Vector3.zero)
        {
            transform.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(forwardFromRecordedBarycentric, groundNormal), groundNormal); // set rotation to be towards the forward point.
        }
        else
        {
            Debug.Log("forward zero");
        }


        // Get the direction towards the edge 
        Vector3 triCenter = (_cm.Vertices[_currentEdgePoints.Start] + _cm.Vertices[_currentEdgePoints.End] + _cm.Vertices[_currentEdgePoints.Other]) / 3;
        Vector3 closestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[_currentEdgePoints.Start], (_cm.Vertices[_currentEdgePoints.Start] - _cm.Vertices[_currentEdgePoints.End]).normalized, triCenter);
        // Vector3 getNewtempForwardDirection = Vector3.zero;
        // Vector3 getNewtempForwardDirectionBehind = Vector3.zero;

        if (_movementMode == MovementMode.Directional)
        {
            // This check works as intended
            if (directionInput.magnitude > 0)
            {
                float angleToRotateby = Camera.main.transform.rotation.eulerAngles.y - transform.rotation.eulerAngles.y;
                _input = Quaternion.Euler(0, angleToRotateby, 0) * _input;

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
                Vector3 cutDirection = Mathf2.RotateAroundAxis(NewtempForwardPlane.normal, groundNormal, 90);
                // Debug.DrawLine(transform.position, transform.position + PlaneNormalRight, Color.blue);
                forwardPointOnTriangle = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _index, _lastIndex, ref _firstMoveDone, cutDirection, transform.position, NewtempForwardPlane, CutType.Test);
                behindPointOnTriangle = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _index, _lastIndex, ref _firstMoveDone, -cutDirection, transform.position, NewtempForwardPlane, CutType.Test);
                // Debug.DrawLine(getNewtempForwardDirection, getNewtempForwardDirection + Vector3.up, Color.black);
                // Debug.DrawLine(getNewtempForwardDirectionBehind, getNewtempForwardDirectionBehind + Vector3.up, Color.black);
                // Debug.DrawLine(transform.position, transform.position + tempForward, Color.magenta);

                forwardFromRecordedBarycentric = (forwardPointOnTriangle - behindPointOnTriangle).normalized;
                // Debug.DrawLine(transform.position, transform.position + tempForward, Color.magenta);
            }
            else
            {
                if (!depenetratePass)
                {
                    _afterDepenetrateRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(forwardFromRecordedBarycentric, Vector3.up).normalized, Vector3.up);

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
        float distance = directionInput.magnitude * _targetSpeed * Time.deltaTime;

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
                forwardFromRecordedBarycentric = totalDepenetrationDirection.normalized;

                _moveDirection = forwardFromRecordedBarycentric;
                newDirection = forwardFromRecordedBarycentric;
                _input = forwardFromRecordedBarycentric;
                depenetrate = true;
                _plane = new Plane(Mathf2.RotateAroundAxis(forwardFromRecordedBarycentric, transform.up, 90), transform.position);
            }
            else
            {
                // distance = totalDepenetrationDirection.magnitude * 0.9f;
                forwardFromRecordedBarycentric = totalDepenetrationDirection.normalized;

                _moveDirection = forwardFromRecordedBarycentric;
                newDirection = forwardFromRecordedBarycentric;
                _input = forwardFromRecordedBarycentric;
                _plane = new Plane(Mathf2.RotateAroundAxis(forwardFromRecordedBarycentric, transform.up, 90), transform.position);
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
            _cut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _index, _lastIndex, ref _firstMoveDone, newDirection, transform.position, _plane, CutType.Start);
        }
        else // else do a test cut - test cut considers all edges and returns a "cut" but doesn't change the current edge/index information. this is good because we want to be able to get an edge we are standing directly on.
        {
            _cut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _index, _lastIndex, ref _firstMoveDone, newDirection, transform.position, _plane, CutType.Test);
        }

        // Add to the total distance to the cut from the position
        totalDistanceChecked += Vector3.Distance(transform.position, _cut);
        // make the currentCheckPosition the same as cut
        _currentCheckPosition = _cut;

        int i = 0;

        // newPosition is updated as we walk through the mesh
        _newPosition = Vector3.zero;
        // if we didn't reach the next edge with this movement, calculate the new position in the current triangle
        if (totalDistanceChecked >= distance)
        {
            _newPosition = transform.position + Vector3.ClampMagnitude(_cut - transform.position, distance);
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
        _lastEdgePoints.Set(_currentEdgePoints.Start, _currentEdgePoints.End, _currentEdgePoints.Other);

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
                int[] nextTriIndices = EdgeUtils.GetNextTri(_cm, ref _index, _currentEdgePoints);
                EdgePoints nextTri = new();
                nextTri.Set(nextTriIndices[0], nextTriIndices[1], nextTriIndices[2]);
                // if GetNextTri() returns same value as index, we are on an edge, else do nothing
                if (_index == _lastIndex)
                {
                    _onEdge = true;
                }
                else
                {
                    _cut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, nextTri, _index, _lastIndex, ref _firstMoveDone, newDirection, _newPosition, _plane, CutType.Next);

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
                        _currentEdgePoints.Set(_lastEdgePoints.Start, _lastEdgePoints.End, _lastEdgePoints.Other);
                        _index = _previousLastIndex;
                        totalDistanceChecked = distance;
#if UNITY_EDITOR
                        EditorApplication.isPaused = true;
#endif
                        break;
                    }
                    Plane tempPlane = new Plane(-transform.right, transform.position);
                }
            }
            else // if on edge
            {
                // get the point where the character is trying to move, if it moved off the current tri into space
                Vector3 movePositionAttempt = _newPosition + newDirection * remainingDistance;
                Vector3 slidePoint = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End]);

                int currentCornerInt = -1;
                int currentCornerIntOther = 0;
                if (slidePoint == _cm.Vertices[_currentEdgePoints.Start])
                {
                    currentCornerInt = _currentEdgePoints.Start;
                    currentCornerIntOther = _currentEdgePoints.End;
                }
                if (slidePoint == _cm.Vertices[_currentEdgePoints.End])
                {
                    currentCornerInt = _currentEdgePoints.End;
                    currentCornerIntOther = _currentEdgePoints.Start;
                }

                ////////////////////////////////
                //                            //
                //   !!!!!!!!!!!!!!!!!!!!!!   //
                //   !!                  !!   //
                //   !!  START REFACTOR  !!   //
                //   !!                  !!   //
                //   !!     510 LINES    !!   //
                //   !!                  !!   //
                //   !!  START REFACTOR  !!   //
                //   !!                  !!   //
                //   !!!!!!!!!!!!!!!!!!!!!!   //
                //                            //
                ////////////////////////////////

                // if we reached a corner
                if (currentCornerInt != -1)
                {
                    totalDistanceChecked = EdgeUtils.MeasureAttemptedSlideAlongEdgeThisFrame(distance, totalDistanceChecked, _newPosition, movePositionAttempt, slidePoint);

                    // If the next edge is another outside edge on the same triangle, switch to that. 
                    if (EdgeUtils.NextEdgeIsOnThisTriangle(_cm, out var nextEdgeOnThisTriangle, _currentEdgePoints, currentCornerInt))
                    {
                        _currentEdgePoints.Set(nextEdgeOnThisTriangle.pointA, nextEdgeOnThisTriangle.pointB, currentCornerIntOther);
                    }
                    else // If the next edge isn't an outside edge on the same triangle
                    {
                        if (slidePoint == _cm.Vertices[_currentEdgePoints.Start])
                        {
                            _cornerReached = _currentEdgePoints.Start;
                        }
                        else if (slidePoint == _cm.Vertices[_currentEdgePoints.End])
                        {
                            _cornerReached = _currentEdgePoints.End;
                        }

                        int timesLooped = 0;

                        List<Edge> tempEdges = new List<Edge>();

                        Edge checkIsOnEdge = new();

                        EdgePoints currentEdgeRealTemp = new();

                        // check the next triangle's edges, we're setting currentEdgeRealTemp because we don't want to override currentEdge etc
                        EdgeUtils.FindMatchingEdgePointsOnTriangle(_index, _currentEdgePoints, ref currentEdgeRealTemp, _cm);

                        if (currentEdgeRealTemp.Start == -1 && currentEdgeRealTemp.End == -1)
                        {
                            EdgeUtils.DebugTriangle(_cm, _index, Color.blue);
#if UNITY_EDITOR
                            EditorApplication.isPaused = true;
#endif
                        }

                        checkIsOnEdge.pointA = currentEdgeRealTemp.Start;
                        checkIsOnEdge.pointB = currentEdgeRealTemp.End;

                        if (!_cm.EdgeAdjacencyInfo.ContainsKey(checkIsOnEdge))
                        {
                            checkIsOnEdge.pointA = currentEdgeRealTemp.End;
                            checkIsOnEdge.pointB = currentEdgeRealTemp.Start;
                        }

                        if (checkIsOnEdge.pointA == -1 || checkIsOnEdge.pointB == -1)
                        {
#if UNITY_EDITOR
                            EditorApplication.isPaused = true;
#endif
                        }

                        if (EdgeUtils.EdgeIsOutsideEdge(checkIsOnEdge, _cm))
                        {
                            foreach (Edge e in _cm.EdgesAttachedToCorner[_cornerReached])
                            {
                                if (!EdgeUtils.EdgesMatchByPosition(e, _currentEdgePoints, _cm))
                                {
                                    tempEdges.Add(e);
                                }
                            }
                        }
                        else
                        {
                            Edge firstCornerEdge = EdgeUtils.FindFirstEdgeOnCorner(_cornerReached, _index, _cm);
                            Edge secondCornerEdge = new Edge();

                            Vector3 firstCornerEdgeNormal = EdgeUtils.GetEdgeNormal(_cm, firstCornerEdge, out var firstCornerEdgeOtherPoint);
                            Vector3 secondCornerEdgeNormal = Vector3.one;

                            timesLooped = 0;

                            Edge firstEdgeCrossed = new Edge();

                            int triangleCheckCorner = _index;
                            int lastTriangleCheckCorner = -1;

                            triangleCheckCorner = _index;
                            lastTriangleCheckCorner = -1;

                            bool edgeFound = false;

                            while (!edgeFound)
                            {
                                foreach (Edge e in _cm.EdgesAttachedToCorner[_cornerReached])
                                {
                                    int edgeTriangleA = _cm.EdgeAdjacencyInfo[e].triangleA;
                                    int edgeTriangleB = _cm.EdgeAdjacencyInfo[e].triangleB;

                                    if (edgeTriangleA == triangleCheckCorner && !edgeFound)
                                    {
                                        if (!EdgeUtils.EdgesMatchByPosition(e, firstEdgeCrossed, _cm))
                                        {
                                            if (EdgeUtils.EdgeIsOutsideEdge(e, _cm))
                                            {
                                                secondCornerEdge = e;
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
                                        if (!EdgeUtils.EdgesMatchByPosition(e, firstEdgeCrossed, _cm))
                                        {
                                            if (EdgeUtils.EdgeIsOutsideEdge(e, _cm))
                                            {
                                                secondCornerEdge = e;
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

                            secondCornerEdgeNormal = EdgeUtils.GetEdgeNormal(_cm, secondCornerEdge, out var secondCornerEdgeOtherPoint);

                            timesLooped = 0;

                            foreach (Edge e in _cm.EdgesAttachedToCorner[_cornerReached])
                            {
                                tempEdges.Add(e);
                            }

                            if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[firstCornerEdge.pointA], _cm.Vertices[firstCornerEdge.pointB]) == _cm.Vertices[_cornerReached] &&
                                Vector3.Dot(firstCornerEdgeNormal, _moveDirection) < 0)
                            {
                                tempEdges.Remove(firstCornerEdge);
                                _index = _cm.EdgeAdjacencyInfo[firstCornerEdge].triangleA;

                                if (_cm.Vertices[firstCornerEdge.pointA] == _cm.Vertices[_cornerReached])
                                {
                                    _currentEdgePoints.End = firstCornerEdge.pointA;
                                    _currentEdgePoints.Start = firstCornerEdge.pointB;
                                }
                                else if (_cm.Vertices[firstCornerEdge.pointB] == _cm.Vertices[_cornerReached])
                                {
                                    _currentEdgePoints.End = firstCornerEdge.pointB;
                                    _currentEdgePoints.Start = firstCornerEdge.pointA;
                                }

                                _currentEdgePoints.Other = firstCornerEdgeOtherPoint;
                            }
                            else if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[secondCornerEdge.pointA], _cm.Vertices[secondCornerEdge.pointB]) == _cm.Vertices[_cornerReached])
                            {
                                tempEdges.Remove(secondCornerEdge);
                                _index = _cm.EdgeAdjacencyInfo[secondCornerEdge].triangleA;

                                if (_cm.Vertices[secondCornerEdge.pointA] == _cm.Vertices[_cornerReached])
                                {
                                    _currentEdgePoints.End = secondCornerEdge.pointA;
                                    _currentEdgePoints.Start = secondCornerEdge.pointB;
                                }
                                else if (_cm.Vertices[secondCornerEdge.pointB] == _cm.Vertices[_cornerReached])
                                {
                                    _currentEdgePoints.End = secondCornerEdge.pointB;
                                    _currentEdgePoints.Start = secondCornerEdge.pointA;
                                }

                                _currentEdgePoints.Other = secondCornerEdgeOtherPoint;
                            }
                        }

                        bool foundNextEdge = false;
                        int x = 0;
                        int tempIndex = _index;
                        int tempLastIndex = _lastIndex;

                        List<Edge> checkedEdges = new List<Edge>();
                        List<Edge> checkedEdgesCorner = new List<Edge>();

                        EdgePoints cornerEdgePoints = new();

                        int cornerTriangleIndex = tempIndex;

                        // if we haven't found an outer edge
                        while (cornerEdgePoints.Start == -1)
                        {
                            foreach (Edge e in _cm.TriangleAdjacencyInfo[cornerTriangleIndex].edges)
                            {
                                // We need to find the other corner edge
                                bool tempEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in tempEdges)
                                {
                                    if (EdgeUtils.EdgesMatchByPosition(e, edgeToCheck, _cm))
                                    {
                                        tempEdgesContainsEdge = true;
                                    }
                                }
                                bool checkedEdgesCornerContainsEdge = false;
                                foreach (Edge edgeToCheck in checkedEdgesCorner)
                                {
                                    if (EdgeUtils.EdgesMatchByPosition(e, edgeToCheck, _cm))
                                    {
                                        checkedEdgesCornerContainsEdge = true;
                                    }
                                }
                                // if the edge is one of the edges attached to the current corner
                                if (tempEdgesContainsEdge && !checkedEdgesCornerContainsEdge)
                                {
                                    // and it's an outside edge
                                    if (EdgeUtils.EdgeIsOutsideEdge(e, _cm))
                                    {
                                        if (_cm.Vertices[e.pointA] == _cm.Vertices[_cornerReached])
                                        {
                                            cornerEdgePoints.Start = e.pointA;
                                            cornerEdgePoints.End = e.pointB;
                                        }
                                        else if (_cm.Vertices[e.pointB] == _cm.Vertices[_cornerReached])
                                        {
                                            cornerEdgePoints.Start = e.pointB;
                                            cornerEdgePoints.End = e.pointA;
                                        }
                                        cornerEdgePoints.Other = EdgeUtils.GetOtherVertexIndexFromTriangle(cornerEdgePoints, cornerTriangleIndex, _cm);
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
                                        foreach (Edge adjacentEdgeOnTriangle in _cm.TriangleAdjacencyInfo[cornerTriangleIndex].edges)
                                        {
                                            if (EdgeUtils.EdgesMatchByPosition(adjacentEdgeOnTriangle, e, _cm))
                                            {
                                                if (!checkedEdgesCorner.Contains(adjacentEdgeOnTriangle))
                                                {
                                                    checkedEdgesCorner.Add(adjacentEdgeOnTriangle);
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
                                    if (EdgeUtils.EdgesMatchByPosition(e, edgeToCheck, _cm))
                                    {
                                        tempEdgesContainsEdge = true;
                                    }
                                }
                                bool checkedEdgesContainsEdge = false;
                                foreach (Edge edgeToCheck in checkedEdges)
                                {
                                    if (EdgeUtils.EdgesMatchByPosition(e, edgeToCheck, _cm))
                                    {
                                        checkedEdgesContainsEdge = true;
                                    }
                                }
                                // if the edge is one of the edges attached to the current corner
                                if (tempEdgesContainsEdge && !checkedEdgesContainsEdge)
                                {
                                    // and it's an outside edge
                                    if (EdgeUtils.EdgeIsOutsideEdge(e, _cm))
                                    {
                                        foundNextEdge = true;
                                        _previousLastIndex = _lastIndex;
                                        _previousIndex = _index;

                                        _lastEdgePoints.Set(_currentEdgePoints.Start, _currentEdgePoints.End, _currentEdgePoints.Other);

                                        _currentEdgePoints.Start = e.pointA;
                                        _currentEdgePoints.End = e.pointB;

                                        _lastIndex = tempLastIndex;
                                        _index = tempIndex;

                                        _currentEdgePoints.Other = EdgeUtils.GetOtherVertexIndex(_currentEdgePoints, _cm);

                                        triCenter = (_cm.Vertices[_currentEdgePoints.Start] + _cm.Vertices[_currentEdgePoints.End] + _cm.Vertices[_currentEdgePoints.Other]) / 3;
                                        closestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[_currentEdgePoints.Start], (_cm.Vertices[_currentEdgePoints.Start] - _cm.Vertices[_currentEdgePoints.End]).normalized, triCenter);

                                        Vector3 previousTriCenter = (_cm.Vertices[_lastEdgePoints.Start] + _cm.Vertices[_lastEdgePoints.End] + _cm.Vertices[_lastEdgePoints.Other]) / 3;
                                        Vector3 previousClosestPointOnEdge = Mathf2.NearestPointOnLine(_cm.Vertices[_lastEdgePoints.Start], (_cm.Vertices[_lastEdgePoints.Start] - _cm.Vertices[_lastEdgePoints.End]).normalized, previousTriCenter);

                                        _newPosition = slidePoint;

                                        Vector3 cornerNormal = Vector3.Cross(_cm.Vertices[_lastEdgePoints.Start] - _cm.Vertices[_lastEdgePoints.End], _cm.Vertices[_currentEdgePoints.Start] - _cm.Vertices[_currentEdgePoints.End]).normalized;
                                        if (Vector3.Dot(cornerNormal, transform.up) < 0)
                                        {
                                            cornerNormal = -cornerNormal;
                                        }
                                        Vector3 newEdgeNormal = (closestPointOnEdge - triCenter).normalized;
                                        Vector3 lastEdgeNormal = (previousClosestPointOnEdge - previousTriCenter).normalized;

                                        Vector3 newFarCorner = _cm.Vertices[currentCornerInt] == _cm.Vertices[_currentEdgePoints.Start] ? _cm.Vertices[_currentEdgePoints.End] : _cm.Vertices[_currentEdgePoints.Start];
                                        Vector3 lastFarCorner = _cm.Vertices[currentCornerInt] == _cm.Vertices[_lastEdgePoints.Start] ? _cm.Vertices[_lastEdgePoints.End] : _cm.Vertices[_lastEdgePoints.Start];

                                        int shouldInvertNewEdgeNormal = 1;
                                        Vector3 cornerDirection = Vector3.ProjectOnPlane(transform.rotation * _input, cornerNormal).normalized;

                                        Vector3 newEdgeNormal2 = Vector3.ProjectOnPlane(newEdgeNormal * shouldInvertNewEdgeNormal, cornerNormal).normalized;
                                        Vector3 lastEdgeNormal2 = Vector3.ProjectOnPlane(lastEdgeNormal, cornerNormal).normalized;

                                        if (Vector3.Dot((transform.rotation * _input).normalized, (closestPointOnEdge - triCenter).normalized) < 0)
                                        {
                                            _onEdge = false;
                                            foundNextEdge = true;
                                        }

                                        Vector3 slidePoint2 = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[_lastEdgePoints.Start], _cm.Vertices[_lastEdgePoints.End]);
                                        Vector3 slidePoint2Clamped = Mathf2.NearestPointOnLine(_cm.Vertices[_lastEdgePoints.Start], _cm.Vertices[_lastEdgePoints.End], movePositionAttempt);
                                        Vector3 slidePoint3 = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End]);
                                        Vector3 slidePoint3Clamped = Mathf2.NearestPointOnLine(_cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End], movePositionAttempt);
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
                                            EdgeUtils.DoRaysIntersect(ray1, ray2) &&
                                            EdgeUtils.IsTriAfterNextThis(_cm, transform, _newPosition, _currentEdgePoints, _index, _lastIndex, _firstMoveDone, _cornerReached, _input)
                                        )
                                        {
                                            // NOT SWITCHING TO OTHER EDGE SO OF COURSE SLIDEPOINT IS STILL AT THE CORNER
                                            _onEdge = true;

                                            _currentEdgePoints.Set(_lastEdgePoints.Start, _lastEdgePoints.End, _lastEdgePoints.Other);

                                            _index = _previousIndex;

                                            _lastIndex = _previousLastIndex;
                                            totalDistanceChecked = distance;
                                            _newPosition = slidePoint;
                                            slidePoint = _cm.Vertices[_cornerReached];
                                            foundNextEdge = true;
                                        }

                                    }
                                    else
                                    {
                                        EdgePoints nextTriCurrentEdgePoints = new();
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
                                            nextTriCurrentEdgePoints.Start = e.pointA;
                                            nextTriCurrentEdgePoints.End = e.pointB;
                                        }
                                        else
                                        {
                                            nextTriCurrentEdgePoints.Start = e.pointB;
                                            nextTriCurrentEdgePoints.End = e.pointA;
                                        }

                                        nextTriCurrentEdgePoints.Other = EdgeUtils.GetOtherVertexIndexFromTriangle(nextTriCurrentEdgePoints, tempIndex, _cm);

                                        // check if we are pointing towards the other edge (not the next edge attached to corner)
                                        // if so, we can come unstuck.

                                        if (EdgeUtils.GetFarEdgeCut(_cm, transform, _input, _cornerReached, tempIndex, _currentEdgePoints, nextTriCurrentEdgePoints, cornerEdgePoints, _plane))
                                        {
                                            _index = tempIndex;
                                            _lastIndex = tempLastIndex;
                                            _currentEdgePoints.Set(nextTriCurrentEdgePoints.End, nextTriCurrentEdgePoints.Start, nextTriCurrentEdgePoints.Other);
                                            _onEdge = false;
                                            foundNextEdge = true;
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

                                return;
                            }
                        }


                    }
                    _newPosition = slidePoint;
                }
                else
                {
                    totalDistanceChecked = distance;
                    _newPosition = slidePoint;
                }

                ////////////////////////////////
                //                            //
                //   !!!!!!!!!!!!!!!!!!!!!!   //
                //   !!                  !!   //
                //   !!   END REFACTOR   !!   //
                //   !!                  !!   //
                //   !!     510 LINES    !!   //
                //   !!                  !!   //
                //   !!   END REFACTOR   !!   //
                //   !!                  !!   //
                //   !!!!!!!!!!!!!!!!!!!!!!   //
                //                            //
                ////////////////////////////////

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

        _barycentricCoordinate = Mathf2.GetBarycentricCoordinates(_newPosition, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End], _cm.Vertices[_currentEdgePoints.Other]);

        Vector3 updatedPosition = EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinate, _currentEdgePoints);

        triCenter = (_cm.Vertices[_currentEdgePoints.Start] + _cm.Vertices[_currentEdgePoints.End] + _cm.Vertices[_currentEdgePoints.Other]) / 3;

        Vector3 testPosition = EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinate, _currentEdgePoints);

        EdgeUtils.GetMatchingEdgeOnAdjacentTriangle(_cm, out edgeAdjacentToCurrent, _currentEdgePoints, _index);

        groundNormal = EdgeUtils.GetNormalFromBarycentric(_cm, _barycentricCoordinate, edgeAdjacentToCurrent);

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
        }
        _testCut = Vector3.zero;
        _testCut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _index, _lastIndex, ref _firstMoveDone, _castDirectionTest, triCenter, _testPlane, CutType.Test);

        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        _lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End], _cm.Vertices[_currentEdgePoints.Other]);
        _testCut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _index, _lastIndex, ref _firstMoveDone, -_castDirectionTest, triCenter, _testPlane, CutType.Test);
        _barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End], _cm.Vertices[_currentEdgePoints.Other]);
        //    Debug.DrawLine(transform.position, transform.position + plane.normal, Color.blue);


        if (forwardFromRecordedBarycentric != Vector3.zero)
        {
            _forwardLastFrame = forwardFromRecordedBarycentric;
        }
        else
        {
            _forwardLastFrame = CharacterModel.forward;
        }

        // Debug.DrawLine(CharacterModel.position, CharacterModel.position + CharacterModel.forward, Color.red);
    }

    //////////////////
    // REFACTOR END //
    //////////////////
    ///

    void OnGUI()
    {
        // Display the frame rate on the screen
        GUIStyle style = new GUIStyle();
        style.normal.textColor = Color.white;
        style.fontSize = 20;
        Rect rect = new Rect(10, 10, 200, 30);
        GUI.Label(rect, "Frame Rate: " + _frameRate.ToString("F2"), style);
    }

    void OnDrawGizmos()
    {
        // Gizmos.color = Color.blue;
        // Gizmos.DrawWireSphere(checkPositionStart, 0.05f);
        // Gizmos.DrawWireSphere(checkPositionEnd, 0.05f);
    }
}
