using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Overhang;
using KinematicCharacterController;

public class ClimbShape : MonoBehaviour
{
    HeroCharacterController _cc;
    ClimbableMesh _cm;

    public Transform CharacterPivot;
    Rigidbody _rb;
    int _currentTriangleIndex, _lastTriangleIndex;

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

    float _deltaTimeFPS;
    float _frameRate;

    public MovementMode _movementMode;
    Vector3 _forwardLastFrame;

    public bool ForceSlide;
    public bool ForceSlideForwardProjection;

    public Collider DepenetrationCapsule;

    Quaternion _afterDepenetrateRotation;

    Vector3 _previousRayCastPosition;
    Vector3 _positionLastFrame;
    float _timeSinceJumped;

    void Awake()
    {
        Physics.autoSyncTransforms = true;
        Cursor.visible = false;
        Cursor.lockState = CursorLockMode.Locked;
        Application.targetFrameRate = 300;
        // set the plane used for pathfinding to be oriented to the character
        _plane = new Plane(-transform.right, transform.position);
        _cc = GetComponent<HeroCharacterController>();
        _rb = GetComponent<Rigidbody>();
    }

    void LateUpdate()
    {
        Climb();
        // Climb();
    }
    void Climb()
    {
        Physics.SyncTransforms();
        // Calculate the time it took to render the last frame
        _deltaTimeFPS += (Time.unscaledDeltaTime - _deltaTimeFPS) * 0.1f;
        _timeSinceJumped += Time.deltaTime;

        // Calculate the frame rate in frames per second
        _frameRate = 1.0f / _deltaTimeFPS;

        // if not on a mesh, raycast to find a mesh
        if (!_isClimbing)
        {

            ClimbUtils.TryStartClimb(ref _cm, transform, _rb, _timeSinceJumped, ref _currentEdgePoints, ref _currentTriangleIndex, ref _lastTriangleIndex, ref _firstMoveDone, ref _testCut, ref _barycentricCoordinate, ref _lastBarycentricCoordinate, ref _barycentricCoordinateBehind, ref _previousRayCastPosition, _layerMask, ref _isClimbing);

            CharacterPivot.rotation = transform.rotation;
        }
        else
        {
            // Debug.DrawLine(CharacterPivot.position, CharacterPivot.position + Vector3.up, Color.green);
            // Debug.DrawLine(transform.position, CharacterPivot.position + Vector3.up, Color.red);


            _cm.RecalculateMesh(false);

            // Get input
            Vector3 tempInput = new Vector3(Input.GetAxisRaw("Horizontal"), 0, Input.GetAxisRaw("Vertical")).normalized;

            // This is for debugging - makes the character go forwards and backwards over an edge rapidly, so I don't have to hammer the keyboard to test. 
            _goForwardTest = Input.GetKeyDown(KeyCode.K) ? !_goForwardTest : _goForwardTest;

            tempInput = _goForwardTest ? new Vector3(0, 0, 1) : tempInput;

            // if input has magnitude then use it
            if (tempInput.magnitude > 0)
            {
                _input = tempInput;
            }

            // calculate movement on the mesh based on input
            Travel(directionInput: tempInput, depenetratePass: false, isFinalPass: false);


            int depenetrationIterations = 1;
            for (int i = 0; i < depenetrationIterations; i++)
            {
                bool isFinalPass = i == depenetrationIterations - 1;
                Depenetrate(isFinalPass);
                Physics.SyncTransforms();

            }

            // Get the new deformed position of the player based on the vertex and barycentric coordinate that was calculated in the last loop
            Vector3 finalPosition = EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinate, _currentEdgePoints);
            Quaternion finalRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized, Vector3.up);


            transform.position = finalPosition;
            _rb.position = finalPosition;
            _cc.Motor.BaseVelocity = Vector3.zero;
            _cc.Motor.SetPosition(finalPosition);

            // Debug.DrawLine(finalPosition, finalPosition + Vector3.up, Color.red);
            // Debug.DrawLine(_positionLastFrame, finalPosition, Color.cyan);

            if (Input.GetKeyDown(KeyCode.Space))
            {
                // Debug.DrawLine(transform.position, transform.position + finalRotation * Vector3.forward, Color.blue);
                // Debug.DrawLine(finalPosition, finalPosition + Vector3.up, Color.red);

                _timeSinceJumped = 0;
                ClimbUtils.LeaveClimbableMesh(transform, _cc, CharacterPivot, _positionLastFrame, finalPosition, finalRotation, _rb, ref _isClimbing);
                return;
            }

            _positionLastFrame = finalPosition;
        }
    }

    void Depenetrate(bool isFinalPass)
    {
        Travel(Vector3.zero, true, isFinalPass);
    }

    void Travel(Vector3 directionInput, bool depenetratePass, bool isFinalPass)
    {
        // Get the new deformed positions for the player based on the vertex and barycentric coordinate that was calculated in the last loop
        (Vector3 pointOnTriangle, Vector3 behindPointOnTriangle, Vector3 forwardPointOnTriangle) =
        (
            EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinate, _currentEdgePoints),
            EdgeUtils.GetPositionFromBarycentric(_cm, _barycentricCoordinateBehind, _currentEdgePoints),
            EdgeUtils.GetPositionFromBarycentric(_cm, _lastBarycentricCoordinate, _currentEdgePoints)
        );

        transform.position = pointOnTriangle;

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        // Switch movement mode based on input
        ClimbUtils.HandleMovementModeSwitch(_cm, transform, ref _currentEdgePoints, ref _currentTriangleIndex, ref _lastTriangleIndex, ref _firstMoveDone, ref _testCut, ref _barycentricCoordinate, ref _lastBarycentricCoordinate, ref _barycentricCoordinateBehind, ref _movementMode, ref _newPosition, ref _onEdge);
        ClimbUtils.GetGroundNormal(_cm, out Vector3 groundNormal, _barycentricCoordinate, _currentEdgePoints, _currentTriangleIndex);

        Vector3 forwardFromRecordedBarycentric = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        float turnAngle = (Input.GetKey(KeyCode.E) ? 100 * Time.deltaTime : 0) - (Input.GetKey(KeyCode.Q) ? 100 * Time.deltaTime : 0);
        forwardFromRecordedBarycentric = ClimbUtils.GetTurnedForwardVector(forwardFromRecordedBarycentric, groundNormal, turnAngle);

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
#if UNITY_EDITOR

            Debug.Log("forward zero");
#endif
        }

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
                    CharacterPivot.rotation = _afterDepenetrateRotation;
                }
                Plane NewtempForwardPlane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterPivot.right, transform.position);
                Vector3 cutDirection = Mathf2.RotateAroundAxis(NewtempForwardPlane.normal, groundNormal, 90);
                forwardPointOnTriangle = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _currentTriangleIndex, _lastTriangleIndex, ref _firstMoveDone, cutDirection, transform.position, NewtempForwardPlane, CutType.Test);
                behindPointOnTriangle = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _currentTriangleIndex, _lastTriangleIndex, ref _firstMoveDone, -cutDirection, transform.position, NewtempForwardPlane, CutType.Test);
                forwardFromRecordedBarycentric = (forwardPointOnTriangle - behindPointOnTriangle).normalized;
            }
            else
            {
                if (!depenetratePass)
                {
                    Debug.DrawLine(transform.position, transform.position + forwardFromRecordedBarycentric, Color.red);
                    _afterDepenetrateRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(forwardFromRecordedBarycentric, Vector3.up).normalized, Vector3.up);
                }
                if (isFinalPass)
                {
                    CharacterPivot.rotation = _afterDepenetrateRotation;
                }
            }
            _targetSpeed = DirectionalSpeed;

        }
        else
        {
            _targetSpeed = ClimbingSpeed;
            CharacterPivot.localRotation = Quaternion.identity;
        }

        // This calculates the move direction. What happens if I just rotate this?
        // Get the move direction relative to the player's orientation
        _moveDirection = (transform.rotation * _input).normalized;

        // newDirection - can't remember what this does but it's possible it will change in the loop but we want to keep moveDirection the same.
        Vector3 newDirection = _moveDirection;

        // calculate distance to move
        Vector3 _checkPositionStart = CharacterPivot.position + CharacterPivot.up * 0.05f;
        Vector3 _checkPositionEnd = CharacterPivot.position + CharacterPivot.up * 0.15f;
        Collider[] colliders = Physics.OverlapCapsule(_checkPositionStart, _checkPositionEnd, 0.05f, _layerMaskForwardProjection);

        bool depenetrate = false;
        bool slide = false;
        float distanceToMoveThisFrame = directionInput.magnitude * _targetSpeed * Time.deltaTime;
        ////////////////////////////////
        //                            //
        //   !!!!!!!!!!!!!!!!!!!!!!   //
        //   !!                  !!   //
        //   !!  START REFACTOR  !!   //
        //   !!                  !!   //
        //   !!  510 to 63 LINES !!   //
        //   !!                  !!   //
        //   !!  START REFACTOR  !!   //
        //   !!                  !!   //
        //   !!!!!!!!!!!!!!!!!!!!!!   //
        //                            //
        ////////////////////////////////

        // This works in Build! It just fails in editor because of dodgy deltaTime. 
        if (depenetratePass)
        {
            Vector3 depenetrationDirection = Vector3.zero;
            Vector3 totalDepenetrationDirection = Vector3.zero;

            float depenetrationDistance = 0;

            if (colliders.Length > 0)
            {
                foreach (Collider col in colliders)
                {
#if UNITY_EDITOR                    
                    Debug.Log("Found Collision " + col.name);
#endif
                    Physics.ComputePenetration(DepenetrationCapsule, DepenetrationCapsule.transform.position, DepenetrationCapsule.transform.rotation, col, col.transform.position, col.transform.rotation, out depenetrationDirection, out depenetrationDistance);
                    if (depenetrationDistance > 0.00001f)
                    {
                        totalDepenetrationDirection += Vector3.ProjectOnPlane(depenetrationDirection * depenetrationDistance, groundNormal);
                    }
                }

                // Don't need to average this! Additive is better.
                Plane wallPlane = new();
                Vector3 wallNormal = depenetrationDirection.normalized;

                wallPlane.SetNormalAndPosition(wallNormal, depenetrationDirection * depenetrationDistance);

                Ray depenetrateRay = EdgeUtils.CreateRayFromDirection(Vector3.zero, totalDepenetrationDirection.normalized);
                wallPlane.Raycast(depenetrateRay, out distanceToMoveThisFrame);

                forwardFromRecordedBarycentric = totalDepenetrationDirection.normalized;

                _moveDirection = forwardFromRecordedBarycentric;
                newDirection = forwardFromRecordedBarycentric;
                _input = forwardFromRecordedBarycentric;
                depenetrate = true;
                _plane = new Plane(Mathf2.RotateAroundAxis(forwardFromRecordedBarycentric, transform.up, 90), transform.position);
            }
            else
            {
                forwardFromRecordedBarycentric = totalDepenetrationDirection.normalized;

                _moveDirection = forwardFromRecordedBarycentric;
                newDirection = forwardFromRecordedBarycentric;
                _input = forwardFromRecordedBarycentric;
                _plane = new Plane(Mathf2.RotateAroundAxis(forwardFromRecordedBarycentric, transform.up, 90), transform.position);
            }
        }

        ////////////////////////////////
        //                            //
        //   !!!!!!!!!!!!!!!!!!!!!!   //
        //   !!                  !!   //
        //   !!   END REFACTOR   !!   //
        //   !!                  !!   //
        //   !!  510 to 63 LINES !!   //
        //   !!                  !!   //
        //   !!   END REFACTOR   !!   //
        //   !!                  !!   //
        //   !!!!!!!!!!!!!!!!!!!!!!   //
        //                            //
        ////////////////////////////////

        if (!depenetrate && !slide)
        {
            // Create plane to calculate "cuts" from
            if (_movementMode == MovementMode.Car)
            {
                _plane = new Plane(CharacterPivot.rotation * (Quaternion.Euler(0, 90, 0) * _input), transform.position);
            }
            else
            {
                _plane = new Plane(Quaternion.FromToRotation(Vector3.up, groundNormal) * CharacterPivot.right, transform.position);
            }

        }

        // total distance checked this frame - if goes over, loop will break
        float totalDistanceChecked = 0;

        // If on the edge (as of last frame) and holding away from said edge then not on edge any more
        if (!ClimbUtils.MovingTowardsEdge(_cm, _moveDirection, _currentEdgePoints))
        {
            _onEdge = false;
        }

        // if not on edge, do a start cut
        // start cut considers all edges
        _cut = transform.position;

        if (!_onEdge)
        {
            _cut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _currentTriangleIndex, _lastTriangleIndex, ref _firstMoveDone, newDirection, transform.position, _plane, CutType.Start);
        }
        else // else do a test cut - test cut considers all edges and returns a "cut" but doesn't change the current edge/index information. this is good because we want to be able to get an edge we are standing directly on.
        {
            _cut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _currentTriangleIndex, _lastTriangleIndex, ref _firstMoveDone, newDirection, transform.position, _plane, CutType.Test);
        }

        // Add to the total distance to the cut from the position
        totalDistanceChecked += Vector3.Distance(transform.position, _cut);
        // make the currentCheckPosition the same as cut
        _currentCheckPosition = _cut;

        int i = 0;

        // newPosition is updated as we walk through the mesh
        _newPosition = Vector3.zero;
        // if we didn't reach the next edge with this movement, calculate the new position in the current triangle
        if (totalDistanceChecked >= distanceToMoveThisFrame)
        {
            _newPosition = transform.position + Vector3.ClampMagnitude(_cut - transform.position, distanceToMoveThisFrame);
        }
        else // else make the next check go from the point on the next edge
        {
            _newPosition = _cut;
        }


        float remainingDistance = distanceToMoveThisFrame - Vector3.Distance(transform.position, _newPosition);
        _castDirectionTest = transform.forward;

        // Why do we need lastIndex, previousLastIndex and previousIndex?

        int _backupLastTriangleIndex = _currentTriangleIndex;
        int _backupCurrentTriangleIndex = _lastTriangleIndex;

        _lastEdgePoints.Set(_currentEdgePoints.Start, _currentEdgePoints.End, _currentEdgePoints.Other);

        // while we have checked less distance than the 
        while (totalDistanceChecked < distanceToMoveThisFrame)
        {
            // if not on edge / just started moving away from edge
            if (!_onEdge)
            {
                // set the last index to be the index - this is because we're about to check the next tri to see if we're on an edge, and we want to compare index to lastIndex
                _lastTriangleIndex = _currentTriangleIndex;

                // take the edge we just passed, then get the tri that we didn't just check
                int[] nextTriIndices = EdgeUtils.GetNextTri(_cm, ref _currentTriangleIndex, _currentEdgePoints);
                EdgePoints nextTri = new();
                nextTri.Set(nextTriIndices[0], nextTriIndices[1], nextTriIndices[2]);
                // if GetNextTri() returns same value as index, we are on an edge, else do nothing
                if (_currentTriangleIndex == _lastTriangleIndex)
                {
                    _onEdge = true;
                }
                else
                {
                    _cut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, nextTri, _currentTriangleIndex, _lastTriangleIndex, ref _firstMoveDone, newDirection, _newPosition, _plane, CutType.Next);

                    totalDistanceChecked += Vector3.Distance(_currentCheckPosition, _cut);

                    Vector3 oldPosition = _newPosition;

                    if (totalDistanceChecked >= distanceToMoveThisFrame)
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
                        _currentTriangleIndex = _backupLastTriangleIndex;
                        totalDistanceChecked = distanceToMoveThisFrame;
#if UNITY_EDITOR
                        EditorApplication.isPaused = true;
#endif
                    }
                }
            }
            else // if on edge
            {
                // get the point where the character is trying to move, if it moved off the current tri into space
                Vector3 movePositionAttempt = _newPosition + newDirection * remainingDistance;
                Vector3 slidePoint = Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End]);

                if (ClimbUtils.CornerReached(_cm, _currentEdgePoints, out EdgePoints cornerEdgePoints, slidePoint))
                {
                    totalDistanceChecked = EdgeUtils.MeasureAttemptedSlideAlongEdgeThisFrame(distanceToMoveThisFrame, totalDistanceChecked, _newPosition, movePositionAttempt, slidePoint);

                    // If the next edge is another outside edge on the same triangle, switch to that. 
                    if (EdgeUtils.NextEdgeIsOnThisTriangle(_cm, out var nextEdgeOnThisTriangle, _currentEdgePoints, cornerEdgePoints.Start))
                    {
                        // Switch to that edge
                        _currentEdgePoints.Set(nextEdgeOnThisTriangle.pointA, nextEdgeOnThisTriangle.pointB, cornerEdgePoints.Other);
                    }
                    else // If the next edge isn't an outside edge on the same triangle
                    {
                        _cornerReached = EdgeUtils.GetMatchingPointOnEdgeFromPosition(_cm, slidePoint, _currentEdgePoints);

                        List<Edge> edgeCandidates = new List<Edge>();

                        // If the first edge of the next triangle attached to the corner is an outside edge, it means 
                        if (EdgeUtils.TryGetMatchingOutsideEdge(_cm, _currentTriangleIndex, _currentEdgePoints))
                        {
                            edgeCandidates = EdgeUtils.GetEdgesAttachedToCornerThatArentThisOne(_cm, _cornerReached, _currentEdgePoints);
                        }
                        else
                        {
                            ClimbUtils.TryGetCornerMovedInto(_cm, _cornerReached, ref _currentTriangleIndex, ref edgeCandidates, movePositionAttempt, _moveDirection, ref _currentEdgePoints);
                        }

                        // Either get next triangle and keep going until outside edge is found, or get far edge cut or cancel if traversal is recursive.
                        ClimbUtils.ResolveCornerTraversal(_cm, transform, ref _currentTriangleIndex, ref _lastTriangleIndex, ref _backupCurrentTriangleIndex, ref _backupLastTriangleIndex, edgeCandidates, ref _currentEdgePoints, ref _lastEdgePoints, ref _newPosition, ref slidePoint, ref _onEdge, ref _input, _firstMoveDone, _cornerReached, ref totalDistanceChecked, distanceToMoveThisFrame, _movementMode, _plane, CharacterPivot);
                    }

                    _newPosition = slidePoint;
                }
                else
                {
                    totalDistanceChecked = distanceToMoveThisFrame;
                    _newPosition = slidePoint;
                }

                remainingDistance = remainingDistance - Vector3.Distance(_newPosition, slidePoint);
                _plane = new Plane(transform.rotation * Quaternion.Euler(0, 90, 0) * _input, _newPosition);

                if (++i > 1000)
                {
#if UNITY_EDITOR
                    Debug.Log("WHOOPS");
                    EditorApplication.isPaused = true;
#endif
                }
            }
        }

        _barycentricCoordinate = Mathf2.GetBarycentricCoordinates(_newPosition, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End], _cm.Vertices[_currentEdgePoints.Other]);

        Vector3 triCenter = EdgeUtils.GetTriangleCenter(_cm, _currentEdgePoints);

        ClimbUtils.GetGroundNormal(_cm, out groundNormal, _barycentricCoordinate, _currentEdgePoints, _currentTriangleIndex);

        _castDirectionTest = Quaternion.FromToRotation(transform.up, groundNormal) * transform.forward;

        if (_movementMode == MovementMode.Car)
        {
            _testPlane = new Plane(-(Quaternion.FromToRotation(transform.up, groundNormal) * transform.right), triCenter);
        }
        else
        {
            if (!isFinalPass)
            {
                _testPlane = new Plane(-(Quaternion.FromToRotation(transform.up, groundNormal) * transform.right), triCenter);

            }
            else
            {
                _testPlane = new Plane(-CharacterPivot.right, triCenter);
            }
        }

        _testCut = Vector3.zero;
        _testCut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _currentTriangleIndex, _lastTriangleIndex, ref _firstMoveDone, _castDirectionTest, triCenter, _testPlane, CutType.Test);

        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        _lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End], _cm.Vertices[_currentEdgePoints.Other]);
        _testCut = EdgeUtils.FindTrianglePlaneIntersection(_cm, ref _currentEdgePoints, _currentEdgePoints, _currentTriangleIndex, _lastTriangleIndex, ref _firstMoveDone, -_castDirectionTest, triCenter, _testPlane, CutType.Test);
        _barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(_testCut, _cm.Vertices[_currentEdgePoints.Start], _cm.Vertices[_currentEdgePoints.End], _cm.Vertices[_currentEdgePoints.Other]);

        if (forwardFromRecordedBarycentric != Vector3.zero)
        {
            _forwardLastFrame = forwardFromRecordedBarycentric;
        }
        else
        {
            _forwardLastFrame = CharacterPivot.forward;
        }
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

    void OnDrawGizmos()
    {
        // Gizmos.color = Color.blue;
        // Gizmos.DrawWireSphere(checkPositionStart, 0.05f);
        // Gizmos.DrawWireSphere(checkPositionEnd, 0.05f);
    }
}
