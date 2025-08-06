using System.Collections;
using System.Collections.Generic;
using Overhang;
using UnityEngine;
using UnityEditor;
using System;

public class ClimbUtils
{
    private static List<Edge> checkedEdges = new List<Edge>();


    public static void TryGetCornerMovedInto(ClimbableMesh cm, int cornerReached, ref int currentTriangleIndex, ref List<Edge> edgeCandidates, Vector3 movePositionAttempt, Vector3 moveDirection, ref EdgePoints currentEdgePoints)
    {
        Edge firstCornerEdge = EdgeUtils.FindFirstEdgeOnCorner(cornerReached, currentTriangleIndex, cm);
        Edge secondCornerEdge = EdgeUtils.FindNextOutsideEdgeFromCorner(cm, cornerReached, currentTriangleIndex);

        edgeCandidates.AddRange(cm.EdgesAttachedToCorner[cornerReached]);


        if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, cm.Vertices[firstCornerEdge.pointA], cm.Vertices[firstCornerEdge.pointB]) == cm.Vertices[cornerReached] &&
           MovingTowardsEdge(cm, moveDirection, firstCornerEdge))
        {
            edgeCandidates.Remove(firstCornerEdge);
            currentTriangleIndex = cm.EdgeAdjacencyInfo[firstCornerEdge].triangleA;
            EdgeUtils.SetOrderedEdgePoints(cm, firstCornerEdge, cornerReached, ref currentEdgePoints);
        }
        else if (Mathf2.GetClosestPointOnFiniteLine(movePositionAttempt, cm.Vertices[secondCornerEdge.pointA], cm.Vertices[secondCornerEdge.pointB]) == cm.Vertices[cornerReached])
        {
            edgeCandidates.Remove(secondCornerEdge);
            currentTriangleIndex = cm.EdgeAdjacencyInfo[secondCornerEdge].triangleA;
            EdgeUtils.SetOrderedEdgePoints(cm, secondCornerEdge, cornerReached, ref currentEdgePoints);
        }
    }

    public static void ResolveCornerTraversal(
        ClimbableMesh cm,
        Transform playerTransform,
        ref int _currentTriangleIndex,
        ref int _lastTriangleIndex,
        ref int _backupCurrentTriangleIndex,
        ref int _backupLastTriangleIndex,
        List<Edge> edgeCandidates,
        ref EdgePoints _currentEdgePoints,
        ref EdgePoints _lastEdgePoints,
        ref Vector3 _newPosition,
        ref Vector3 slidePoint,
        ref bool _onEdge,
        ref Vector3 _input,
        bool _firstMoveDone,
        int _cornerReached,
        ref float totalDistanceChecked,
        float distanceToMoveThisFrame,
        MovementMode _movementMode,
        Plane _plane,
        Transform CharacterModel
    )
    {
        int x = 0;
        int tempIndex = _currentTriangleIndex;
        int tempLastIndex = _lastTriangleIndex;

        bool foundNextEdge = false;

        checkedEdges.Clear();

        EdgePoints cornerEdgePoints = new(true);

        Edge foundEdge = EdgeUtils.FindFirstEdgeOnCorner(_cornerReached, _currentTriangleIndex, cm);

        if (EdgeUtils.EdgeIsOutsideEdge(foundEdge, cm))
        {
            EdgeUtils.SetOrderedEdgePointsFromTriangle(cm, foundEdge, _cornerReached, _currentTriangleIndex, ref cornerEdgePoints);
        }

        Vector3 rotatedInput = playerTransform.rotation * _input;

        // until we've found edge
        while (!foundNextEdge)
        {
            // we need to use a different method for contains - checking by their positions, since hard edges has different indices
            // scan the edges of the current triangle
            foreach (Edge e in cm.TriangleAdjacencyInfo[tempIndex].edges)
            {
                // if the edge is one of the edges attached to the current corner
                if (!EdgeUtils.EdgeIsACandidateAndNotAlreadyPassed(cm, e, edgeCandidates, checkedEdges))
                {
                    // and it's an outside edge
                    if (EdgeUtils.EdgeIsOutsideEdge(e, cm))
                    {
                        foundNextEdge = true;
                        _backupLastTriangleIndex = _lastTriangleIndex;
                        _backupCurrentTriangleIndex = _currentTriangleIndex;

                        _lastEdgePoints.Set(_currentEdgePoints.Start, _currentEdgePoints.End, _currentEdgePoints.Other);
                        EdgeUtils.SetOrderedEdgePoints(cm, e, e.pointA, ref _currentEdgePoints);

                        _lastTriangleIndex = tempLastIndex;
                        _currentTriangleIndex = tempIndex;

                        _newPosition = slidePoint;

                        if (!ClimbUtils.MovingTowardsEdge(cm, rotatedInput, _currentEdgePoints))
                        {
                            _onEdge = false;
                            foundNextEdge = true;
                        }

                        // This is for stopping the check from bouncing between edges when stuck in a corner
                        // Problem is that it's sticking to corners when the GetClosestPointOnLine attemptedMovePosition is outside the line
                        // In order to determine if we should be able to move round the corner, we need to know if it's a convex corner or not
                        // DoRaysIntersect of the edge normals determines this.
                        // If they do, then we check if the closest point on each edge to the attempted move position is equal to the corner - that's the only situation where we should move round the corner  
                        if (
                            EdgeUtils.CornersAreConvex(cm, _currentEdgePoints, _lastEdgePoints) &&
                            EdgeUtils.DoesFutureMoveLandOnSameTriangle(cm, playerTransform, _newPosition, _currentEdgePoints, _currentTriangleIndex, _lastTriangleIndex, _firstMoveDone, _cornerReached, _input)
                        )
                        {
                            // NOT SWITCHING TO OTHER EDGE SO OF COURSE SLIDEPOINT IS STILL AT THE CORNER
                            _onEdge = true;

                            _currentEdgePoints.Set(_lastEdgePoints.Start, _lastEdgePoints.End, _lastEdgePoints.Other);

                            _currentTriangleIndex = _backupCurrentTriangleIndex;
                            _lastTriangleIndex = _backupLastTriangleIndex;

                            totalDistanceChecked = distanceToMoveThisFrame;
                            _newPosition = slidePoint;
                            slidePoint = cm.Vertices[_cornerReached];
                            foundNextEdge = true;
                        }

                    }
                    else
                    {
                        EdgePoints nextTriCurrentEdgePoints = new(true);
                        // else switch to the tri on the other side of the edge
                        tempLastIndex = tempIndex;

                        tempIndex = EdgeUtils.GetOtherTriangleOnEdgeFromAdjacencyInfo(cm, e, tempIndex);

                        checkedEdges.Add(e);
                        EdgeUtils.SetOrderedEdgePointsFromTriangle(cm, e, _cornerReached, tempIndex, ref nextTriCurrentEdgePoints);

                        // check if we are pointing towards the other edge (not the next edge attached to corner)
                        // if so, we can come unstuck.
                        if (EdgeUtils.GetFarEdgeCut(cm, playerTransform, CharacterModel, _input, _cornerReached, tempIndex, ref nextTriCurrentEdgePoints, cornerEdgePoints, _movementMode))
                        {
                            _currentTriangleIndex = tempIndex;
                            _lastTriangleIndex = tempLastIndex;
                            _currentEdgePoints.Set(nextTriCurrentEdgePoints.Start, nextTriCurrentEdgePoints.End, nextTriCurrentEdgePoints.Other);
                            _onEdge = false;
                            foundNextEdge = true;
                            totalDistanceChecked = distanceToMoveThisFrame;
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
                Debug.Log("couldn't find next edge");
#endif
                return;
            }
        }
    }
    public static bool MovingTowardsEdge(ClimbableMesh cm, Vector3 _moveDirection, EdgePoints edgePoints)
    {
        Vector3 currentEdgeNormal = EdgeUtils.GetEdgeNormalFromEdgePoints(cm, edgePoints);
        return Vector3.Dot(_moveDirection, currentEdgeNormal) <= 0;
    }
    public static bool MovingTowardsEdge(ClimbableMesh cm, Vector3 _moveDirection, Edge edge)
    {
        Vector3 currentEdgeNormal = EdgeUtils.GetEdgeNormal(cm, edge);
        return Vector3.Dot(_moveDirection, currentEdgeNormal) <= 0;
    }
    public static bool CornerReached(ClimbableMesh cm, EdgePoints _currentEdgePoints, out EdgePoints cornerEdgePoints, Vector3 slidePoint)
    {
        cornerEdgePoints = new(true);

        if (slidePoint == cm.Vertices[_currentEdgePoints.Start])
            cornerEdgePoints.Set(_currentEdgePoints.Start, 0, _currentEdgePoints.End);

        if (slidePoint == cm.Vertices[_currentEdgePoints.End])
            cornerEdgePoints.Set(_currentEdgePoints.End, 0, _currentEdgePoints.Start);

        // if we reached a corner
        bool cornerReached = cornerEdgePoints.Start != -1;
        return cornerReached;
    }

    public static Vector3 GetTurnedForwardVector(Vector3 forwardVector, Vector3 groundNormal, float turnAngle)
    {
        forwardVector = Quaternion.Euler(
            Vector3.Dot(groundNormal, Vector3.right) * turnAngle,
            Vector3.Dot(groundNormal, Vector3.up) * turnAngle,
            Vector3.Dot(groundNormal, Vector3.forward) * turnAngle)

            * forwardVector;

        return forwardVector;
    }

    public static void GetGroundNormal(ClimbableMesh cm, out Vector3 groundNormal, Vector3 barycentricCoordinate, EdgePoints currentEdgePoints, int currentTriangleIndex)
    {
        // Get the 'outer' edge points to the current triangle
        EdgeUtils.GetMatchingEdgeOnAdjacentTriangle(cm, out var edgePointsAdjacentToCurrent, currentEdgePoints, currentTriangleIndex);

        // Calculate the ground normal based on coordinates from the last frame of where we should be standing, translated to the new, deformed triangle
        groundNormal = EdgeUtils.GetNormalFromBarycentric(cm, barycentricCoordinate, edgePointsAdjacentToCurrent);
    }

    public static void HandleMovementModeSwitch(
        ClimbableMesh cm,
        Transform playerTransform,
        ref EdgePoints currentEdgePoints,
        ref int currentTriangleIndex,
        ref int lastTriangleIndex,
        ref bool firstMoveDone,
        ref Vector3 testCut,
        ref Vector3 barycentricCoordinate,
        ref Vector3 lastBarycentricCoordinate,
        ref Vector3 barycentricCoordinateBehind,
        ref MovementMode currentMovementMode,
        ref Vector3 newPosition,
        ref bool onEdge
    )
    {
        MovementMode targetMovementMode;
        if (Input.GetKey(KeyCode.LeftShift))
        {
            targetMovementMode = MovementMode.Car;
        }
        else
        {
            targetMovementMode = MovementMode.Directional;
        }

        SwitchMovementMode(cm, playerTransform, ref currentEdgePoints, ref currentTriangleIndex, ref lastTriangleIndex, ref firstMoveDone, ref testCut, ref barycentricCoordinate, ref lastBarycentricCoordinate, ref barycentricCoordinateBehind, targetMovementMode, ref currentMovementMode, ref newPosition, ref onEdge);
    }

    public static void SwitchMovementMode(
        ClimbableMesh cm,
        Transform playerTransform,
        ref EdgePoints currentEdgePoints,
        ref int currentTriangleIndex,
        ref int lastTriangleIndex,
        ref bool firstMoveDone,
        ref Vector3 testCut,
        ref Vector3 barycentricCoordinate,
        ref Vector3 lastBarycentricCoordinate,
        ref Vector3 barycentricCoordinateBehind,
        MovementMode targetmovementMode,

        ref MovementMode currentMovementMode,
        ref Vector3 newPosition,
        ref bool onEdge
    )
    {
        if (currentMovementMode == targetmovementMode)
            return;

        (Vector3, Vector3, Vector3) previousEdgePointPositions = new(cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);

        int indexTemp = -3;

        if (targetmovementMode == MovementMode.Car)
        {
            indexTemp = cm.GetArea(currentTriangleIndex / 3) * 3;
        }
        else if (targetmovementMode == MovementMode.Directional)
        {
            indexTemp = cm.GetMainBody(currentTriangleIndex / 3) * 3;
            cm.RecalculateMesh(isStart: false);
        }

        if (indexTemp == -3)
            return;

        currentTriangleIndex = indexTemp;
        AssignEdgeVertexIndicesByMatchingTrianglePoints(cm, ref currentEdgePoints, previousEdgePointPositions, currentTriangleIndex);

        Vector3 triCenterTemp = (cm.Vertices[currentEdgePoints.Start] + cm.Vertices[currentEdgePoints.End] + cm.Vertices[currentEdgePoints.Other]) / 3;
        Plane plane = new Plane(-playerTransform.right, triCenterTemp);

        testCut = EdgeUtils.FindTrianglePlaneIntersection(cm, ref currentEdgePoints, currentEdgePoints, currentTriangleIndex, lastTriangleIndex, ref firstMoveDone, playerTransform.forward, triCenterTemp, plane, CutType.Start);
        lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);

        testCut = EdgeUtils.FindTrianglePlaneIntersection(cm, ref currentEdgePoints, currentEdgePoints, currentTriangleIndex, lastTriangleIndex, ref firstMoveDone, -playerTransform.forward, triCenterTemp, plane, CutType.Test);
        barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);

        if (targetmovementMode == MovementMode.Car)
        {
            barycentricCoordinate = Mathf2.GetBarycentricCoordinates(newPosition, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);
        }
        else if (targetmovementMode == MovementMode.Directional)
        {
            onEdge = false;
        }

        currentMovementMode = targetmovementMode;
    }

    public static void AssignEdgeVertexIndicesByMatchingTrianglePoints(ClimbableMesh _cm, ref EdgePoints currentEdgePoints, (Vector3, Vector3, Vector3) edgePointPositions, int index)
    {
        FindTriangleVertexIndexMatchingPosition(_cm, ref currentEdgePoints.Start, edgePointPositions.Item1, index);
        FindTriangleVertexIndexMatchingPosition(_cm, ref currentEdgePoints.End, edgePointPositions.Item2, index);
        FindTriangleVertexIndexMatchingPosition(_cm, ref currentEdgePoints.Other, edgePointPositions.Item3, index);
    }

    public static void FindTriangleVertexIndexMatchingPosition(ClimbableMesh cm, ref int pI, Vector3 matchPosition, int index)
    {
        for (int i = 0; i < 3; i++)
        {
            int currentIndex = cm.Triangles[index + i];
            if (cm.Vertices[currentIndex] == matchPosition)
            {
                pI = currentIndex;
            }
        }
    }

    public static void TryStartClimb(
        ref ClimbableMesh cm,
        Transform playerTransform,
        ref EdgePoints currentEdgePoints,
        ref int currentTriangleIndex,
        ref int lastTriangleIndex,
        ref bool firstMoveDone,
        ref Vector3 testCut,
        ref Vector3 barycentricCoordinate,
        ref Vector3 lastBarycentricCoordinate,
        ref Vector3 barycentricCoordinateBehind,

        ref Vector3 previousRaycastPosition,
        LayerMask layerMask,
        ref bool isClimbing
    )
    {
        firstMoveDone = false;
        RaycastHit hit;
        if (Physics.Raycast(playerTransform.position + playerTransform.up * 0.1f, -playerTransform.up, out hit, 0.2f, layerMask)
        || Physics.Raycast(previousRaycastPosition, (previousRaycastPosition - playerTransform.position).normalized, out hit, Vector3.Distance(previousRaycastPosition, playerTransform.position), layerMask))
        {
            GameObject temp = hit.collider.gameObject;
            cm = temp.GetComponent<ClimbableMesh>();

            isClimbing = true;
            playerTransform.position = hit.point;
            currentTriangleIndex = hit.triangleIndex * 3;
            barycentricCoordinate = hit.barycentricCoordinate;

            currentEdgePoints.Set(cm.Triangles[currentTriangleIndex], cm.Triangles[currentTriangleIndex + 1], cm.Triangles[currentTriangleIndex + 2]);

            Vector3 triCenter = EdgeUtils.GetTriangleCenter(cm, currentEdgePoints);

            Plane plane = new Plane(-playerTransform.right, triCenter);

            testCut = EdgeUtils.FindTrianglePlaneIntersection(cm, ref currentEdgePoints, currentEdgePoints, currentTriangleIndex, lastTriangleIndex, ref firstMoveDone, playerTransform.forward, triCenter, plane, CutType.Start);
            lastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);
            testCut = EdgeUtils.FindTrianglePlaneIntersection(cm, ref currentEdgePoints, currentEdgePoints, currentTriangleIndex, lastTriangleIndex, ref firstMoveDone, -playerTransform.forward, triCenter, plane, CutType.Test);
            barycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(testCut, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);
        }
        previousRaycastPosition = playerTransform.position + playerTransform.up * 0.1f;
    }

    public static void LeaveClimbableMesh(Transform playerTransform, Transform CharacterModel, Rigidbody rb, ref bool isClimbing)
    {
        // MYSTERY
        playerTransform.position = CharacterModel.position;
        rb.position = CharacterModel.position;
        rb.GetComponent<KinematicCharacterController.KinematicCharacterMotor>().SetPositionAndRotation(CharacterModel.position, CharacterModel.rotation);
        isClimbing = false;
    }
}

public class MoveContext
{

}