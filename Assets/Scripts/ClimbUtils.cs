using System.Collections;
using System.Collections.Generic;
using Overhang;
using UnityEngine;
using System;

public class ClimbUtils
{

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

            Vector3 triCenter = (cm.Vertices[currentEdgePoints.Start] + cm.Vertices[currentEdgePoints.End] + cm.Vertices[currentEdgePoints.Other]) / 3;

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