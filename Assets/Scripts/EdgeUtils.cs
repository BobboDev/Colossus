using System.Collections;
using System.Collections.Generic;
using Overhang;
using UnityEngine;
using System;

public class EdgeUtils
{
    public static Vector3 GetEdgeNormal(ClimbableMesh cm, Edge cornerEdge, out int cornerEdgeOther)
    {
        cornerEdgeOther = GetOtherVertexIndex(cm, cornerEdge);
        Vector3 triCenter = (cm.Vertices[cornerEdge.pointA] + cm.Vertices[cornerEdge.pointB] + cm.Vertices[cornerEdgeOther]) / 3;
        Vector3 edgeNormal = (triCenter - Mathf2.NearestPointOnLine(cm.Vertices[cornerEdge.pointA], (cm.Vertices[cornerEdge.pointA] - cm.Vertices[cornerEdge.pointB]).normalized, triCenter)).normalized;

        return edgeNormal;
    }

    public static void FindMatchingEdgePointsOnTriangle(int triangleIndex, EdgePoints points, ref EdgePoints pointsOut, ClimbableMesh cm)
    {
        int start = -1;
        int end = -1;

        foreach (Edge e in cm.TriangleAdjacencyInfo[triangleIndex].edges)
        {
            if (start == -1)
            {
                if (cm.Vertices[points.Start] == cm.Vertices[e.pointA])
                    start = e.pointA;
                else if (cm.Vertices[points.Start] == cm.Vertices[e.pointB])
                    start = e.pointB;
            }

            if (end == -1)
            {
                if (cm.Vertices[points.End] == cm.Vertices[e.pointA])
                    end = e.pointA;
                else if (cm.Vertices[points.End] == cm.Vertices[e.pointB])
                    end = e.pointB;
            }
        }

        // Fallbacks using 'Other' if needed
        foreach (Edge e in cm.TriangleAdjacencyInfo[triangleIndex].edges)
        {
            if (start == -1)
            {
                if (cm.Vertices[points.Other] == cm.Vertices[e.pointA])
                    start = e.pointA;
                else if (cm.Vertices[points.Other] == cm.Vertices[e.pointB])
                    start = e.pointB;
            }

            if (end == -1)
            {
                if (cm.Vertices[points.Other] == cm.Vertices[e.pointA])
                    end = e.pointA;
                else if (cm.Vertices[points.Other] == cm.Vertices[e.pointB])
                    end = e.pointB;
            }
        }

        pointsOut.Set(start, end, -1);
    }

    public static Edge FindFirstEdgeOnCorner(int cornerIndex, int triangleIndex, ClimbableMesh cm)
    {
        Edge? firstEdge = new();
        int lastTriangle = -1;

        // Limit loop to avoid infinite traversal
        for (int i = 0; i < 100; i++)
        {
            foreach (var edge in cm.EdgesAttachedToCorner[cornerIndex])
            {
                // Get the two triangles connected by this edge
                var adjacent = cm.EdgeAdjacencyInfo[edge];
                int triA = adjacent.triangleA;
                int triB = adjacent.triangleB;

                // Skip if this edge doesn't touch the current triangle
                if (triA != triangleIndex && triB != triangleIndex)
                    continue;

                // Store the first edge seen (whether or not it's outside)
                if (firstEdge == null)
                    firstEdge = edge;

                // If it's an outside edge or a degenerate (A == B), return it
                if (EdgeUtils.EdgeIsOutsideEdge(edge, cm) || triA == triB)
                    return edge;

                // Determine the next triangle to move to
                int nextTriangle = (triangleIndex == triA) ? triB : triA;

                // Prevent going back to the previous triangle (infinite loop)
                if (nextTriangle == lastTriangle)
                    continue;

                // Advance to the next triangle
                lastTriangle = triangleIndex;
                triangleIndex = nextTriangle;

                // Break to restart the foreach with the new triangle
                break;
            }
        }

        // Log a warning if we've looped too many times
        Debug.LogWarning("FindFirstEdgeOnCorner exceeded max iterations.");

        // Return default if no outside edge found (firstEdge is optional fallback)
        return default;
    }

    public static bool EdgeIsOutsideEdge(Edge edge, ClimbableMesh cm)
    {
        int firstTriAdjacentToEdge = cm.EdgeAdjacencyInfo[edge].triangleA;
        int secondTriAdjacentToEdge = cm.EdgeAdjacencyInfo[edge].triangleB;

        return firstTriAdjacentToEdge == secondTriAdjacentToEdge;
    }

    public static bool EdgesMatchByPosition(Edge edgeA, Edge edgeB, ClimbableMesh cm)
    {
        Vector3 a1 = cm.Vertices[edgeA.pointA];
        Vector3 a2 = cm.Vertices[edgeA.pointB];
        Vector3 b1 = cm.Vertices[edgeB.pointA];
        Vector3 b2 = cm.Vertices[edgeB.pointB];
        return (a1 == b1 && a2 == b2) || (a1 == b2 && a2 == b1);
    }

    public static bool EdgesMatchByPosition(Edge edgeA, EdgePoints edgePoints, ClimbableMesh cm)
    {
        Vector3 a1 = cm.Vertices[edgeA.pointA];
        Vector3 a2 = cm.Vertices[edgeA.pointB];
        Vector3 b1 = cm.Vertices[edgePoints.Start];
        Vector3 b2 = cm.Vertices[edgePoints.End];
        return (a1 == b1 && a2 == b2) || (a1 == b2 && a2 == b1);
    }

    public static int GetOtherVertexIndex(Edge cornerEdge, ClimbableMesh _cm)
    {
        foreach (Edge e in _cm.TriangleAdjacencyInfo[_cm.EdgeAdjacencyInfo[cornerEdge].triangleA].edges)
        {
            if (_cm.Vertices[e.pointA] != _cm.Vertices[cornerEdge.pointA] && _cm.Vertices[e.pointA] != _cm.Vertices[cornerEdge.pointB])
                return e.pointA;
            if (_cm.Vertices[e.pointB] != _cm.Vertices[cornerEdge.pointA] && _cm.Vertices[e.pointB] != _cm.Vertices[cornerEdge.pointB])
                return e.pointB;
        }

        return -1;
    }

    public static int GetOtherVertexIndex(EdgePoints edge, ClimbableMesh _cm)
    {
        Edge cornerEdge = new() { pointA = edge.Start, pointB = edge.End };

        foreach (Edge e in _cm.TriangleAdjacencyInfo[_cm.EdgeAdjacencyInfo[cornerEdge].triangleA].edges)
        {
            if (_cm.Vertices[e.pointA] != _cm.Vertices[cornerEdge.pointA] && _cm.Vertices[e.pointA] != _cm.Vertices[cornerEdge.pointB])
                return e.pointA;
            if (_cm.Vertices[e.pointB] != _cm.Vertices[cornerEdge.pointA] && _cm.Vertices[e.pointB] != _cm.Vertices[cornerEdge.pointB])
                return e.pointB;
        }

        return -1;
    }

    public static int GetOtherVertexIndexFromTriangle(EdgePoints cornerEdge, int triangleIndex, ClimbableMesh _cm)
    {
        foreach (Edge e in _cm.TriangleAdjacencyInfo[triangleIndex].edges)
        {
            if (_cm.Vertices[e.pointA] != _cm.Vertices[cornerEdge.Start] && _cm.Vertices[e.pointA] != _cm.Vertices[cornerEdge.End])
                return e.pointA;
            if (_cm.Vertices[e.pointB] != _cm.Vertices[cornerEdge.Start] && _cm.Vertices[e.pointB] != _cm.Vertices[cornerEdge.End])
                return e.pointB;
        }

        return -1;
    }

    public static bool VertexPositionsAreMatching(int vertex1, int vertex2, ClimbableMesh cm)
    {
        return cm.Vertices[vertex1] == cm.Vertices[vertex2];
    }

    public static void GetMatchingEdgeOnAdjacentTriangle(ClimbableMesh cm, ref EdgePoints target, EdgePoints source, int triangleIndex)
    {
        Edge[] adjacentEdges = cm.TriangleAdjacencyInfo[triangleIndex].edges;
        List<Vector3> vertices = cm.Vertices;

        TryMatchEdgePointFromAdjacency(ref target.Start, source.Start, adjacentEdges, vertices);
        TryMatchEdgePointFromAdjacency(ref target.End, source.End, adjacentEdges, vertices);
        TryMatchEdgePointFromAdjacency(ref target.Other, source.Other, adjacentEdges, vertices);
    }


    public static void TryMatchEdgePointFromAdjacency(ref int targetIndex, int targetVertexIndex, Edge[] edges, List<Vector3> vertices)
    {
        Vector3 targetPosition = vertices[targetVertexIndex];

        foreach (Edge e in edges)
        {
            if (vertices[e.pointA] == targetPosition)
            {
                targetIndex = e.pointA;
                return;
            }
            else if (vertices[e.pointB] == targetPosition)
            {
                targetIndex = e.pointB;
                return;
            }
        }
    }

    public static int GetOtherTriangleOnEdge(Edge edge, int currentTriangleIndex)
    {
        int firstTriAdjacentToEdge = edge.triangleA;
        int secondTriAdjacentToEdge = edge.triangleB;

        if (currentTriangleIndex == firstTriAdjacentToEdge)
            return secondTriAdjacentToEdge;

        if (currentTriangleIndex == secondTriAdjacentToEdge)
            return firstTriAdjacentToEdge;

        return 0;
    }

    public static Vector3 GetPositionFromBarycentric(ClimbableMesh cm, Vector3 barycentricCoordinate, EdgePoints edgePoints)
    {
        return barycentricCoordinate.x * cm.Vertices[edgePoints.Start] +
               barycentricCoordinate.y * cm.Vertices[edgePoints.End] +
               barycentricCoordinate.z * cm.Vertices[edgePoints.Other];
    }
    public static Vector3 GetNormalFromBarycentric(ClimbableMesh cm, Vector3 barycentricCoordinate, EdgePoints edgePoints)
    {
        return (barycentricCoordinate.x * cm.Normals[edgePoints.Start] +
                barycentricCoordinate.y * cm.Normals[edgePoints.End] +
                barycentricCoordinate.z * cm.Normals[edgePoints.Other]).normalized;
    }

    public static Ray CreateRay(Vector3 point1, Vector3 point2) => new Ray { origin = point1, direction = (point2 - point1).normalized };

    public static bool GetFarEdgeCut(ClimbableMesh cm, Transform playerTransform, Vector3 input, int currentCornerInt, int triangleIndex, EdgePoints currentEdgePoints, EdgePoints nextTriCurrentEdgePoints, EdgePoints cornerEdgePoints, Plane plane)
    {
        Vector3 triCenter = (cm.Vertices[cornerEdgePoints.Start] + cm.Vertices[cornerEdgePoints.End] + cm.Vertices[cornerEdgePoints.Other]) / 3;
        Vector3 closestPointOnEdge = Mathf2.NearestPointOnLine(cm.Vertices[cornerEdgePoints.Start], (cm.Vertices[cornerEdgePoints.Start] - cm.Vertices[cornerEdgePoints.End]).normalized, triCenter);
        Vector3 nextTriCenter = (cm.Vertices[nextTriCurrentEdgePoints.Start] + cm.Vertices[nextTriCurrentEdgePoints.End] + cm.Vertices[cornerEdgePoints.Other]) / 3;
        Vector3 nextTriClosestPointOnEdge = Mathf2.NearestPointOnLine(cm.Vertices[nextTriCurrentEdgePoints.Start], (cm.Vertices[nextTriCurrentEdgePoints.Start] - cm.Vertices[nextTriCurrentEdgePoints.End]).normalized, nextTriCenter);

        Vector3 previousTriCenter = (cm.Vertices[currentEdgePoints.Start] + cm.Vertices[currentEdgePoints.End] + cm.Vertices[currentEdgePoints.Other]) / 3;

        Vector3 cornerNormal = Vector3.Cross(cm.Vertices[cornerEdgePoints.Start] - cm.Vertices[cornerEdgePoints.Other], cm.Vertices[cornerEdgePoints.Start] - cm.Vertices[cornerEdgePoints.End]).normalized;
        if (Vector3.Dot(cornerNormal, playerTransform.up) < 0)
        {
            cornerNormal = -cornerNormal;
        }
        Vector3 edgeNormal = (closestPointOnEdge - triCenter).normalized;
        Vector3 nextTriEdgeNormal = (nextTriClosestPointOnEdge - nextTriCenter).normalized;
        Vector3 edgeNormal2 = Vector3.ProjectOnPlane(edgeNormal, cornerNormal).normalized;
        Vector3 nextTriEdgeNormal2 = Vector3.ProjectOnPlane(nextTriEdgeNormal, playerTransform.up).normalized;

        bool planeHitsFarEdge = false;

        // if move direction is facing into the wall that we're sliding on
        Vector3 cornerAdjustedMoveDirection = Vector3.ProjectOnPlane((playerTransform.rotation * input).normalized, cornerNormal).normalized;
        if (Vector3.Dot((playerTransform.rotation * input).normalized, nextTriEdgeNormal2) < 0 && Vector3.Dot(cornerAdjustedMoveDirection, edgeNormal2) < 0)
        {
            foreach (Edge e in cm.TriangleAdjacencyInfo[triangleIndex].edges)
            {
                if (cm.Vertices[e.pointA] != cm.Vertices[currentCornerInt] && cm.Vertices[e.pointB] != cm.Vertices[currentCornerInt])
                {
                    Ray farEdgeRay = EdgeUtils.CreateRay(cm.Vertices[e.pointA], cm.Vertices[e.pointB]);
                    if (plane.Raycast(farEdgeRay, out var farEdgeHitDistance))
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

    // Updates the last edge state variables if not in Test mode.
    public static void UpdateEdgeState(ref EdgePoints _currentEdgePoints, (int start, int end, int other) edge, CutType cutType,
                                ref bool hasIntersection, bool isFirstEdge)
    {
        if (cutType != CutType.Test)
        {
            _currentEdgePoints.Set(edge.start, edge.end, edge.other);

            if (!isFirstEdge) hasIntersection = true;
        }
    }

    public static bool DoRaysIntersect(Ray ray1, Ray ray2)
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

    Vector3 GetAdjustedNormal(ClimbableMesh cm, int normalIndex)
    {
        return cm.transform.TransformDirection(cm.Normals[normalIndex]);
    }

    // Computes ray data (ray, length, intersection distance) for each edge against the plane.
    public static (Ray ray, float length, float hitDistance)[] ComputeRayDataForEdges(ClimbableMesh cm, (int start, int end, int other)[] edges, Plane plane)
    {
        var rayData = new (Ray ray, float length, float hitDistance)[3];
        for (int i = 0; i < 3; i++)
        {
            Vector3 startPos = cm.Vertices[edges[i].start];
            Vector3 endPos = cm.Vertices[edges[i].end];
            Vector3 rayDirection = (endPos - startPos).normalized;
            Ray ray = new Ray(startPos, rayDirection);
            float edgeLength = Vector3.Distance(startPos, endPos);
            plane.Raycast(ray, out float hitDistance);
            rayData[i] = (ray, edgeLength, hitDistance);
        }
        return rayData;
    }

    // Orders triangle vertices based on the cut type, aligning with the previous edge for CutType.Next.
    public static (int vertexA, int vertexB, int vertexC) OrderVerticesForCut(ClimbableMesh cm, EdgePoints currentEdgePoints, int currentTriangleIndex, int lastTriangleIndex, int p1, int p2, int p3, CutType cutType)
    {
        if (cutType == CutType.Next && lastTriangleIndex != currentTriangleIndex)
        {
            int vertexA = MatchVertexByPosition(cm, currentEdgePoints.Start, p1, p2, p3); // Start matches previous edge start
            int vertexC = MatchVertexByPosition(cm, currentEdgePoints.End, p1, p2, p3);   // End matches previous edge end
            int vertexB = (p1 != vertexA && p1 != vertexC) ? p1 :
                          (p2 != vertexA && p2 != vertexC) ? p2 : p3; // Middle is the remaining vertex
            return (vertexA, vertexB, vertexC);
        }
        return (p1, p2, p3); // Default order for Start or Test
    }
    // Updates last edge indices to match the current triangle's vertices.
    public static void UpdateEdgeIndices(ClimbableMesh cm, ref EdgePoints _currentEdgePoints, int p1, int p2, int p3)
    {
        _currentEdgePoints.Set(
            MatchVertexByPosition(cm, _currentEdgePoints.Start, p1, p2, p3),
            MatchVertexByPosition(cm, _currentEdgePoints.End, p1, p2, p3),
            MatchVertexByPosition(cm, _currentEdgePoints.Other, p1, p2, p3)
        );
    }

    // Finds the vertex index among p1, p2, p3 that matches the target vertex by position.
    public static int MatchVertexByPosition(ClimbableMesh cm, int target, int p1, int p2, int p3)
    {
        if (cm.Vertices[p1] == cm.Vertices[target]) return p1;
        if (cm.Vertices[p2] == cm.Vertices[target]) return p2;
        if (cm.Vertices[p3] == cm.Vertices[target]) return p3;
        return p1; // Fallback to p1 if no match (assumes a match should exist)
    }


    public static int GetOtherVertexIndex(ClimbableMesh cm, Edge cornerEdge)
    {
        foreach (Edge e in cm.TriangleAdjacencyInfo[cm.EdgeAdjacencyInfo[cornerEdge].triangleA].edges)
        {
            if (cm.Vertices[e.pointA] != cm.Vertices[cornerEdge.pointA] && cm.Vertices[e.pointA] != cm.Vertices[cornerEdge.pointB])
                return e.pointA;
            if (cm.Vertices[e.pointB] != cm.Vertices[cornerEdge.pointA] && cm.Vertices[e.pointB] != cm.Vertices[cornerEdge.pointB])
                return e.pointB;
        }

        return -1; // Fallback if not found
    }

    // Determines if an intersection point should be accepted based on cut type and direction.
    public static bool ShouldAcceptIntersection(CutType cutType, Vector3 direction,
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

    // Validates an intersection, including extended conditions beyond the edge length.
    public static bool IsIntersectionValid(float hitDistance, float rayLength,
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

    // Processes an edge to find and validate an intersection with the plane, updating the result if accepted.
    public static void ProcessEdgeIntersection(ClimbableMesh cm, ref EdgePoints currentEdgePoints, int currentTriangleIndex, int lastTriangleIndex, (int start, int end, int other) edge,
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
                    UpdateEdgeState(ref currentEdgePoints, edge, cutType, ref hasIntersection, isFirstEdge);
                }
            }
        }
        // Case 2: Ray starts on the plane (hitDistance = 0), use start vertex under specific conditions
        else if (hitDistance == 0 && cutType != CutType.Test && lastTriangleIndex != currentTriangleIndex &&
                 (isFirstEdge || !hasIntersection))
        {
            result = cm.Vertices[edge.start];
            UpdateEdgeState(ref currentEdgePoints, edge, cutType, ref hasIntersection, isFirstEdge);
        }
    }

    /// Finds the next intersection point between a plane and a triangle's edges as part of a cutting operation.
    public static Vector3 FindTrianglePlaneIntersection(ClimbableMesh cm, ref EdgePoints currentEdgePoints, EdgePoints targetEdgePoints, int currentTriangleIndex, int lastTriangleIndex, ref bool firstMoveDone, Vector3 direction, Vector3 position, Plane plane, CutType cutType)
    {
        // Update tracking of the previous edge when moving to a new triangle (except in Test or Start modes)
        if (currentTriangleIndex != lastTriangleIndex && cutType != CutType.Test && cutType != CutType.Start)
        {
            EdgeUtils.UpdateEdgeIndices(cm, ref currentEdgePoints, targetEdgePoints.Start, targetEdgePoints.End, targetEdgePoints.Other);
        }

        // Reorder vertices for CutType.Next to ensure continuity with the previous edge
        (int vertexA, int vertexB, int vertexC) = EdgeUtils.OrderVerticesForCut(cm, currentEdgePoints, currentTriangleIndex, lastTriangleIndex, targetEdgePoints.Start, targetEdgePoints.End, targetEdgePoints.Other, cutType);

        // Define the triangle's edges: each tuple represents (start vertex, end vertex, opposite vertex)
        var edges = new[]
        {
        (start: vertexA, end: vertexB, other: vertexC), // Edge A -> B, opposite C
        (start: vertexB, end: vertexC, other: vertexA), // Edge B -> C, opposite A
        (start: vertexA, end: vertexC, other: vertexB)  // Edge A -> C, opposite B
    };

        // Precompute ray data for each edge (ray, length, and plane intersection distance)
        var edgeRayData = EdgeUtils.ComputeRayDataForEdges(cm, edges, plane);

        // Select edges to process: only the first two for Next (new edges), all three otherwise
        int edgesToProcessCount = cutType == CutType.Next ? 2 : 3;

        Vector3 result = position; // Default return value if no intersection is found
        bool hasIntersection = false;

        // In GetNextCut method, replace the loop inside the if (firstMoveDone) block:
        if (firstMoveDone)
        {
            for (int i = 0; i < edgesToProcessCount; i++)
            {
                EdgeUtils.ProcessEdgeIntersection(cm, ref currentEdgePoints, currentTriangleIndex, lastTriangleIndex, edges[i], edgeRayData[i], edges, edgeRayData,
                                       direction, position, cutType,
                                       ref result, ref hasIntersection, isFirstEdge: i == 0);
            }
        }

        firstMoveDone = true; // Mark that the first move has occurred for subsequent calls
        return result;
    }

    public static int[] GetNextTri(ClimbableMesh cm, ref int currentTriangleIndex, EdgePoints currentEdgePoints)
    {
        // Get the three adjacent edges of the triangle we're currently checking
        Edge[] adjacentEdges = cm.TriangleAdjacencyInfo[currentTriangleIndex].edges;

        foreach (Edge e in adjacentEdges)
        {
            // Get the edge of the three that is equal to the edge that has just been passed
            bool edgeMatches = (cm.Vertices[e.pointA] == cm.Vertices[currentEdgePoints.Start] && cm.Vertices[e.pointB] == cm.Vertices[currentEdgePoints.End]) ||
                (cm.Vertices[e.pointA] == cm.Vertices[currentEdgePoints.End] && cm.Vertices[e.pointB] == cm.Vertices[currentEdgePoints.Start]);

            if (edgeMatches)
            {
                int nextTriangle = EdgeUtils.EdgeIsOutsideEdge(e, cm)
                    ? cm.EdgeAdjacencyInfo[e].triangleA // If adjacent triangles are the same, return the same triangle
                    : cm.EdgeAdjacencyInfo[e].triangleA != currentTriangleIndex
                        ? cm.EdgeAdjacencyInfo[e].triangleA // If edge has two triangles and the first is not equal to the current triangle, return that
                        : cm.EdgeAdjacencyInfo[e].triangleB; // Otherwise, return the second, that's all that's left

                // Set index to the new triangle
                currentTriangleIndex = nextTriangle;

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

    public static void AssignEdgeVertexIndicesByMatchingTrianglePoints(ClimbableMesh cm, EdgePoints _currentEdgePoints, (Vector3, Vector3, Vector3) edgePointPositions, int index)
    {
        if (cm == null)
            return;
        FindTriangleVertexIndexMatchingPosition(cm, ref _currentEdgePoints.Start, edgePointPositions.Item1, index);
        FindTriangleVertexIndexMatchingPosition(cm, ref _currentEdgePoints.End, edgePointPositions.Item2, index);
        FindTriangleVertexIndexMatchingPosition(cm, ref _currentEdgePoints.Other, edgePointPositions.Item3, index);
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

    public static bool IsTriAfterNextThis(ClimbableMesh cm, Transform playerTransform, Vector3 currentCheckPosition, EdgePoints currentEdgePoints, int currentTriangleIndex, int lastTriangleIndex, bool firstMoveDone, int currentCornerIndex, Vector3 input)
    {
        EdgePoints edgeAdjacentToCurrent = new();

        Vector3 tempBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(currentCheckPosition, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);

        Vector3 tempTriCenter = (cm.Vertices[currentEdgePoints.Start] + cm.Vertices[currentEdgePoints.End] + cm.Vertices[currentEdgePoints.Other]) / 3;

        EdgeUtils.GetMatchingEdgeOnAdjacentTriangle(cm, ref edgeAdjacentToCurrent, currentEdgePoints, currentTriangleIndex);

        Vector3 tempGroundNormal = EdgeUtils.GetNormalFromBarycentric(cm, tempBarycentricCoordinate, edgeAdjacentToCurrent);

        Vector3 tempCastDirectionTest = Quaternion.FromToRotation(playerTransform.up, tempGroundNormal) * playerTransform.forward;

        Plane tempTestPlane = new Plane(-(Quaternion.FromToRotation(playerTransform.up, tempGroundNormal) * playerTransform.right), tempTriCenter);

        Vector3 tempTestCut = Vector3.zero;
        tempTestCut = EdgeUtils.FindTrianglePlaneIntersection(cm, ref currentEdgePoints, currentEdgePoints, currentTriangleIndex, lastTriangleIndex, ref firstMoveDone, tempCastDirectionTest, tempTriCenter, tempTestPlane, CutType.Test);

        // Get the barycentric coordinate of the place that would have pointed forward, for next time - recalculating animation
        Vector3 tempLastBarycentricCoordinate = Mathf2.GetBarycentricCoordinates(tempTestCut, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);
        tempTestCut = EdgeUtils.FindTrianglePlaneIntersection(cm, ref currentEdgePoints, currentEdgePoints, currentTriangleIndex, lastTriangleIndex, ref firstMoveDone, -tempCastDirectionTest, tempTriCenter, tempTestPlane, CutType.Test);
        Vector3 tempBarycentricCoordinateBehind = Mathf2.GetBarycentricCoordinates(tempTestCut, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End], cm.Vertices[currentEdgePoints.Other]);
        Vector3 behindPointOnTriangle = EdgeUtils.GetPositionFromBarycentric(cm, tempBarycentricCoordinateBehind, currentEdgePoints);
        // At the end of last loop we do a 'test' cut to get the next position in FRONT
        // Here we recalculate it with deformations. This is the forward cut, NOT the movement direction cut 
        // This is also recorded from the tri center
        Vector3 forwardPointOnTriangle = EdgeUtils.GetPositionFromBarycentric(cm, tempLastBarycentricCoordinate, currentEdgePoints);

        // Calculate the the direction towards the FORWARD facing point, NOT the movement facing point
        Vector3 tempForward = (forwardPointOnTriangle - behindPointOnTriangle).normalized;

        Quaternion tempRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(tempForward, tempGroundNormal), tempGroundNormal); // set rotation to be towards the forward point.

        Vector3 tempMovePositionAttempt = cm.Vertices[currentCornerIndex] + (tempRotation * input).normalized;

        Vector3 slidePoint = Mathf2.GetClosestPointOnFiniteLine(tempMovePositionAttempt, cm.Vertices[currentEdgePoints.Start], cm.Vertices[currentEdgePoints.End]);
        // This is for stopping the check from bouncing between edges when stuck in a corner
        // Problem is that it's sticking to corners when the GetClosestPointOnLine attemptedMovePosition is outside the line

        // In order to determine if we should be able to move round the corner, we need to know if it's a convex corner or not
        // DoRaysIntersect of the edge normals determines this.
        // If they do, then we check if the closest point on each edge to the attempted move position is equal to the corner - that's the only situation where we should move round the corner  

        if (slidePoint == cm.Vertices[currentCornerIndex])
        {
            Debug.Log("tri repeat");
            return true;
        }
        else
        {
            // Debug.DrawLine(cm.meshVerts[cornerReached], tempMovePositionAttempt, Color.magenta);
            return false;
        }
    }

    public static void DebugTriangle(ClimbableMesh cm, int triangle, Color color)
    {
        if (cm.EdgeAdjacencyInfo[cm.TriangleAdjacencyInfo[triangle].edges[0]].triangleB != -1)
            Debug.DrawLine(cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[0].pointA], cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[0].pointB], color, 0);

        if (cm.EdgeAdjacencyInfo[cm.TriangleAdjacencyInfo[triangle].edges[1]].triangleB != -1)
            Debug.DrawLine(cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[1].pointA], cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[1].pointB], color, 0);

        if (cm.EdgeAdjacencyInfo[cm.TriangleAdjacencyInfo[triangle].edges[2]].triangleB != -1)
            Debug.DrawLine(cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[2].pointA], cm.Vertices[cm.TriangleAdjacencyInfo[triangle].edges[2].pointB], color, 0);
    }
}

public enum CutType
{
    Start,
    Next,
    Test
}

public enum MovementMode
{
    Car,
    Directional
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