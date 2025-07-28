using System.Collections;
using System.Collections.Generic;
using Overhang;
using UnityEngine;

public class EdgeExtensions
{
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
        Edge? firstEdge = null;
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
                if (EdgeExtensions.EdgeIsOutsideEdge(edge, cm) || triA == triB)
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