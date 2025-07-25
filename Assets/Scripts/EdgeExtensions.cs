using System.Collections;
using System.Collections.Generic;
using Overhang;
using UnityEngine;

public static class EdgeExtensions
{
    public static bool IsOuterEdge(this Edge edge, ClimbableMesh cm)
    {
        int firstTriAdjacentToEdge = cm.EdgeAdjacencyInfo[edge].triangleA;
        int secondTriAdjacentToEdge = cm.EdgeAdjacencyInfo[edge].triangleB;

        return firstTriAdjacentToEdge == secondTriAdjacentToEdge;
    }

    public static int FindVertexNotOnEdge(this ClimbableMesh climbableMesh, AdjacentEdges edges, Edge edgeToCompare)
    {
        int vertexNotOnWall = -1;
        foreach (Edge e in edges.edges)
        {
            if (climbableMesh.Vertices[e.pointA] != climbableMesh.Vertices[edgeToCompare.pointA] && climbableMesh.Vertices[e.pointA] != climbableMesh.Vertices[edgeToCompare.pointB])
            {
                vertexNotOnWall = e.pointA;
            }
            else if (climbableMesh.Vertices[e.pointB] != climbableMesh.Vertices[edgeToCompare.pointA] && climbableMesh.Vertices[e.pointB] != climbableMesh.Vertices[edgeToCompare.pointB])
            {
                vertexNotOnWall = e.pointB;
            }
        }
        return vertexNotOnWall;
    }

    public static int GetOtherTriangleOnEdge(this Edge edge, int currentTriangleIndex)
    {
        int firstTriAdjacentToEdge = edge.triangleA;
        int secondTriAdjacentToEdge = edge.triangleB;

        if (currentTriangleIndex == firstTriAdjacentToEdge)
            return secondTriAdjacentToEdge;

        if (currentTriangleIndex == secondTriAdjacentToEdge)
            return firstTriAdjacentToEdge;

        return 0;
    }

    public static bool IsIdenticalToByPosition(this Edge edge, Edge edgeToCompare, ClimbableMesh climbableMesh)
    {
        return (climbableMesh.Vertices[edge.pointA] == climbableMesh.Vertices[edgeToCompare.pointA] && climbableMesh.Vertices[edge.pointB] == climbableMesh.Vertices[edgeToCompare.pointB]) ||
               (climbableMesh.Vertices[edge.pointA] == climbableMesh.Vertices[edgeToCompare.pointB] && climbableMesh.Vertices[edge.pointB] == climbableMesh.Vertices[edgeToCompare.pointA]);
    }
}
