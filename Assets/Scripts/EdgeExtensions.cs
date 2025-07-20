using System.Collections;
using System.Collections.Generic;
using Overhang;
using UnityEngine;

public static class EdgeExtensions
{
    public static bool EdgeIsOutsideEdge(this Edge edge)
    {
        int firstTriAdjacentToEdge = edge.triangleA;
        int secondTriAdjacentToEdge = edge.triangleB;

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

    public static bool IsIdenticalToByPosition(this Edge edge, Edge edgeToCompare)
    {
        Vector3 firstVertexPositionOfEdge = edge.climbableMesh.Vertices[edge.pointA];
        Vector3 secondVertexPositionOfEdge = edge.climbableMesh.Vertices[edge.pointB];

        Vector3 firstVertexPositionOfEdgeToCompare = edge.climbableMesh.Vertices[edgeToCompare.pointA];
        Vector3 secondVertexPositionOfEdgeToCompare = edge.climbableMesh.Vertices[edgeToCompare.pointB];

        bool pointAIsPointACompare = firstVertexPositionOfEdge == firstVertexPositionOfEdgeToCompare;
        bool pointBIsPointBCompare = secondVertexPositionOfEdge == secondVertexPositionOfEdgeToCompare;

        bool pointAIsPointBCompare = firstVertexPositionOfEdge == secondVertexPositionOfEdgeToCompare;
        bool pointBIsPointACompare = secondVertexPositionOfEdge == firstVertexPositionOfEdgeToCompare;

        bool edgesAreIdentical = pointAIsPointACompare && pointBIsPointBCompare || pointAIsPointBCompare && pointBIsPointACompare;

        return edgesAreIdentical;
    }
}
