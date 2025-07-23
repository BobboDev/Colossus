using Overhang;
using UnityEditor;
using UnityEngine;

public static class ClimbableMeshExtensions
{
    public static Edge CreateNewEdge(this ClimbableMesh climbableMesh, int pointA, int pointB)
    {
        return new Edge() { pointA = pointA, pointB = pointB };
    }

    public static Edge GetEdgeFromVertexIndices(this ClimbableMesh climbableMesh, int pointA, int pointB)
    {
        Edge edge = climbableMesh.CreateNewEdge(pointA, pointB);

        if (!climbableMesh.EdgeAdjacencyInfo.ContainsKey(edge))
        {
            edge = climbableMesh.CreateNewEdge(pointB, pointA);
        }

        if (!edge.EdgeExists())
        {
#if UNITY_EDITOR
            EditorApplication.isPaused = true;
            Debug.LogError("Edge Doesn't Exist");
#endif
        }

        return edge;
    }

    public static bool EdgeExists(this Edge edge)
    {
        return !(edge.pointA == -1 || edge.pointB == -1);
    }

    public static Edge GetOppositeWallOnCorner(this ClimbableMesh climbableMesh, int cornerIndex, int triangleIndex, Edge firstEdgeCrossed)
    {
        int currentCheckingTriangleInCorner = triangleIndex;
        bool edgeFound = false;
        int trianglePreviouslyChecked = -1;
        int timesLooped = 0;

        // CONTEXT
        // Iterate over edges attached to corner until we reach an edge to slide along

        while (!edgeFound)
        {
            foreach (Edge cornerEdgeBeingChecked in climbableMesh.EdgesAttachedToCorner[cornerIndex])
            {
                if (cornerEdgeBeingChecked.EdgeIsOutsideEdge())
                {
                    edgeFound = true;
                    return cornerEdgeBeingChecked;
                }
                else
                {
                    // the next triangle to check 
                    int triangleOnOtherSideOfEdge = cornerEdgeBeingChecked.GetOtherTriangleOnEdge(currentCheckingTriangleInCorner);

                    // If the corner being checked is not the first edge we crossed
                    if (!cornerEdgeBeingChecked.IsIdenticalToByPosition(firstEdgeCrossed, climbableMesh))
                    {
                        // And the triangle isn't the one we just checked
                        if (trianglePreviouslyChecked != triangleOnOtherSideOfEdge)
                        {
                            trianglePreviouslyChecked = currentCheckingTriangleInCorner;
                            currentCheckingTriangleInCorner = triangleOnOtherSideOfEdge;
                        }
                    }
                }
            }
            timesLooped++;
            if (timesLooped > 100)
            {
                Debug.LogError("Couldn't find other edge in corner");
                break;
            }
        }

        return new();
    }
}
