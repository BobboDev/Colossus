using System.Collections;
using System.Collections.Generic;
using Overhang;
using UnityEngine;

public class DebugUtils : MonoBehaviour
{
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
