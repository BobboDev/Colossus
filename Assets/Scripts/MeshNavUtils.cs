using Overhang;

public static class MeshNavUtils
{
    // public static EdgePoints GetNextTri(this ClimbableMesh cm, ref int index, int lastEdgeStart, int lastEdgeEnd)
    // {
    //     // Get the three adjacent edges of the triangle we're currently checking
    //     Edge[] adjacentEdges = cm.TriangleInfos[index].edges;

    //     foreach (Edge e in adjacentEdges)
    //     {
    //         // Get the edge of the three that is equal to the edge that has just been passed
    //         bool edgeMatches = (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeStart] && cm.Vertices[e.pointB] == cm.Vertices[lastEdgeEnd]) ||
    //             (cm.Vertices[e.pointA] == cm.Vertices[lastEdgeEnd] && cm.Vertices[e.pointB] == cm.Vertices[lastEdgeStart]);

    //         if (edgeMatches)
    //         {
    //             int nextTriangle = cm.edgeInfos[e].triangleA == cm.edgeInfos[e].triangleB
    //                 ? cm.edgeInfos[e].triangleA // If adjacent triangles are the same, return the same triangle
    //                 : cm.edgeInfos[e].triangleA != index
    //                     ? cm.edgeInfos[e].triangleA // If edge has two triangles and the first is not equal to the current triangle, return that
    //                     : cm.edgeInfos[e].triangleB; // Otherwise, return the second, that's all that's left

    //             // Set index to the new triangle
    //             index = nextTriangle;

    //             return new EdgePoints
    //             {
    //                 Start = cm.Triangles[nextTriangle],
    //                 End = cm.Triangles[nextTriangle + 1],
    //                 Other = cm.Triangles[nextTriangle + 2]
    //             };
    //         }
    //     }

    //     return default;
    // }
}
