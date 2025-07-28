using System.Collections;
using System.Collections.Generic;
using Overhang;
using UnityEngine;

public static class EdgeUtils
{
    public static bool EdgeIsOutsideEdge(this Edge edge)
    {
        int firstTriAdjacentToEdge = edge.triangleA;
        int secondTriAdjacentToEdge = edge.triangleB;

        return firstTriAdjacentToEdge == secondTriAdjacentToEdge;
    }

}