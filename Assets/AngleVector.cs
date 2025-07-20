using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Overhang;
public class AngleVector : MonoBehaviour
{
    public static Vector3 GetTriangleDeformedForwardVector(ClimbableMesh cm, int triangleIndex, Vector3 barycentricCoordinates, float angle, Transform t)
    {

        int[] tris = new int[] { 0, 1, 2 };
        Vector3[] tempVerts = new Vector3[] { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };

        Debug.Log(cm.Vertices.Count);
        for (int i = 0; i < 3; i++)
        {
            tempVerts[i] = cm.Vertices[cm.Triangles[triangleIndex + i]];
        }

        // 'rotate' the Barycentric coordinate by desired angle
        Vector3 adjustedBarycentric = Quaternion.Euler(angle * barycentricCoordinates.x,
                                                        angle * barycentricCoordinates.y,
                                                        angle * barycentricCoordinates.z) * Vector3.forward;

        // Add the vector two sides together to get the corner's 'normal'
        Vector3[] cornerNormals = new Vector3[]{ -( (tempVerts[2] - tempVerts[0]) + (tempVerts[1] - tempVerts[0]) ),
                                                 -( (tempVerts[0] - tempVerts[1]) + (tempVerts[2] - tempVerts[1]) ),
                                                 -( (tempVerts[1] - tempVerts[2]) + (tempVerts[0] - tempVerts[2]) )};

        Debug.Log("1 " + adjustedBarycentric);
        // Calculate output vector
        Vector3 adjustedDirection = adjustedBarycentric.x * cornerNormals[0] +
                                    adjustedBarycentric.y * cornerNormals[1] +
                                    adjustedBarycentric.z * cornerNormals[2];

        adjustedBarycentric = cornerNormals[0] / adjustedDirection.x +
                              cornerNormals[1] / adjustedDirection.x +
                              cornerNormals[2] / adjustedDirection.x;

        Debug.Log("2 " + adjustedDirection);
        Debug.Log("3 " + adjustedBarycentric);

        Debug.DrawLine(t.position, t.position + adjustedDirection, Color.green);

        return adjustedDirection;

    }
    // float GetAngleOfTri(Vector3 baryCentric)
    // {

    //     float tempAngle;

    //     tempAngle = Quaternion.Euler( barycentricCoordinates.x * Vector3.forward,
    //                                   barycentricCoordinates.y * Vector3.forward,
    //                                   barycentricCoordinates.z * Vector3.forward );


    //     return tempAngle;
    // }
    public static Vector3 GetTriangleDeformedForwardVector(ClimbableMesh cm, int p1, int p2, int p3, Vector3 barycentricCoordinates, float angle, Transform t)
    {

        int[] tris = new int[] { 0, 1, 2 };
        Vector3[] tempVerts = new Vector3[] { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };

        Debug.Log(cm.Vertices.Count);

        tempVerts[0] = cm.Vertices[cm.Triangles[p1]];
        tempVerts[1] = cm.Vertices[cm.Triangles[p2]];
        tempVerts[2] = cm.Vertices[cm.Triangles[p3]];

        // 'rotate' the Barycentric coordinate by desired angle
        Vector3 adjustedBarycentric = Quaternion.Euler(angle * barycentricCoordinates.x,
                                                        angle * barycentricCoordinates.y,
                                                        angle * barycentricCoordinates.z) * Vector3.forward;

        // Add the vector two sides together to get the corner's 'normal'
        Vector3[] cornerNormals = new Vector3[]{ -( (tempVerts[2] - tempVerts[0]) + (tempVerts[1] - tempVerts[0]) ),
                                                 -( (tempVerts[0] - tempVerts[1]) + (tempVerts[2] - tempVerts[1]) ),
                                                 -( (tempVerts[1] - tempVerts[2]) + (tempVerts[0] - tempVerts[2]) )};

        Debug.Log("1 " + adjustedBarycentric);
        // Calculate output vector
        Vector3 adjustedDirection = adjustedBarycentric.x * cornerNormals[0] +
                                    adjustedBarycentric.y * cornerNormals[1] +
                                    adjustedBarycentric.z * cornerNormals[2];

        Debug.DrawLine(t.position, t.position + adjustedDirection, Color.green);

        return adjustedDirection;

    }

    public static float GetBarycentricAngle(ClimbableMesh cm, Vector3 normal, Vector3 barycentricCoordinates, Vector3 forward, Transform t)
    {

        int[] tris = new int[] { 0, 1, 2 };
        Vector3[] tempVerts = new Vector3[] { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };

        float tempAngle = 0;

        // Debug.Log(cm.meshVerts.Count);
        // for (int i = 0; i < 3; i++)
        // {
        //     tempVerts[i] = cm.meshVerts[cm.triangles[triangleIndex + i]];
        // }

        Vector3 reverseBarycentric = Quaternion.Inverse(Quaternion.LookRotation(forward, normal)) * barycentricCoordinates;

        Debug.Log("FLEH " + reverseBarycentric);
        // Vector3 fleh = 

        // Debug.DrawLine(t.position,t.position + adjustedDirection,Color.green); 

        // tempAngle = reverseBarycentric;
        Debug.Log(tempAngle);
        return tempAngle;

    }

}