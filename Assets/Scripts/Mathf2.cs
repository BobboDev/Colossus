using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public static class Mathf2
{
    public static Vector3 NearestPointOnLine(Vector3 origin, Vector3 direction, Vector3 point)
    {
        direction.Normalize();//this needs to be a unit vector
        var v = point - origin;
        var d = Vector3.Dot(v, direction);
        return origin + direction * d;
    }
    
    public static Vector3 GetClosestPointOnFiniteLine(Vector3 point, Vector3 line_start, Vector3 line_end)
    {
        Vector3 line_direction = line_end - line_start;
        float line_length = line_direction.magnitude;
        line_direction.Normalize();
        float project_length = Mathf.Clamp(Vector3.Dot(point - line_start, line_direction), 0f, line_length);
        return line_start + line_direction * project_length;
    }

    public static Vector3 GetClosestPointOnInfiniteLine(Vector3 lineStart, Vector3 lineEnd, Vector3 point)
    {
        Vector3 lineDirection = (lineEnd - lineStart).normalized;
        Vector3 pointDirection = point - lineStart;

        float dotProduct = Vector3.Dot(pointDirection, lineDirection);
        return lineStart + lineDirection * dotProduct;
    }

    public static Vector3 ClosestPointOnTriangle( Vector3 p1, Vector3 p2, Vector3 p3, Vector3 point )
    {
        Vector3 edge0 = p2 - p1;
        Vector3 edge1 = p3 - p1;
        Vector3 v0 = p1 - point;

        float a = Vector3.Dot( edge0, edge0 );
        float b = Vector3.Dot( edge0, edge1 );
        float c = Vector3.Dot( edge1, edge1 );
        float d = Vector3.Dot( edge0, v0 );
        float e = Vector3.Dot( edge1, v0 );

        float det = a*c - b*b;
        float s = b*e - c*d;
        float t = b*d - a*e;

        if ( s + t < det )
        {
            if ( s < 0f )
            {
                if ( t < 0f )
                {
                    if ( d < 0f )
                    {
                        s = Mathf.Clamp( -d/a, 0f, 1f );
                        t = 0f;
                    }
                    else
                    {
                        s = 0f;
                        t = Mathf.Clamp( -e/c, 0f, 1f );
                    }
                }
                else
                {
                    s = 0f;
                    t = Mathf.Clamp( -e/c, 0f, 1f );
                }
            }
            else if ( t < 0f )
            {
                s = Mathf.Clamp( -d/a, 0f, 1f );
                t = 0f;
            }
            else
            {
                float invDet = 1f / det;
                s *= invDet;
                t *= invDet;
            }
        }
        else
        {
            if ( s < 0f )
            {
                float tmp0 = b+d;
                float tmp1 = c+e;
                if ( tmp1 > tmp0 )
                {
                    float numer = tmp1 - tmp0;
                    float denom = a-2*b+c;
                    s = Mathf.Clamp( numer/denom, 0f, 1f );
                    t = 1-s;
                }
                else
                {
                    t = Mathf.Clamp( -e/c, 0f, 1f );
                    s = 0f;
                }
            }
            else if ( t < 0f )
            {
                if ( a+d > b+e )
                {
                    float numer = c+e-b-d;
                    float denom = a-2*b+c;
                    s = Mathf.Clamp( numer/denom, 0f, 1f );
                    t = 1-s;
                }
                else
                {
                    s = Mathf.Clamp( -e/c, 0f, 1f );
                    t = 0f;
                }
            }
            else
            {
                float numer = c+e-b-d;
                float denom = a-2*b+c;
                s = Mathf.Clamp( numer/denom, 0f, 1f );
                t = 1f - s;
            }
        }

        return p1 + s * edge0 + t * edge1;
    }


    public static Vector3 PlanesIntersection( Plane p0, Plane p1, Plane p2)
    {
        const float EPSILON = 1e-4f;

        var det = Vector3.Dot( Vector3.Cross( p0.normal, p1.normal ), p2.normal );
        if (Mathf.Abs(det) < EPSILON)
        {
            return Vector3.zero;
        }

        Vector3 intersectionPoint = 
            ( -( p0.distance * Vector3.Cross( p1.normal, p2.normal ) ) -
            ( p1.distance * Vector3.Cross( p2.normal, p0.normal ) ) -
            ( p2.distance * Vector3.Cross( p0.normal, p1.normal ) ) ) / det;

        return intersectionPoint;
    }


    public static Vector3 GetBarycentricCoordinates(Vector3 f, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 v0 = b - a, v1 = c - a, v2 = f - a;
        float d00 = Vector3.Dot(v0, v0);
        float d01 = Vector3.Dot(v0, v1);
        float d11 = Vector3.Dot(v1, v1);
        float d20 = Vector3.Dot(v2, v0);
        float d21 = Vector3.Dot(v2, v1);
        float denom = d00 * d11 - d01 * d01;
        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;

        return new Vector3 (u,v,w);
    }

    // Compute barycentric coordinates (u, v, w) for
    // point p with respect to triangle (a, b, c)
    // public static Vector3 GetBarycentricCoordinates(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    // {
    //     Vector2 v0 = b - a, v1 = c - a, v2 = p - a;
    //     float den = v0.x * v1.y - v1.x * v0.y;
    //     float v = (v2.x * v1.y - v1.x * v2.y) / den;
    //     float w = (v0.x * v2.y - v2.x * v0.y) / den;
    //     float u = 1.0f - v - w;

    //     return new Vector3 (u,v,w);
    // }

    // Function to convert a Vector2 to an angle in degrees
    public static float VectorToAngle(Vector2 vector)
    {
        // Calculate the angle in radians using Mathf.Atan2
        float angleRadians = Mathf.Atan2(vector.y, vector.x);

        // Convert radians to degrees
        float angleDegrees = angleRadians * Mathf.Rad2Deg;

        // Ensure the angle is positive (0 to 360)
        angleDegrees = (angleDegrees + 360) % 360;

        return angleDegrees;
    }

    public static Vector3 RotateAroundAxis(Vector3 vector, Vector3 axis, float angle)
    {
        // convert axis to unit vector
        axis = axis.normalized;

        // calculate sine and cosine of angle
        float sin = Mathf.Sin(angle * Mathf.Deg2Rad);
        float cos = Mathf.Cos(angle * Mathf.Deg2Rad);

        // calculate vector components parallel and perpendicular to axis
        Vector3 parallel = Vector3.Dot(vector, axis) * axis;
        Vector3 perpendicular = vector - parallel;

        // rotate perpendicular component around axis
        Vector3 newPerpendicular = cos * perpendicular + sin * Vector3.Cross(axis, perpendicular);

        // combine parallel and perpendicular components to get rotated vector
        return parallel + newPerpendicular;
    }
}


