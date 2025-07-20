
using Overhang;
using UnityEngine;

public class VectorUtils : MonoBehaviour
{
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
        Vector3 p1 = ray1.origin;
        Vector3 p2 = ray2.origin;
        Vector3 p1_to_p2 = p2 - p1;
        float t1 = Vector3.Dot(Vector3.Cross(p1_to_p2, v2), cross) / denominator;
        float t2 = Vector3.Dot(Vector3.Cross(p1_to_p2, v1), cross) / denominator;
        return (t1 >= 0f) && (t2 >= 0f);
    }
}
