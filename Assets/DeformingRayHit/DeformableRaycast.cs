using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Deform;


public class DeformableRaycast
{
    public static DeformableRayHit Raycast(Vector3 origin, Vector3 direction)
    {
        DeformableRayHit hit = new DeformableRayHit();
        Ray ray = new Ray();
        ray.origin = origin;
        ray.direction = direction;
        float distance;
        float closest = Mathf.Infinity;
        DeformableRaycastable dr;
        foreach (var item in GameManager.instance.meshManager.skinnedMeshRenderers)
        {
            if (item.bounds.IntersectRay(ray, out distance))
            {
                if (distance < closest)
                {
                    closest = distance;
                    hit.gameObject = item.gameObject;
                }
            }
        }

        dr = hit.gameObject.GetComponent<DeformableRaycastable>();

        dr.GetDeformedMesh();
        dr.tris =  dr.dupMesh.triangles;
        dr.norms = dr.dupMesh.normals;
        dr.verts = dr.dupMesh.vertices;
        // if (dr.doDebug)
            // Debug.DrawLine( ray.origin, ray.origin + ray.direction*100, Color.red);

        for (int i = 0; i < dr.tris.Length; i+=3){
                
            if (Vector3.Dot( dr.norms[dr.tris[i + 0]], ray.direction) > 0)
                continue;

            if (dr.RayTriangleIntersect(ray, 
                                    dr.verts[dr.tris[i + 0]],
                                    dr.verts[dr.tris[i + 2]],   
                                    dr.verts[dr.tris[i + 1]] ))
            {

                hit.point =  (1 - dr.u - dr.v)  * dr.verts[dr.tris[i + 0]] + dr.v * dr.verts[dr.tris[i + 1]] + dr.u * dr.verts[dr.tris[i + 2]];
                hit.normal = (1 - dr.u - dr.v)  * dr.norms[dr.tris[i + 0]] + dr.v * dr.norms[dr.tris[i + 1]] + dr.u * dr.norms[dr.tris[i + 2]];
                hit.triangleIndex = i;
                hit.barycentricCoordinate = Mathf2.GetBarycentricCoordinates(hit.point, dr.verts[dr.tris[i + 0]],dr.verts[dr.tris[i + 1]],dr.verts[dr.tris[i + 2]]);

                dr.didMod = true;
            }
        }
        
        return hit;
        
    }
}


