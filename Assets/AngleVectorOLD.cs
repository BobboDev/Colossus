using System.Collections;
using System.Collections.Generic;
// using Unity.Mathematics;
using UnityEngine;

namespace Overhang
{
    public class AngleVectorOLD : MonoBehaviour
    {
        Mesh mesh;

        Vector3[] verts;
        Vector3[] targetVerts;
        Vector3[] normals;
        int[] tris;

        MeshFilter meshFilter;

        public float angle;
        Vector3 direction;

        public Vector3 barycentricCoordinates;
        public ClimbableMesh cm;
        float timer;

        public Vector3 barycentricTarget;
        public Transform barycentricTargetTransform;
        public Transform character;
        float randomrot = 0;
        // Start is called before the first frame update
        void Start()
        {
            tris = new int[]{ 0,1,2 };
            verts = new Vector3[]{ new Vector3(0,0,0), new Vector3(0,0,0), new Vector3(0,0,0) };
            targetVerts = new Vector3[]{ new Vector3(0,0,0), new Vector3(0,0,0), new Vector3(0,0,0) };
            normals = new Vector3[]{ Vector3.up, Vector3.up, Vector3.up };
            
            mesh = new Mesh();


            meshFilter = GetComponent<MeshFilter>();
        }

        // Update is called once per frame
        void Update()
        {
            timer += Time.deltaTime;
            UpdateMesh();
        }

        void UpdateMesh()
        {
            if (timer > 1)
            {
                timer -= 1;
                randomrot = Random.Range((float)0,(float)360);
                for (int i = 0; i < 3; i++)
                {
                    Vector3 randomVertPos = Quaternion.Euler(0,Random.Range((float)120 * i,(float)120 * (i + 1)),0) * Vector3.forward;
                    
                    randomVertPos = Quaternion.Euler(0,randomrot,0) * randomVertPos;
                    targetVerts[i] = new Vector3( randomVertPos.x, 0, randomVertPos.z );
                }
                
            }

            for (int i = 0; i < 3; i++)
            {
                verts[i] = Vector3.Lerp(verts[i], targetVerts[i], 5 * Time.deltaTime);
            }
            for (int i = 0; i < 3; i++)
            {
                normals[i] = Vector3.Cross((verts[ i ]-verts[ (i + 1) % 3 ]).normalized, (verts[ (i + 1) % 3 ]-verts[ (i + 2) % 3 ]).normalized).normalized;
            }

            mesh.vertices = verts;
            mesh.triangles = tris;
            mesh.normals = normals;
            meshFilter.sharedMesh = mesh;


            barycentricTargetTransform.position = barycentricTarget.x * verts[0] +
                                                  barycentricTarget.y * verts[1] +
                                                  barycentricTarget.z * verts[2] ;

            Vector3 adjustedPosition =  barycentricCoordinates.x * verts[0] +
                                        barycentricCoordinates.y * verts[1] +
                                        barycentricCoordinates.z * verts[2] ;


            Vector3 adjustedNormal = barycentricCoordinates.x * normals[0] +
                                     barycentricCoordinates.y * normals[1] +
                                     barycentricCoordinates.z * normals[2] ;


            Quaternion tempQuaternion = Quaternion.AngleAxis(angle,adjustedNormal);

            Vector3 adjustedBarycentric = Quaternion.Euler( angle * barycentricCoordinates.x, 
                                                            angle * barycentricCoordinates.y, 
                                                            angle * barycentricCoordinates.z ) * Vector3.forward;


            Vector3[] cornerNormals = new Vector3[]{ -((verts[2] - verts[0]) + (verts[1] - verts[0])),
                                                     -((verts[0] - verts[1]) + (verts[2] - verts[1])),
                                                     -((verts[1] - verts[2]) + (verts[0] - verts[2]))};


            // Reverse engineering
            // adjustedNormal = character.up
            // adjustedDirection = character.forward
            // adjustedPosition = character.position
            //
            // Now we need to get the angle from adjustedBarycentric
            // 

            Vector3 adjustedDirection = adjustedBarycentric.x * cornerNormals[0] +
                                        adjustedBarycentric.y * cornerNormals[1] +
                                        adjustedBarycentric.z * cornerNormals[2] ;

            
            Vector3 barycentricCoordinate;
            barycentricCoordinate.x = adjustedDirection.x/ Vector3.Cross(cornerNormals[1],cornerNormals[2]).x ;
            barycentricCoordinate.y = Vector3.Dot(adjustedDirection,Vector3.Cross(cornerNormals[2],cornerNormals[0]));
            barycentricCoordinate.z = Vector3.Dot(adjustedDirection,Vector3.Cross(cornerNormals[0],cornerNormals[1]));

// recalculatedadjustedBarycentric.x = adjustedDirection.dot(cornerNormals[1].cross(cornerNormals[2])) / cornerNormals[0].dot(cornerNormals[1].cross(cornerNormals[2]));
// recalculatedadjustedBarycentric.y = adjustedDirection.dot(cornerNormals[2].cross(cornerNormals[0])) / cornerNormals[1].dot(cornerNormals[2].cross(cornerNormals[0]));
// recalculatedadjustedBarycentric.z = adjustedDirection.dot(cornerNormals[0].cross(cornerNormals[1])) / cornerNormals[2].dot(cornerNormals[0].cross(cornerNormals[1]));

            Debug.Log("bc " + barycentricCoordinate);
            Debug.Log("ad " + adjustedDirection);
            

            character.position = adjustedPosition;
            character.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(adjustedDirection, adjustedNormal), adjustedNormal.normalized);
            
            // meshFilter.transform.rotation = Quaternion.Lerp(meshFilter.transform.rotation,Quaternion.Euler(0,randomrot,0),0.1f);

            for (int i = 0; i < 3; i++)
            {
                Debug.DrawLine(verts[i],verts[i] + cornerNormals[i], Color.green);
            }
            Debug.DrawLine(adjustedPosition, adjustedPosition + adjustedDirection, Color.red);

            Debug.Log(" ");
            Debug.Log("Adjusted Direction: " + adjustedDirection.normalized);
            Debug.Log("Forward: " + character.forward);
            Debug.Log(" ");
            Debug.Log("Adjusted Normal: " + adjustedNormal);
            Debug.Log("Up: " + character.up);
            

            // float tempAngle;

            // tempAngle = Quaternion.LookRotation( cornerNormals[0].normalized, adjustedNormal).eulerAngles.y;
            // if (tempAngle >= 0 && tempAngle < 120)
            //     Debug.Log(0);
            // tempAngle = Quaternion.LookRotation( cornerNormals[1].normalized, adjustedNormal).eulerAngles.y;
            // if (tempAngle >= 0 && tempAngle < 120)
            //     Debug.Log(1);
            // tempAngle = Quaternion.LookRotation( cornerNormals[2].normalized, adjustedNormal).eulerAngles.y;
            // if (tempAngle >= 0 && tempAngle < 120)
            //     Debug.Log(2);

        }

    }
}