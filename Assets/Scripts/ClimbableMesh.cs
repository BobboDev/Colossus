using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Deform;
using UnityEditor;
using System.Linq;
using Unity.Collections;

namespace Overhang
{
    [RequireComponent(typeof(MeshCollider))]
    [RequireComponent(typeof(FaceSelection))]
    public class ClimbableMesh : MonoBehaviour
    {
        public bool test;
        [HideInInspector]
        public SkinnedMeshRenderer smr;
        [HideInInspector]
        public MeshFilter mf;
        [HideInInspector]
        public Transform climbableMeshTransform;

        [HideInInspector]
        public Mesh mesh;

        [HideInInspector]
        public List<Vector3> Vertices = new List<Vector3>();
        [HideInInspector]
        public List<Vector3> Normals = new List<Vector3>();

        [HideInInspector]
        public int[] Triangles;

        [HideInInspector]
        public Dictionary<int, AdjacentEdges> TriangleAdjacencyInfo, OriginalTriangleAdjacencyInfo = new();

        [HideInInspector]
        public Dictionary<Edge, AdjacentTriangles> EdgeAdjacencyInfo, OriginalEdgeAdjacencyInfo = new();

        [HideInInspector]
        public Dictionary<int, HashSet<Edge>> EdgesAttachedToCorner, OriginalEdgesAttachedToCorner = new();

        [HideInInspector]
        public Dictionary<int, HashSet<int>> SharedNormals = new();

        [HideInInspector]
        public HashSet<Edge> OutsideEdges, OriginalOutsideEdges = new();

        [HideInInspector]
        public Dictionary<PositionEdge, List<Edge>> SharedEdges, OriginalSharedEdges = new();

        [HideInInspector]
        public MeshCollider MeshCollider;

        FaceSelection FaceSelection;

        // Dictionary to map original triangle indices to new mesh triangle indices
        private Dictionary<int, int> originalToNewTriangleMap;

        Dictionary<string, Dictionary<int, int>> areaOriginalToNewTriangleMap;
        Dictionary<string, Dictionary<int, int>> areaNewToOriginalTriangleMap;

        Dictionary<string, Dictionary<int, int>> areaOriginalToNewTriangleMapGizmos;
        Dictionary<string, Dictionary<int, int>> areaNewToOriginalTriangleMapGizmos;

        public string CurrentArea;

        // Mesh colliderMesh;

        // Start is called before the first frame update
        void Awake()
        {
            FaceSelection = GetComponent<FaceSelection>();
            InitializeMesh();
        }

        // Getting stuff working based on slope, no edge   
        // AT THE SAME TIME WE RECORD THE EDGE INFOS, RECORD POSITIONS IN A <position, index> dictionary
        // once all recorded, specify new TriangleBs for those 

        // NEW PLAN

        // INSTEAD OF USING THE RAW MESH DATA, CREATE A "BETTER" MESH WHICH TRIES TO REPLICATE A SMOOTHED MESH WITH NO HARD EDGES, THEN USE THAT!

        void AddTriangle(int triangleIndex)
        {
            AdjacentEdges tempAdjacentEdges = new();
            tempAdjacentEdges.edges = new Edge[3];
            for (int i = 0; i < 3; i++)
            {
                // edges are always defined with the lowest 'triangle' index first.
                // so, 6 -> 15, never 15 -> 6
                // this is so that when we refer to them, we know how to define the edge
                if (Triangles[triangleIndex + i] < Triangles[(triangleIndex + ((i + 1) % 3))])
                {
                    tempAdjacentEdges.edges[i] = new Edge()
                    {
                        pointA = Triangles[triangleIndex + i],
                        pointB = Triangles[(triangleIndex + ((i + 1) % 3))],
                    };
                }
                else
                {
                    tempAdjacentEdges.edges[i] = new Edge()
                    {
                        pointB = Triangles[triangleIndex + i],
                        pointA = Triangles[(triangleIndex + ((i + 1) % 3))],
                    };
                }
                AddVertex(Triangles[triangleIndex + i], tempAdjacentEdges.edges[i]);
                AddVertex(Triangles[(triangleIndex + ((i + 1) % 3))], tempAdjacentEdges.edges[i]);
                // add side to list with current triangle index
                // if already associated to a triangle, make it triangleB
                // if triangleA == triangleB, it's an edge with no adjacent tri
                AddEdge(tempAdjacentEdges.edges[i], triangleIndex);
            }
            // add triangle with 3 sides to list
            TriangleAdjacencyInfo.Add(triangleIndex, tempAdjacentEdges);

        }

        void AddEdge(Edge edge, int triangleIndex)
        {
            AdjacentTriangles tempEntry;

            if (!EdgeAdjacencyInfo.TryGetValue(edge, out tempEntry))
            {
                AdjacentTriangles tempAdjacentTriangles = new AdjacentTriangles() { triangleA = triangleIndex, triangleB = triangleIndex, edgeA = edge };
                if (!OutsideEdges.Contains(edge))
                {
                    OutsideEdges.Add(edge);
                }
                EdgeAdjacencyInfo.Add(edge, tempAdjacentTriangles);
            }
            else
            {
                AdjacentTriangles tempAdjacentTriangles = new AdjacentTriangles() { triangleA = EdgeAdjacencyInfo[edge].triangleA, triangleB = triangleIndex };
                if (OutsideEdges.Contains(edge))
                {
                    OutsideEdges.Remove(edge);
                }
                EdgeAdjacencyInfo[edge] = tempAdjacentTriangles;
            }
        }

        void AddVertex(int key, Edge value)
        {
            if (!EdgesAttachedToCorner.ContainsKey(key))
            {
                HashSet<Edge> tempVertices = new HashSet<Edge>();
                tempVertices.Add(value);
                EdgesAttachedToCorner.Add(key, tempVertices);
            }
            else
            {
                EdgesAttachedToCorner[key].Add(value);
            }
        }
        void GenerateAdjacencyInfo(bool isOnArea, bool isStart)
        {
            if (isOnArea || isStart)
            {
                // if (outsideEdges != null && outsideEdges.Count > 0)
                OutsideEdges = new();
                // if (triangleInfos != null && triangleInfos.Count > 0)
                TriangleAdjacencyInfo = new();
                // if (edgeInfos != null && edgeInfos.Count > 0)
                EdgeAdjacencyInfo = new();
                // if (vertexInfos != null && vertexInfos.Count > 0)
                EdgesAttachedToCorner = new();
                // if (sharedEdges != null && sharedEdges.Count > 0)
                SharedEdges = new();

                for (int i = 0; i < Triangles.Length; i += 3)
                {
                    AddTriangle(i);
                }
            }
            else
            {
                OutsideEdges = OriginalOutsideEdges;
                TriangleAdjacencyInfo = OriginalTriangleAdjacencyInfo;
                EdgeAdjacencyInfo = OriginalEdgeAdjacencyInfo;
                EdgesAttachedToCorner = OriginalEdgesAttachedToCorner;
                SharedEdges = OriginalSharedEdges;
            }
        }

        void InitializeMesh()
        {
            MeshCollider = GetComponent<MeshCollider>();
            smr = GetComponent<SkinnedMeshRenderer>();
            mf = GetComponent<MeshFilter>();
            mesh = new Mesh();

            if (smr != null)
            {
                Debug.Log("nut");
                smr.BakeMesh(mesh);
                climbableMeshTransform = smr.transform;
            }
            else if (mf != null)
            {
                mesh = mf.sharedMesh;
                climbableMeshTransform = mf.transform;
            }

            // Dictionary<string, List<int>> areaFaceIndices = new();

            // // Collect face indices for each area
            // foreach (FaceSelection.AreaEntry a in FaceSelection.Areas)
            // {
            //     areaFaceIndices[a.Key] = new List<int>(a.Value.TrianglesInSelection);
            // }

            areaOriginalToNewTriangleMap = new Dictionary<string, Dictionary<int, int>>();
            areaNewToOriginalTriangleMap = new Dictionary<string, Dictionary<int, int>>();
            foreach (FaceSelection.AreaEntry a in FaceSelection.Areas)
            {
                ExtractFaces(mesh, a, ref areaOriginalToNewTriangleMap, ref areaNewToOriginalTriangleMap);
                // Debug.Log(areaOriginalToNewTriangleMap.Count);
            }

            areaOriginalToNewTriangleMapGizmos = new Dictionary<string, Dictionary<int, int>>();
            areaNewToOriginalTriangleMapGizmos = new Dictionary<string, Dictionary<int, int>>();
            foreach (FaceSelection.AreaEntry a in FaceSelection.Areas)
            {
                ExtractFaces(mesh, a, ref areaOriginalToNewTriangleMapGizmos, ref areaNewToOriginalTriangleMapGizmos);
                // Debug.Log(areaOriginalToNewTriangleMap.Count);
            }
            // DrawEdges(tempMesh);
            // colliderMesh = mesh;
            MeshCollider.sharedMesh = mesh;
            Triangles = mesh.triangles;



            // Now you have areaOriginalToNewTriangleMap and areaNewToOriginalTriangleMap to use as needed


            RecalculateMesh(true);

            CalculateEdges(false, true);

            OriginalOutsideEdges = OutsideEdges;
            OriginalTriangleAdjacencyInfo = TriangleAdjacencyInfo;
            OriginalEdgeAdjacencyInfo = EdgeAdjacencyInfo;
            OriginalEdgesAttachedToCorner = EdgesAttachedToCorner;
            OriginalSharedEdges = SharedEdges;

            gameObject.layer = LayerMask.NameToLayer("Climbable");

        }

        void CalculateEdges(bool isOnArea, bool isStart)
        {

            GenerateAdjacencyInfo(isOnArea, isStart);

            if (!isStart && !isOnArea)
                return;

            HashSet<PositionEdge> tempPositionEdges = new();

            //Checking just outside edges as those are the ones that are potentially 'hard'
            foreach (Edge e in OutsideEdges)
            {
                PositionEdge tempPositionEdge;
                // making sure pointA is always the vertex closer to world origin (0,0). Is that a good idea?
                if (Vertices[e.pointA].magnitude < Vertices[e.pointB].magnitude)
                {
                    tempPositionEdge = new PositionEdge() { pointA = Vertices[e.pointA], pointB = Vertices[e.pointB] };
                }
                else
                {
                    tempPositionEdge = new PositionEdge() { pointB = Vertices[e.pointA], pointA = Vertices[e.pointB] };
                }
                // if a matching edge is found, add this to its position edges with a key of positionEdge
                if (tempPositionEdges.Contains(tempPositionEdge))
                {
                    SharedEdges[tempPositionEdge].Add(e);
                }
                else
                {
                    List<Edge> tempEdgeList = new List<Edge>();
                    tempEdgeList.Add(e);
                    tempPositionEdges.Add(tempPositionEdge);
                    SharedEdges.Add(tempPositionEdge, tempEdgeList);
                }
            }

            var itemsToRemove = new List<PositionEdge>();
            Dictionary<Edge, AdjacentTriangles> tempEdgeInfos = EdgeAdjacencyInfo;

            for (int i = 0; i < 15; i++)
            {
                foreach (var e in SharedEdges)
                {
                    if (e.Value.Count != 2)
                    {
                        itemsToRemove.Add(e.Key);
                    }
                    else
                    {
                        //
                        AdjacentTriangles tempEdge1 = new() { triangleA = EdgeAdjacencyInfo[e.Value[0]].triangleA, triangleB = EdgeAdjacencyInfo[e.Value[1]].triangleA };
                        tempEdgeInfos[e.Value[0]] = tempEdge1;
                        AdjacentTriangles tempEdge2 = new() { triangleA = EdgeAdjacencyInfo[e.Value[1]].triangleA, triangleB = EdgeAdjacencyInfo[e.Value[0]].triangleA };
                        tempEdgeInfos[e.Value[1]] = tempEdge2;
                        // PROBLEM!!!!
                        // I think this is where shared edges should be sharing connected vertices, but they're not in some cases...
                        // e.Value[#].pointA/B are two edges that are identical in placement but different
                        // if the pointAs have the same position

                        if (mesh.vertices[e.Value[0].pointA] == mesh.vertices[e.Value[1].pointA])
                        {
                            // for every edge coming off of line B's pointA 
                            foreach (Edge edge in EdgesAttachedToCorner[e.Value[1].pointA])
                            {
                                // if it isn't already attached to pointA
                                if (!EdgesAttachedToCorner[e.Value[0].pointA].Contains(edge))
                                {
                                    // and if the second line isn't equal to this other line
                                    if (!(e.Value[1].pointA == edge.pointA && e.Value[1].pointB == edge.pointB) && !(e.Value[1].pointA == edge.pointB && e.Value[1].pointB == edge.pointA))
                                        EdgesAttachedToCorner[e.Value[0].pointA].Add(edge); // then add the edge to pointA
                                }
                            }
                            foreach (Edge edge in EdgesAttachedToCorner[e.Value[0].pointA])
                            {
                                if (!EdgesAttachedToCorner[e.Value[1].pointA].Contains(edge))
                                {
                                    if (!(e.Value[0].pointA == edge.pointA && e.Value[0].pointB == edge.pointB) && !(e.Value[0].pointA == edge.pointB && e.Value[0].pointB == edge.pointA))
                                        EdgesAttachedToCorner[e.Value[1].pointA].Add(edge);
                                }
                            }
                        }

                        if (mesh.vertices[e.Value[0].pointA] == mesh.vertices[e.Value[1].pointB])
                        {
                            foreach (Edge edge in EdgesAttachedToCorner[e.Value[1].pointB])
                            {
                                if (!EdgesAttachedToCorner[e.Value[0].pointA].Contains(edge))
                                {
                                    if (!(e.Value[1].pointA == edge.pointA && e.Value[1].pointB == edge.pointB) && !(e.Value[1].pointA == edge.pointB && e.Value[1].pointB == edge.pointA))
                                        EdgesAttachedToCorner[e.Value[0].pointA].Add(edge);
                                }
                            }
                            foreach (Edge edge in EdgesAttachedToCorner[e.Value[0].pointA])
                            {
                                if (!EdgesAttachedToCorner[e.Value[1].pointB].Contains(edge))
                                {
                                    if (!(e.Value[0].pointA == edge.pointA && e.Value[0].pointB == edge.pointB) && !(e.Value[0].pointA == edge.pointB && e.Value[0].pointB == edge.pointA))
                                        EdgesAttachedToCorner[e.Value[1].pointB].Add(edge);
                                }
                            }
                        }
                        if (mesh.vertices[e.Value[0].pointB] == mesh.vertices[e.Value[1].pointB])
                        {
                            foreach (Edge edge in EdgesAttachedToCorner[e.Value[1].pointB])
                            {
                                if (!EdgesAttachedToCorner[e.Value[0].pointB].Contains(edge))
                                {
                                    if (!(e.Value[1].pointA == edge.pointA && e.Value[1].pointB == edge.pointB) && !(e.Value[1].pointA == edge.pointB && e.Value[1].pointB == edge.pointA))
                                        EdgesAttachedToCorner[e.Value[0].pointB].Add(edge);
                                }
                            }
                            foreach (Edge edge in EdgesAttachedToCorner[e.Value[0].pointB])
                            {
                                if (!EdgesAttachedToCorner[e.Value[1].pointB].Contains(edge))
                                {
                                    if (!(e.Value[0].pointA == edge.pointA && e.Value[0].pointB == edge.pointB) && !(e.Value[0].pointA == edge.pointB && e.Value[0].pointB == edge.pointA))
                                        EdgesAttachedToCorner[e.Value[1].pointB].Add(edge);
                                }
                            }
                        }
                        if (mesh.vertices[e.Value[0].pointB] == mesh.vertices[e.Value[1].pointA])
                        {
                            foreach (Edge edge in EdgesAttachedToCorner[e.Value[1].pointA])
                            {
                                if (!EdgesAttachedToCorner[e.Value[0].pointB].Contains(edge))
                                {
                                    if (!(e.Value[1].pointA == edge.pointA && e.Value[1].pointB == edge.pointB) && !(e.Value[1].pointA == edge.pointB && e.Value[1].pointB == edge.pointA))
                                        EdgesAttachedToCorner[e.Value[0].pointB].Add(edge);
                                }
                            }
                            foreach (Edge edge in EdgesAttachedToCorner[e.Value[0].pointB])
                            {
                                if (!EdgesAttachedToCorner[e.Value[1].pointA].Contains(edge))
                                {
                                    if (!(e.Value[0].pointA == edge.pointA && e.Value[0].pointB == edge.pointB) && !(e.Value[0].pointA == edge.pointB && e.Value[0].pointB == edge.pointA))
                                        EdgesAttachedToCorner[e.Value[1].pointA].Add(edge);
                                }
                            }
                        }
                        Edge tempEdge = new Edge() { triangleA = e.Value[0].triangleA };
                    }
                }
            }
            EdgeAdjacencyInfo = tempEdgeInfos;

            foreach (var e in itemsToRemove)
            {
                SharedEdges.Remove(e);
            }
        }
        public int GetArea(int index)
        {
            // Example usage of the lookup method
            if (TryGetMatchingAreaAndIndex(index, out string matchingArea, out int newTriangleIndex, areaOriginalToNewTriangleMap))
            {
                CurrentArea = matchingArea;
                RecalculateMesh(false);
                CalculateEdges(true, false);
                Debug.Log($"Matching area: {matchingArea}, New triangle index: {newTriangleIndex}");


                return newTriangleIndex;
            }
            else
            {
                CurrentArea = "";
                Debug.Log("No matching triangle found.");
                return -1;
            }
        }

        public int GetMainBody(int index)
        {
            // Example usage of the lookup method
            if (TryGetMatchingIndex(index, out int newTriangleIndex, areaNewToOriginalTriangleMap))
            {
                RecalculateMesh(false);
                CurrentArea = "";
                CalculateEdges(false, false);
                Debug.Log($"New triangle index: {newTriangleIndex}");
                return newTriangleIndex;
            }
            else
            {
                Debug.Log("No matching triangle found.");
                return -1;
            }
        }

        void Update()
        {
            // if (!Climbing)
            RecalculateMesh(false);
        }
        void LateUpdate()
        {
            if (test)
            {
                foreach (var e in EdgeAdjacencyInfo.Values)
                {
                    // foreach (var e2 in e.edges)
                    // {
                    //     // Debug.DrawLine(meshVerts[e2.pointA],meshVerts[e2.pointB],Color.yellow,0);

                    if (e.triangleA == e.triangleB)
                    {
                        // Highlight Edges
                        // Debug.DrawLine(meshVerts[e.edgeA.pointA], meshVerts[e.edgeA.pointB], Color.yellow, 0);
                    }
                    // }

                }
            }
        }

        // Method to find matching area and triangle index
        public bool TryGetMatchingIndex(int originalTriangleIndex,
            out int newTriangleIndex,
            Dictionary<string, Dictionary<int, int>> areaNewToOriginalTriangleMap)
        {
            // Initialize output parameters
            newTriangleIndex = -1;

            foreach (var areaMap in areaNewToOriginalTriangleMap)
            {
                if (areaMap.Key == CurrentArea)
                {
                    Debug.Log(areaMap.Key);
                    if (areaMap.Value.TryGetValue(originalTriangleIndex, out newTriangleIndex))
                    {
                        return true; // Match found
                    }
                }
            }

            return false; // No match found
        }

        // Method to find matching area and triangle index
        public bool TryGetMatchingAreaAndIndex(int originalTriangleIndex,
            out string matchingArea,
            out int newTriangleIndex,
            Dictionary<string, Dictionary<int, int>> areaOriginalToNewTriangleMap)
        {
            // Initialize output parameters
            matchingArea = null;
            newTriangleIndex = -1;

            foreach (var areaMap in areaOriginalToNewTriangleMap)
            {
                if (areaMap.Value.ContainsKey(originalTriangleIndex))
                {
                    if (areaMap.Value.TryGetValue(originalTriangleIndex, out newTriangleIndex))
                    {
                        matchingArea = areaMap.Key;
                        return true; // Match found
                    }
                }

            }
            return false; // No match found
        }
        public void RecalculateMesh(bool isStart)
        {
            Vertices.Clear();
            Normals.Clear();
            Vector3[] tempVertices = new Vector3[0];
            Vector3[] tempNormals = new Vector3[0];

            if (smr != null)
            {
                smr.BakeMesh(mesh, true);
                tempVertices = mesh.vertices;
                tempNormals = mesh.normals;
                climbableMeshTransform = smr.transform;
            }
            else if (mf != null)
            {
                mesh = mf.sharedMesh;
                tempVertices = mesh.vertices;
                tempNormals = mesh.normals;
                climbableMeshTransform = mf.transform;
            }

            Dictionary<string, List<int>> areaFaceIndices = new();

            // // Collect face indices for each area
            // foreach (FaceSelection.AreaEntry a in FaceSelection.Areas)
            // {
            //     areaFaceIndices[a.Key] = new List<int>(a.Value.TrianglesInSelection);
            // }
            if (isStart)
            {
                areaOriginalToNewTriangleMap = new Dictionary<string, Dictionary<int, int>>();
                areaNewToOriginalTriangleMap = new Dictionary<string, Dictionary<int, int>>();

            }
            foreach (FaceSelection.AreaEntry a in FaceSelection.Areas)
            {

                if (isStart)
                {
                    areaNewToOriginalTriangleMap.Add(a.Value.AreaName, new Dictionary<int, int>());
                    // Just setting the maps
                    ExtractFaces(mesh, a, ref areaOriginalToNewTriangleMap, ref areaNewToOriginalTriangleMap);

                }
                else
                {
                    if (CurrentArea == a.Key)
                    {
                        Mesh meshToDelete = mesh;
                        mesh = ExtractFaces(mesh, a);
                        Destroy(meshToDelete);
                        break;
                    }
                }
            }
            if (isStart)
            {
                foreach (var v in areaNewToOriginalTriangleMap)
                {
                    Debug.Log("Map Value name: " + v.Key + " Count " + v.Value.Count);
                    // Debug.Log();
                }
            }
            Triangles = mesh.triangles;

            MeshCollider.sharedMesh = mesh;

            if (smr != null)
            {
                tempVertices = mesh.vertices;
                tempNormals = mesh.normals;
            }
            else if (mf != null)
            {
                tempVertices = mesh.vertices;
                tempNormals = mesh.normals;
            }

            Matrix4x4 localToWorldMatrix = climbableMeshTransform.localToWorldMatrix;

            foreach (Vector3 v in tempVertices)
            {
                Vertices.Add(localToWorldMatrix.MultiplyPoint3x4(v));
            }

            foreach (Vector3 n in tempNormals)
            {
                Normals.Add(localToWorldMatrix.MultiplyVector(n));
            }

            // You have areaOriginalToNewTriangleMap and areaNewToOriginalTriangleMap here as well
        }



        Mesh ExtractFaces(
    Mesh sourceMesh,
    FaceSelection.AreaEntry area,
    ref Dictionary<string, Dictionary<int, int>> areaOriginalToNewTriangleMap,
    ref Dictionary<string, Dictionary<int, int>> areaNewToOriginalTriangleMap)
        {
            // Retrieve source mesh data
            Vector3[] vertices = sourceMesh.vertices;
            Vector3[] normals = sourceMesh.normals;
            Vector2[] uvs = sourceMesh.uv;
            int[] triangles = sourceMesh.triangles;

            // Pre-allocate lists with estimated size
            int estimatedTriangleCount = area.Value.TrianglesInSelection.Count * 3;
            List<Vector3> newVertices = new List<Vector3>(estimatedTriangleCount);
            List<Vector3> newNormals = normals.Length > 0 ? new List<Vector3>(estimatedTriangleCount) : null;
            List<Vector2> newUVs = uvs.Length > 0 ? new List<Vector2>(estimatedTriangleCount) : null;
            List<int> newTriangles = new List<int>(estimatedTriangleCount);

            Dictionary<int, int> vertexMapping = new Dictionary<int, int>(estimatedTriangleCount);

            int newTriangleIndex = 0;
            string areaId = area.Value.AreaName;
            List<int> faceIndices = area.Value.TrianglesInSelection;

            // Initialize mappings for this area if not already done
            if (!areaOriginalToNewTriangleMap.ContainsKey(areaId))
            {
                areaOriginalToNewTriangleMap[areaId] = new Dictionary<int, int>(estimatedTriangleCount / 3);
            }

            if (!areaNewToOriginalTriangleMap.ContainsKey(areaId))
            {
                areaNewToOriginalTriangleMap[areaId] = new Dictionary<int, int>(estimatedTriangleCount / 3);
            }

            foreach (int faceIndex in faceIndices)
            {
                int triIndex = faceIndex * 3;

                // Process the three vertices of this face
                for (int j = 0; j < 3; j++)
                {
                    int vertexIndex = triangles[triIndex + j];

                    if (!vertexMapping.ContainsKey(vertexIndex))
                    {
                        vertexMapping[vertexIndex] = newVertices.Count;
                        newVertices.Add(vertices[vertexIndex]);

                        if (newNormals != null)
                            newNormals.Add(normals[vertexIndex]);

                        if (newUVs != null)
                            newUVs.Add(uvs[vertexIndex]);
                    }

                    newTriangles.Add(vertexMapping[vertexIndex]);
                }

                // Map the original triangle index to the new triangle index
                areaOriginalToNewTriangleMap[areaId][faceIndex] = newTriangleIndex / 3;
                areaNewToOriginalTriangleMap[areaId][newTriangleIndex / 3] = faceIndex;

                newTriangleIndex += 3;
            }

            // Create the new mesh
            Mesh newMesh = new Mesh
            {
                vertices = newVertices.ToArray(),
                triangles = newTriangles.ToArray()
            };

            if (newNormals != null)
                newMesh.normals = newNormals.ToArray();

            if (newUVs != null)
                newMesh.uv = newUVs.ToArray();

            newMesh.RecalculateBounds();

            // Only recalculate normals if they were not provided
            if (newNormals == null)
                newMesh.RecalculateNormals();



            return newMesh;
        }

        Mesh ExtractFaces(
            Mesh sourceMesh,
            FaceSelection.AreaEntry area)
        {
            // Retrieve source mesh data
            Vector3[] vertices = sourceMesh.vertices;
            Vector3[] normals = sourceMesh.normals;
            int[] triangles = sourceMesh.triangles;

            // Pre-allocate lists with estimated size
            int estimatedTriangleCount = area.Value.TrianglesInSelection.Count * 3;
            List<Vector3> newVertices = new List<Vector3>(estimatedTriangleCount);
            List<Vector3> newNormals = normals.Length > 0 ? new List<Vector3>(estimatedTriangleCount) : null;
            List<int> newTriangles = new List<int>(estimatedTriangleCount);

            Dictionary<int, int> vertexMapping = new Dictionary<int, int>(estimatedTriangleCount);

            int newTriangleIndex = 0;
            string areaId = area.Key;
            List<int> faceIndices = area.Value.TrianglesInSelection;


            foreach (int faceIndex in faceIndices)
            {
                int triIndex = faceIndex * 3;

                // Process the three vertices of this face
                for (int j = 0; j < 3; j++)
                {
                    int vertexIndex = triangles[triIndex + j];

                    if (!vertexMapping.ContainsKey(vertexIndex))
                    {
                        vertexMapping[vertexIndex] = newVertices.Count;
                        newVertices.Add(vertices[vertexIndex]);

                        if (newNormals != null)
                            newNormals.Add(normals[vertexIndex]);
                    }

                    newTriangles.Add(vertexMapping[vertexIndex]);
                }

                newTriangleIndex += 3;
            }

            // Create the new mesh
            Mesh newMesh = new Mesh
            {
                vertices = newVertices.ToArray(),
                triangles = newTriangles.ToArray()
            };

            if (newNormals != null)
                newMesh.normals = newNormals.ToArray();

            // newMesh.RecalculateBounds();

            // // Only recalculate normals if they were not provided
            // if (newNormals == null)
            //     newMesh.RecalculateNormals();

            return newMesh;
        }






        // Method to get the new triangle index from the original triangle index
        public int GetNewTriangleIndex(int originalTriangleIndex)
        {
            if (originalToNewTriangleMap != null && originalToNewTriangleMap.ContainsKey(originalTriangleIndex))
            {
                return originalToNewTriangleMap[originalTriangleIndex];
            }
            else
            {
                Debug.LogError("Triangle index not found in the map.");
                return -1; // Return an invalid index if not found
            }
        }
        void OnDrawGizmos()
        {
#if UNITY_EDITOR
            if (!EditorApplication.isPlaying)
                return;
#endif
            if (!FaceSelection)
                return;

            foreach (FaceSelection.AreaEntry a in FaceSelection.Areas)
            {
                Mesh tempMesh = new();

                if (smr != null)
                {
                    Destroy(tempMesh);
                    smr.BakeMesh(tempMesh);
                }
                if (mf != null)
                {
                    Destroy(tempMesh);
                    tempMesh = mf.mesh;
                }

                Mesh meshToDelete = tempMesh;
                tempMesh = ExtractFaces(tempMesh, a, ref areaOriginalToNewTriangleMapGizmos, ref areaNewToOriginalTriangleMapGizmos);


                if (tempMesh == null)
                {
                    Debug.Log("null mesh");
                    return;
                }

                // Step 1: Identify boundary edges
                Dictionary<DebugEdge, int> edgeCount = new Dictionary<DebugEdge, int>();

                int[] triangles = tempMesh.triangles;
                Vector3[] vertices = tempMesh.vertices;

                for (int i = 0; i < triangles.Length; i += 3)
                {
                    DebugEdge edge1 = new DebugEdge(triangles[i], triangles[i + 1]);
                    DebugEdge edge2 = new DebugEdge(triangles[i + 1], triangles[i + 2]);
                    DebugEdge edge3 = new DebugEdge(triangles[i + 2], triangles[i]);

                    CountEdge(edge1, edgeCount);
                    CountEdge(edge2, edgeCount);
                    CountEdge(edge3, edgeCount);
                }
                int x = 0;
                // Step 2: Draw debug lines for boundary edges
                foreach (var edge in edgeCount)
                {
                    if (edge.Value == 1) // Boundary edge (appears only once)
                    {
                        Vector3 v1 = climbableMeshTransform.transform.TransformPoint(vertices[edge.Key.VertexIndex1]);
                        Vector3 v2 = climbableMeshTransform.transform.TransformPoint(vertices[edge.Key.VertexIndex2]);
                        x++;
                        // Draw the debug line for a single frame duration
                        // Debug.DrawLine(v1, v2, Color.yellow);
                    }
                }

                // Debug.Log(x);
                void CountEdge(DebugEdge edge, Dictionary<DebugEdge, int> edgeCount)
                {
                    if (edgeCount.ContainsKey(edge))
                    {
                        edgeCount[edge]++;
                    }
                    else
                    {
                        edgeCount[edge] = 1;
                    }
                }
                Destroy(tempMesh);
                Destroy(meshToDelete);


            }
        }
        // Edge struct to represent edges, with equality checks
        public struct DebugEdge
        {
            public int VertexIndex1;
            public int VertexIndex2;

            public DebugEdge(int vertexIndex1, int vertexIndex2)
            {
                // Ensure that the lower index is always first to make edges comparable
                if (vertexIndex1 < vertexIndex2)
                {
                    VertexIndex1 = vertexIndex1;
                    VertexIndex2 = vertexIndex2;
                }
                else
                {
                    VertexIndex1 = vertexIndex2;
                    VertexIndex2 = vertexIndex1;
                }
            }

            // Override Equals and GetHashCode to compare edges correctly
            public override bool Equals(object obj)
            {
                if (!(obj is DebugEdge)) return false;
                DebugEdge other = (DebugEdge)obj;
                return VertexIndex1 == other.VertexIndex1 && VertexIndex2 == other.VertexIndex2;
            }

            public override int GetHashCode()
            {
                return VertexIndex1.GetHashCode() ^ VertexIndex2.GetHashCode();
            }
        }
    }
}


public struct VertexInfo
{
    public Vector3 point;
    public Vector3 normal;
}

public struct PositionEdge
{
    public Vector3 pointA;
    public Vector3 pointB;
}

public struct Edge
{
    public int pointA;
    public int pointB;

    public int triangleA;
    public int triangleB;
}

public struct EdgeIndices
{
    public int originalIndex;
    public int newIndex;
}

public struct AdjacentEdges
{
    public Edge[] edges;
}

public struct AdjacentTriangles
{
    public int triangleA;
    public Edge edgeA;
    public int triangleB;
    public Edge edgeB;
}