using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Deform
{
	public class DeformableRayHit{
		public Vector3 point;
		public Vector3 normal;
		public int triangleIndex;
		public Vector3 barycentricCoordinate;
		public GameObject gameObject;
	}	


	public class DeformableRaycastable : MonoBehaviour
	{
		public SkinnedMeshRenderer smr;
		public Vector3[] verts;
		public Vector3[] norms;
        public int[] tris;
		public Mesh dupMesh;

		public bool didMod = false;
		public bool doDebug = true;

		public  float u;
		public  float v;
		public  float t; // distance from ray origin

		Vector3[] origverts;
		Vector3[] vertexData;
		Vector3[] tverts;
		Color[] cols;

		static Mesh mesh;
		Matrix4x4[] finalMatrix;
		float softRange = 0.05f;
		int i;
		BoneWeight[] weights;
		Color ColorRand;

		
		public bool RayTriangleIntersect(Ray r, Vector3 vert0, Vector3 vert1, Vector3 vert2)
		{
			// this originally comes from http://www.graphics.cornell.edu/pubs/1997/MT97.pdf
			t  = 0; 
			v = 0;
			u = 0;
			var edge1 = vert1 - vert0;
			var edge2 = vert2 - vert0;
			var pvec = Vector3.Cross(r.direction, edge2);
			float det = Vector3.Dot(edge1, pvec);
			if (det > -0.00001)
			{
				return false;    	
			}
			float inv_det = 1.0f / det;
			var tvec = r.origin - vert0;
			u = Vector3.Dot(tvec, pvec) * inv_det;
			if (u < -0.001 || u > 1.001)
				return false;
			var qvec = Vector3.Cross(tvec, edge1);
			v = Vector3.Dot(r.direction, qvec) * inv_det;
			if (v < -0.001 || u + v > 1.001)
				return false;
			t = Vector3.Dot(edge2, qvec) * inv_det;
			if (t <= 0)
				return false;  
			return true;
		}


		void Awake(){
			InitializeDeformable();
		}

		public void InitializeDeformable()
		{
			mesh = smr.sharedMesh;

			origverts = mesh.vertices;
			norms = mesh.normals;
			cols = mesh.colors;

			finalMatrix = new Matrix4x4[origverts.Length];
			vertexData =  mesh.vertices;
			
			// init a duplicate mesh
			dupMesh = new Mesh();
			dupMesh.vertices = mesh.vertices;
			dupMesh.triangles = mesh.triangles;	
			dupMesh.normals = mesh.normals;  
			// fps.inputstring = "\nVerts:" + origverts.Length.ToString() + "\nTris:" +   	mesher.triangles.Length.ToString();			
			weights = mesh.boneWeights; 
		}

		public void GetDeformedMesh(){
			// Debug.Log(smr);
			
			Matrix4x4[] matrices = new Matrix4x4[smr.bones.Length]; 
			for (i = 0; i < matrices.Length; i++) 
				matrices[i] = smr.bones[i].localToWorldMatrix * mesh.bindposes[i];
					
			
			for(i=0;i<origverts.Length;i++){ 
			BoneWeight weight = weights[i] ; 
			Matrix4x4 m0 = matrices[weight.boneIndex0]; 
			Matrix4x4 m1 = matrices[weight.boneIndex1]; 
			Matrix4x4 m2 = matrices[weight.boneIndex2]; 
			Matrix4x4 m3 = matrices[weight.boneIndex3]; 
			finalMatrix[i] = Matrix4x4.identity; 
			for(var n=0;n<16;n++){ 
				m0[n] *= weight.weight0; 
				m1[n] *= weight.weight1; 
				m2[n] *= weight.weight2; 
				m3[n] *= weight.weight3; 
				finalMatrix[i][n] = m0[n]+m1[n]+m2[n]+m3[n]; 
			} 
			vertexData[i] = finalMatrix[i].MultiplyPoint3x4(origverts[i]); 
			//vertexData[i].normal = finalMatrix[i].MultiplyVector(normals[i]); 
			}	
			dupMesh.vertices = vertexData;
			dupMesh.RecalculateNormals();
		}

		



		void LateUpdate(){
			if (didMod){
				mesh.colors = cols;
				didMod = false;
			}
		}
	}
}

