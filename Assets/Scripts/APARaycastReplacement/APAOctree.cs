/* *****************************************************************************
 * 
 *								EDUCATION RESEARCH GROUP
 *							MORGRIDGE INSTITUTE FOR RESEARCH
 * 			
 * 				
 * Copyright (c) 2012 EDUCATION RESEARCH, MORGRIDGE INSTITUTE FOR RESEARCH
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated  * documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
 * SOFTWARE.
 *  
 * 
 ******************************************************************************/
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public class APAOctree {
	
	public List<APAOctree> m_children; 
	public APAOctree parent;
	public Bounds bounds;
	public List<APATriangle> triangles;
	
//	~APAOctree(){
//		m_children.Clear();
//		m_children.TrimExcess();
//		parent = null;
//		triangles.Clear();
//		triangles.TrimExcess();
//	}
	public APAOctree(){
		this.m_children = new List<APAOctree>();
		this.triangles = new List<APATriangle>();
		this.parent = null;
	}
	public APAOctree(Bounds parentBounds, int generations){
		this.bounds = parentBounds;
		this.m_children = new List<APAOctree>();
		this.triangles = new List<APATriangle>();
		this.parent = null;
		CreateChildren(this, generations);
	}
	
	
	protected void CreateChildren(APAOctree parent, int generations){
		m_children = new List<APAOctree>();
		Vector3 c = parent.bounds.center;
		float u = parent.bounds.extents.x * 0.5f;
		float v = parent.bounds.extents.y * 0.5f;
		float w = parent.bounds.extents.z * 0.5f;
		Vector3 childrenSize = parent.bounds.extents;
		Vector3[] childrenCenters = {
			new Vector3(c.x + u, c.y + v, c.z + w),
			new Vector3(c.x + u, c.y + v, c.z - w),
			new Vector3(c.x + u, c.y - v, c.z + w),
			new Vector3(c.x + u, c.y - v, c.z - w),
			new Vector3(c.x - u, c.y + v, c.z + w),
			new Vector3(c.x - u, c.y + v, c.z - w),
			new Vector3(c.x - u, c.y - v, c.z + w),
			new Vector3(c.x - u, c.y - v, c.z - w)
		};
		
		for (int i = 0; i < childrenCenters.Length; i++){
			APAOctree o = new APAOctree();
			o.parent = parent;
			o.bounds = new Bounds(childrenCenters[i], childrenSize);
			m_children.Add(o);
			if (generations > 0){
				o.CreateChildren(o, generations - 1);
			}
		}
	}
	
	
	public APAOctree IndexTriangle(APATriangle triangle){
		return IndexTriangle(this, triangle);	
	}
	
	public APAOctree IndexTriangle(APAOctree parentNode, APATriangle triangle){
		APAOctree finalNode = parentNode;
		if (this.ContainsTriangle(triangle)){
			finalNode = this;
			for (int i = 0; i < m_children.Count; i++){ 
				finalNode = m_children[i].IndexTriangle(this, triangle);
				if (finalNode != this) return finalNode;
			}
			return finalNode;
		}
		return finalNode;
	}
	
	public bool AddTriangle(APATriangle t){
		triangles.Add(t);
		return true;
	}
	
	public bool ContainsTriangle(APATriangle triangle){
		return 	bounds.Contains(triangle.pt0) &&
				bounds.Contains(triangle.pt1) && 
				bounds.Contains(triangle.pt2);
	}
	
	public void Clear(){
		int total = ClearOctree(this);
		Debug.Log("Total Nodes Cleared: " + total);
	}
	
	protected int ClearOctree(APAOctree o){
		int count = 0;
		for (int i = 0; i < o.m_children.Count; i++){
			count += ClearOctree(o.m_children[i]);
		}
		o.triangles.Clear();
		o.triangles.TrimExcess();
		o.parent = null;
		o.m_children.Clear();
		o.m_children.TrimExcess();
		count ++;
		return count;
	}

}
