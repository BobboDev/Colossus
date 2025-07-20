using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Collections;
using UnityEditor;

[RequireComponent(typeof(SkinnedMeshRenderer))]
public class SkinnedPositionsFromGPU : MonoBehaviour
{
    public struct VertexData
    {
        public Vector3 postion;
        public Vector3 normal;
        public Vector4 tangent;
    }

    public float size = 0.01f;
    public bool outputNumFrames = false;
    public bool useRootBone = true;
    public bool enableGPU;

    SkinnedMeshRenderer smr;

    GraphicsBuffer gpuVertexBuffer;
    NativeArray<VertexData>[] vertices = new NativeArray<VertexData>[2];
    Matrix4x4 lastLocalToWorld;

    bool doRequest;
    bool reading;
    bool pingPong;
    int frameCount;

    private void OnEnable()
    {
        smr = GetComponent<SkinnedMeshRenderer>();

        vertices[0] = new NativeArray<VertexData>(smr.sharedMesh.vertexCount, Allocator.Persistent);
        vertices[1] = new NativeArray<VertexData>(smr.sharedMesh.vertexCount, Allocator.Persistent);
        smr.vertexBufferTarget |= GraphicsBuffer.Target.Raw;

        
        if (QualitySettings.renderPipeline is not null || GraphicsSettings.currentRenderPipeline is not null)
            RenderPipelineManager.beginFrameRendering += RenderPipelineManager_beginFrameRendering;
        else
            Camera.onPreRender += onPreRender;
    }

    void GetGPUVertexBuffer()
    {
        if (Application.isPlaying && doRequest && enableGPU)
        {
            doRequest = false;
            gpuVertexBuffer ??= smr.GetVertexBuffer(); //Vertex buffer gets stale after frame is done?

            if (gpuVertexBuffer != null)
            {
                reading = true;
                frameCount = Time.frameCount;
                pingPong = !pingPong;
                AsyncGPUReadback.RequestIntoNativeArray(ref vertices[pingPong ? 1 : 0], gpuVertexBuffer, GPURequestCallback); //can't read from native container while readback is in process
                if (useRootBone)
                {
                    lastLocalToWorld = smr.rootBone.transform.localToWorldMatrix;
                }
                else
                {
                    lastLocalToWorld = smr.transform.localToWorldMatrix;
                }
            }
        }
    }
    
    private void onPreRender(Camera cam)
    {
        GetGPUVertexBuffer();
    }

    private void RenderPipelineManager_beginFrameRendering(ScriptableRenderContext arg1, Camera[] arg2)
    {
        GetGPUVertexBuffer();
    }

    // public void ToggleEnableGPU()
    // {
    //     enableGPU = !enableGPU;
    //     if (enableGPU) SceneView.lastActiveSceneView.drawGizmos = true;
    // }

    private void Update()
    {
        if(!reading && enableGPU)
        {
            doRequest = true;
        }
    }

    [ContextMenu("Start Request")]
    public void StartRequest()
    {
        doRequest = true;
    }

    private void GPURequestCallback(AsyncGPUReadbackRequest obj)
    {
        reading = false;
        if(outputNumFrames)
            Debug.Log($"Readback took {Time.frameCount - frameCount} frames!");
    }

    private void OnDisable()
    {
        AsyncGPUReadback.WaitAllRequests();

        gpuVertexBuffer?.Dispose();
        gpuVertexBuffer = null;

        if(vertices[0].IsCreated)
            vertices[0].Dispose();
        if(vertices[1].IsCreated)
            vertices[1].Dispose();
    }

    private void OnDrawGizmos()
    {
        if (smr != null && enableGPU)
        {
            Gizmos.matrix = lastLocalToWorld;
            Vector3 cubeSize = new Vector3(size, size, size);
            int index = pingPong ? 0 : 1;

            if (vertices != null && vertices[index].IsCreated)
            {
                for (int i = 0; i < vertices[index].Length; i++)
                {
                    Gizmos.DrawCube(vertices[index][i].postion, cubeSize);
                }
            }
        }
    }
}
