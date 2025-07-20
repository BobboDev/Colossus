using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine.Rendering;

// From Unity documentation: OpenGL ES 3.1 (for (Android, iOS, tvOS platforms) only guarantees support for 4 compute buffers at a time.

namespace RainyReignGames.SkinnedGPURaycast
{
    public static class ShaderPhysics
    {
        private const float MAX_DISTANCE = 1024f;

        private static readonly string kernelName = "ShaderRaycast";
        private static int kernelIndex;
        private static readonly CommandBuffer commandBuffer = new ();
        private static ComputeShader _computeShader;
        private static Dictionary<Mesh, GraphicsBuffer> cache = new();
        public const int RaycastHitStride = 10 * sizeof(float) + 5 * sizeof(uint); // Stride must match sizeof(RaycastHit)
        public const int RayStride = 6 * sizeof(float) + sizeof(uint);
        private static int verticesID;
        private static int trianglesID;
        private static int raysID;
        private static int maxDistanceID;
        private static int indexStrideId;
        private static int worldToLocalMatrixID;
        private static int resultID;
        private static ProfilerMarker bufferMarker = new ProfilerMarker("Buffer get & create");

        private static List<RaycastData> raycastQueue = new List<RaycastData>(10);
        private static Dictionary<ComputeBuffer, int> rayBuffers = new ();
        
        /// <summary>
        /// Hit data from the raycast.
        /// All relevant properties are in object space. If transformation is required, use a callback function.
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct RaycastHit
        {
            /// <summary>
            /// Barycentric coordinates of the hit point.
            /// </summary>
            public float3 barycentricCoordinate;
            
            /// <summary>
            /// The triangle's normal vector in object space. This is computed from the cross product of the edges,
            /// so it is distinct from the vertex normals.
            /// </summary>
            public float3 faceNormal;
            
            /// <summary>
            /// The coordinates of the raycast hit in object space.
            /// </summary>
            public float3 hitPoint;
            
            // public float2 textureCoord;
            
            /// <summary>
            /// The distance from the ray's origin to the hit point in object space.
            /// </summary>
            public float distance;

            /// <summary>
            /// The indices into the vertex attributes for this triangle. This is the same information in the index buffer,
            /// e.g., using <c>mesh.triangles[triangleIndex]</c>.
            /// </summary>
            public uint3 triangle;
            
            /// <summary>
            /// The index of the hit triangle in an array of <c>int3</c>. If using an array of <c>int</c>,
            /// multiply by 3 to get the start index of the triangle.
            /// </summary>
            public uint triangleIndex;
            
            /// <summary>
            /// The id field of the relevant ray.
            /// </summary>
            public uint rayId;

            /// <summary>
            /// Interpolates the attribute value at the hit point.
            /// </summary>
            /// <param name="attribute">An IList containing the vertex attribute.</param>
            /// <returns>The interpolated attribute</returns>
            public Vector2 Interpolate(IList<Vector2> attribute)
            {
                return barycentricCoordinate.x * attribute[(int)triangle.x] +
                       barycentricCoordinate.y * attribute[(int)triangle.y] +
                       barycentricCoordinate.z * attribute[(int)triangle.z];
            }
            /// <summary>
            /// Interpolates the attribute value at the hit point.
            /// </summary>
            /// <param name="attribute">An IList containing the vertex attribute.</param>
            /// <returns>The interpolated attribute</returns>
            public Vector3 Interpolate(IList<Vector3> attribute)
            {
                return barycentricCoordinate.x * attribute[(int)triangle.x] +
                       barycentricCoordinate.y * attribute[(int)triangle.y] +
                       barycentricCoordinate.z * attribute[(int)triangle.z];
            }
            /// <summary>
            /// Interpolates the attribute value at the hit point.
            /// </summary>
            /// <param name="attribute">An IList containing the vertex attribute.</param>
            /// <returns>The interpolated attribute</returns>
            public Vector4 Interpolate(IList<Vector4> attribute)
            {
                return barycentricCoordinate.x * attribute[(int)triangle.x] +
                       barycentricCoordinate.y * attribute[(int)triangle.y] +
                       barycentricCoordinate.z * attribute[(int)triangle.z];
            }
        };

        public struct Ray
        {
            public float3 origin;
            public float3 direction;
            public uint id;

            public Ray(float3 origin, float3 direction)
            {
                this.origin = origin;
                this.direction = direction;
                this.id = math.hash(new float3x2(origin, direction));
            }
            public Ray(float3 origin, float3 direction, uint id)
            {
                this.origin = origin;
                this.direction = direction;
                this.id = id;
            }
            
            public static implicit operator Ray(UnityEngine.Ray ray)
            {
                return new Ray()
                {
                    origin = ray.origin,
                    direction = ray.direction,
                    id = math.hash(new float3x2(ray.origin, ray.direction))
                };
            }

            public static implicit operator UnityEngine.Ray(Ray ray)
            {
                return new UnityEngine.Ray(ray.origin, ray.direction);
            }
        }
        
        static ShaderPhysics()
        {
            
        }
        
        public static NativeArray<RaycastHit> EmptyHits() => new(0, Allocator.Temp);

        [RuntimeInitializeOnLoadMethod]
        static void Initialize()
        {
            commandBuffer.name = "ShaderRaycast";
            _computeShader = Resources.Load<ComputeShader>("ShaderRaycast");
            unsafe
            {
                Debug.Assert(sizeof(RaycastHit) == RaycastHitStride, "sizeof(RaycastHit) does not match RaycastHitStride!");
            }

            if(_computeShader is null)
            {
                Debug.LogError("ShaderRaycast compute shader not found!");
                return;
            }

            kernelIndex = _computeShader.FindKernel(kernelName);
            verticesID = Shader.PropertyToID("_Vertices");
            trianglesID = Shader.PropertyToID("_Triangles");
            raysID = Shader.PropertyToID("_Rays");
            maxDistanceID = Shader.PropertyToID("maxDistance");
            indexStrideId = Shader.PropertyToID("indexStride");
            worldToLocalMatrixID = Shader.PropertyToID("worldToLocalMatrix");
            resultID = Shader.PropertyToID("Result");

            if (!_computeShader.IsSupported(kernelIndex)) Debug.LogError("ShaderRaycast Compute Shader not supported on this platform!");

            if (QualitySettings.renderPipeline is not null || GraphicsSettings.currentRenderPipeline is not null)
                RenderPipelineManager.beginContextRendering += OnbeginContextRendering;
            else
                Camera.onPreRender += OnPreRender;
        }

        private static void OnbeginContextRendering(ScriptableRenderContext context, List<Camera> cameras)
        {
            ProcessQueue();
        }

        private static void OnPreRender(Camera camera)
        {
            ProcessQueue();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ProcessQueue()
        {
            foreach(var item in raycastQueue) item.ProcessItem();
            raycastQueue.Clear();
            
            Graphics.ExecuteCommandBuffer(commandBuffer);
            commandBuffer.Clear();
        }


        struct RaycastData : IDisposable
        {
            internal SkinnedMeshRenderer smr;
            internal float maxDistance;
            internal GraphicsBuffer indexBuffer;
            internal ComputeBuffer resultBuffer;
            internal ComputeBuffer countBuffer;
            internal ComputeBuffer rayBuffer;
            internal Action<NativeArray<RaycastHit>, SkinnedMeshRenderer, Matrix4x4> raycastCallback;//Matri4x4 is the localToWorld matrix at the time when the raycast was invoked.
            internal Matrix4x4 localToWorldMatrix;

            public void Dispose()
            {
                indexBuffer.Dispose();
                resultBuffer.Dispose();
                countBuffer.Dispose();
                
                if (rayBuffers.TryGetValue(rayBuffer, out var bufferCount))
                {
                    if (bufferCount <= 1)
                    {
                        rayBuffers.Remove(rayBuffer);
                        rayBuffer.Dispose();
                    }
                    else
                        rayBuffers[rayBuffer] = bufferCount - 1;
                }
                else
                {
                    rayBuffer.Dispose();
                }
            }
            
            internal void ProcessItem()
            {
                var vertexBuffer = smr.GetVertexBuffer();
                if (vertexBuffer is null)
                {
                    Debug.LogError($"Could not get vertex buffer.");
                    Dispose();
                    return;
                }
                
                localToWorldMatrix = smr.rootBone.localToWorldMatrix;

                commandBuffer.SetComputeBufferParam(_computeShader, kernelIndex, verticesID, vertexBuffer);
                commandBuffer.SetComputeBufferParam(_computeShader, kernelIndex, trianglesID, indexBuffer);
                commandBuffer.SetComputeBufferParam(_computeShader, kernelIndex, raysID, rayBuffer);
                commandBuffer.SetComputeIntParam(_computeShader, indexStrideId, indexBuffer.stride);
                commandBuffer.SetComputeFloatParam(_computeShader, maxDistanceID, maxDistance);
                commandBuffer.SetComputeMatrixParam(_computeShader, worldToLocalMatrixID, smr.rootBone.worldToLocalMatrix); //TODO, consider converting rays to local space before sending and removing the need for worldToLocalMatrix
                commandBuffer.SetComputeBufferParam(_computeShader, kernelIndex, resultID, resultBuffer);

                var triCount = indexBuffer.count / 3;
                
                commandBuffer.DispatchCompute(_computeShader, kernelIndex, (int)math.ceil(triCount / 64f), 1, 1);
                commandBuffer.CopyCounterValue(resultBuffer, countBuffer, 0);

                
                var raycastData = this;
                var counter = new NativeArray<int>(1, Allocator.Persistent);
                AsyncGPUReadbackRequest counterRequest = default;
                commandBuffer.RequestAsyncReadbackIntoNativeArray(ref counter, countBuffer, 1, 0, counterCallback);
                commandBuffer.RequestAsyncReadback(resultBuffer, resultCallback);

                void counterCallback(AsyncGPUReadbackRequest request)
                {
                    counterRequest = request;
                }
                void resultCallback(AsyncGPUReadbackRequest request)
                {
                    if (!counterRequest.done) Debug.LogWarning($"resultRequest completed first!");
                    counterRequest.WaitForCompletion();

                    NativeArray<RaycastHit> results;
                    try
                    {
                        results = request.GetData<RaycastHit>();
                        if (counter[0] < results.Length)
                            results = results.GetSubArray(0, counter[0]);
                    }
                    catch (Exception e)
                    {
                        results = EmptyHits();
                        Debug.LogError(e);
                    }

                    raycastData.raycastCallback?.Invoke(results, raycastData.smr, raycastData.localToWorldMatrix);
                
                    counter.Dispose();
                    vertexBuffer.Dispose();
                    raycastData.Dispose();
                }
            }
        }

        /// <summary>
        /// Perform a GPU-based raycast using the given ray(s), with the given renderer(s) as possible targets. The GPU will typically take 2-3 frames
        /// to complete the raycast, after which the main thread will invoke the callback function. 
        /// </summary>
        /// <param name="renderers">An array of SkinnedMeshRenderers that will be considered targets of the raycast</param>
        /// <param name="rays">An array of rays that will be used for the cast</param>
        /// <param name="raycastCallback">Callback function that will be invoked on each renderer after the GPU raycast is completed</param>
        public static void RaycastAll(SkinnedMeshRenderer[] renderers, Ray[] rays, Action<NativeArray<RaycastHit>, SkinnedMeshRenderer, Matrix4x4> raycastCallback = null)
        {
            RaycastAll(renderers, rays, MAX_DISTANCE, raycastCallback);
        }

        /// <summary>
        /// Perform a GPU-based raycast using the given ray(s), with the given renderer(s) as possible targets. The GPU will typically take 2-3 frames
        /// to complete the raycast, after which the main thread will invoke the callback function.
        /// </summary>
        /// <param name="renderers">An array of SkinnedMeshRenderers that will be considered targets of the raycast</param>
        /// <param name="rays">An array of rays that will be used for the cast</param>
        /// <param name="maxDistance">The maximum distance of a hit point</param>
        /// <param name="raycastCallback">Callback function that will be invoked on each renderer after the GPU raycast is completed</param>
        public static void RaycastAll(SkinnedMeshRenderer[] renderers, Ray[] rays, float maxDistance = MAX_DISTANCE, Action<NativeArray<RaycastHit>, SkinnedMeshRenderer, Matrix4x4> raycastCallback = null)
        {
            ComputeBuffer rayBuffer = null;
            var nativeRays = new NativeArray<Ray>(rays.Length, Allocator.Temp);
            int rayCount = 0;
            
            var bounds = new NativeArray<Bounds>(renderers.Length, Allocator.Temp);
            for (int rendererIndex = 0; rendererIndex < renderers.Length; rendererIndex++)
                bounds[rendererIndex] = renderers[rendererIndex].bounds;
            
            for (int rayIndex = 0; rayIndex < rays.Length; rayIndex++)
            {
                for (int index = 0; index < bounds.Length; index++)
                {
                    if (bounds[index].IntersectRay(rays[rayIndex]))
                    {
                        nativeRays[rayCount] = rays[rayIndex];
                        rayCount++;
                        break;
                    }
                }
            }
            
            for (int rendererIndex = 0; rendererIndex < renderers.Length; rendererIndex++)
            {
                if (rayCount > 0)
                {
                    if(rayBuffer is null)
                    {
                        rayBuffer = new ComputeBuffer(rayCount, RayStride, ComputeBufferType.Structured); // stride is sizeof(Ray)
                        rayBuffer.SetData(nativeRays, 0, 0, rayCount);
                    }
                    RaycastAll(renderers[rendererIndex], rayBuffer, maxDistance, raycastCallback);
                }
                else
                {
                    raycastCallback?.Invoke(EmptyHits(), renderers[rendererIndex], renderers[rendererIndex].rootBone.localToWorldMatrix);
                }
            }
        }

        /// <summary>
        /// Perform a GPU-based raycast using the given ray(s), with the given renderer(s) as possible targets. The GPU will typically take 2-3 frames
        /// to complete the raycast, after which the main thread will invoke the callback function.
        /// </summary>
        /// <param name="smr">The SkinnedMeshRenderer that will be the target of the raycast</param>
        /// <param name="rays">An array of rays that will be used for the cast</param>
        /// <param name="raycastCallback">Callback function that will be invoked on the renderer after the GPU raycast is completed</param>
        public static void RaycastAll(SkinnedMeshRenderer smr, Ray[] rays, Action<NativeArray<RaycastHit>, SkinnedMeshRenderer, Matrix4x4> raycastCallback = null)
        {
            RaycastAll(smr, rays, MAX_DISTANCE, raycastCallback);
        }

        /// <summary>
        /// Perform a GPU-based raycast using the given ray(s), with the given renderer(s) as possible targets. The GPU will typically take 2-3 frames
        /// to complete the raycast, after which the main thread will invoke the callback function.
        /// </summary>
        /// <param name="smr">The SkinnedMeshRenderer that will be the target of the raycast</param>
        /// <param name="rays">An array of rays that will be used for the cast</param>
        /// <param name="maxDistance">The maximum distance of a hit point</param>
        /// <param name="raycastCallback">Callback function that will be invoked on the renderer after the GPU raycast is completed</param>
        public static void RaycastAll(SkinnedMeshRenderer smr, ComputeBuffer rays, float maxDistance = MAX_DISTANCE, Action<NativeArray<RaycastHit>, SkinnedMeshRenderer, Matrix4x4> raycastCallback = null)
        {
            var mesh = smr.sharedMesh;
            if (mesh is null)
            {
                Debug.LogWarning($"Mesh is null.");
                return;
            }
            
            // var uvStream = mesh.GetVertexAttributeStream(VertexAttribute.TexCoord0);

            SetBufferTargets(smr, mesh);

            bufferMarker.Begin();
            RaycastData raycastData = new RaycastData
            {
                smr = smr,
                maxDistance = maxDistance,
                indexBuffer = mesh.GetIndexBuffer(),
                resultBuffer = new ComputeBuffer(rays.count * 4, RaycastHitStride, ComputeBufferType.Append),
                countBuffer = new ComputeBuffer(1, sizeof(int), ComputeBufferType.IndirectArguments),
                rayBuffer = rays,
                raycastCallback = raycastCallback
            };

            rayBuffers.TryGetValue(rays, out int buffersCount);
            rayBuffers[rays] = buffersCount + 1;
            raycastData.resultBuffer.SetCounterValue(0);

            raycastQueue.Add(raycastData);
            bufferMarker.End();
        }

        /// <summary>
        /// Perform a GPU-based raycast using the given ray(s), with the given renderer(s) as possible targets. The GPU will typically take 2-3 frames
        /// to complete the raycast, after which the main thread will invoke the callback function.
        /// </summary>
        /// <param name="smr">The SkinnedMeshRenderer that will be the target of the raycast</param>
        /// <param name="rays">An array of rays that will be used for the cast</param>
        /// <param name="maxDistance">The maximum distance of a hit point</param>
        /// <param name="raycastCallback">Callback function that will be invoked on the renderer after the GPU raycast is completed</param>
        public static void RaycastAll(SkinnedMeshRenderer smr, Ray[] rays, float maxDistance = MAX_DISTANCE, Action<NativeArray<RaycastHit>, SkinnedMeshRenderer, Matrix4x4> raycastCallback = null)
        {
            if (rays is null || rays.Length == 0)
            {
                raycastCallback?.Invoke(EmptyHits(), smr, smr.rootBone.localToWorldMatrix);
                return;
            }
            
            var rayBuffer = new ComputeBuffer(rays.Length, RayStride, ComputeBufferType.Structured); // stride is sizeof(Ray)
            rayBuffer.SetData(rays);
            RaycastAll(smr, rayBuffer, maxDistance, raycastCallback);
        }

        /// <summary>
        /// Perform a GPU-based raycast using the given ray, with the given renderer as possible targets. The GPU will typically take 2-3 frames
        /// to complete the raycast, after which the main thread will invoke the callback function.
        /// </summary>
        /// <param name="smr">The SkinnedMeshRenderer that will be the target of the raycast</param>
        /// <param name="ray">The Ray that will be used for the cast</param>
        /// <param name="maxDistance">The maximum distance of a hit point</param>
        /// <param name="raycastCallback">Callback function that will be invoked on the renderer after the GPU raycast is completed</param>
        public static void Raycast(SkinnedMeshRenderer smr, Ray ray, float maxDistance = MAX_DISTANCE,
            Action<NativeArray<RaycastHit>, SkinnedMeshRenderer, Matrix4x4> raycastCallback = null)
        {
            if (!smr.bounds.IntersectRay(ray))
            {
                raycastCallback?.Invoke(EmptyHits(), smr, smr.rootBone.localToWorldMatrix);
                return;
            }
            
            var mesh = smr.sharedMesh;
            if (mesh is null)
            {
                Debug.LogWarning($"Mesh is null.");
                return;
            }
            
            SetBufferTargets(smr, mesh);
            
            bufferMarker.Begin();
            RaycastData raycastData = new RaycastData
            {
                smr = smr,
                maxDistance = maxDistance,
                indexBuffer = mesh.GetIndexBuffer(),
                resultBuffer = new ComputeBuffer(4, RaycastHitStride, ComputeBufferType.Append),
                countBuffer = new ComputeBuffer(1, sizeof(int), ComputeBufferType.IndirectArguments),
                rayBuffer = new ComputeBuffer(1, RayStride, ComputeBufferType.Structured), // stride is sizeof(Ray),
                raycastCallback = raycastCallback
            };


            var rays = new NativeArray<Ray>(1, Allocator.Temp);
            rays[0] = ray;
            raycastData.rayBuffer.SetData(rays);
            raycastData.resultBuffer.SetCounterValue(0);

            raycastQueue.Add(raycastData);
            bufferMarker.End();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void SetBufferTargets(SkinnedMeshRenderer smr, Mesh mesh)
        {
#if UNITY_EDITOR
            if ((smr.vertexBufferTarget & GraphicsBuffer.Target.Raw) == 0)
                Debug.LogWarning($"On some platforms, VertexBufferTarget on {smr.name} must be set to Raw in an earlier frame to avoid data corruption for the first time accessing the vertex buffer in the same frame.", smr);
#endif
            smr.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
            mesh.indexBufferTarget |= GraphicsBuffer.Target.Raw;
        }
    }
}