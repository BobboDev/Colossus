using UnityEngine;
using Unity.Collections;
using RainyReignGames.SkinnedGPURaycast;
using Ray = RainyReignGames.SkinnedGPURaycast.ShaderPhysics.Ray;

[RequireComponent(typeof(ParticleSystem))]
public class RainManager : MonoBehaviour
{
    public SkinnedMeshRenderer[] renderers;
    public GameObject splashPrefab;

    ParticleSystem ps;
    int maxParticles;
    NativeArray<ParticleSystem.Particle> particles;
    Ray[] rays;
    float maxDistance = 1024f;

    // Start is called before the first frame update
    void Start()
    {
        ps = GetComponent<ParticleSystem>();
        maxParticles = ps.main.maxParticles;

        particles = new NativeArray<ParticleSystem.Particle>(maxParticles, Allocator.Persistent);
        rays = new Ray[maxParticles];

        maxDistance = Mathf.Abs(ps.velocityOverLifetime.y.constant);
    }

    // Update is called once per frame
    void Update()
    {
        ps.GetParticles(particles);

        if (renderers != null && Time.frameCount > 10)
        {
            UpdateRays();

            ShaderPhysics.RaycastAll(renderers, rays, maxDistance, RaycastCallback);
        }
    }

    //TODO, burst compile?
    private void UpdateRays()
    {
        if (rays != null)
        {
            for (int i = 0; i < particles.Length; i++)
            {
                rays[i].origin = particles[i].position;
                rays[i].direction = Vector3.down;
                rays[i].id = (uint)i;
            }
        }
    }

    /*private void OnDrawGizmos()
    {
        if (rays != null)
        {
            Gizmos.color = Color.red;
            for (int i = 0; i < rays.Length; i++)
            {
                Gizmos.DrawCube(rays[i].origin, Vector3.one * 0.1f);
                Gizmos.DrawRay(rays[i].origin, rays[i].direction);
            }
        }
    }*/

    private void OnDisable()
    {
        particles.Dispose();
    }

    void RaycastCallback(NativeArray<ShaderPhysics.RaycastHit> hits, SkinnedMeshRenderer renderer, Matrix4x4 localToWorldMatrix)
    {
        if (hits.Length > 0 && particles.IsCreated)
        {
            float shortest = float.MaxValue;
            Vector3 closest = Vector3.zero;
            int closestID = -1;
            for (int i = 0; i < hits.Length; i++)
            {
                if (hits[i].distance < shortest)
                {
                    shortest = hits[i].distance;
                    closest = hits[i].hitPoint;
                    closestID = (int)hits[i].rayId;
                }
            }

            Vector3 psScale = ps.shape.scale;

            ParticleSystem.Particle particle = particles[closestID];
            particle.position = new Vector3(Random.Range(psScale.x * -0.5f, psScale.x * 0.5f), transform.position.y, Random.Range(psScale.z * -0.5f, psScale.z * 0.5f));
            particles[closestID] = particle;

            ps.SetParticles(particles);
            Instantiate(splashPrefab, localToWorldMatrix.MultiplyPoint3x4(closest), Quaternion.identity);
        }
    }
}
