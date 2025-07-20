using UnityEngine;
using Unity.Collections;
using RainyReignGames.SkinnedGPURaycast;

[RequireComponent(typeof(Camera))]
public class ScreenRaycast : MonoBehaviour
{
    public SkinnedMeshRenderer[] targets;
    public HitVisual hitPrefab;
    public bool breakOnRaycast;

    Camera cam;
    ShaderPhysics.Ray[] rays;

    // Start is called before the first frame update
    void Start()
    {
        rays = new ShaderPhysics.Ray[1];
        cam = GetComponent<Camera>();
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetMouseButtonUp(0) && targets != null)
        {
            Ray ray = cam.ScreenPointToRay(Input.mousePosition);
            rays[0] = ray;

            ShaderPhysics.RaycastAll(targets, rays, RaycastCallback);
            if (breakOnRaycast)
                Debug.Break();
        }
    }

    void RaycastCallback(NativeArray<ShaderPhysics.RaycastHit> hits, SkinnedMeshRenderer smr, Matrix4x4 localToWorldMatrix)
    {
        if (hitPrefab)
        {
            foreach (var hit in hits)
            {
                Instantiate(hitPrefab, localToWorldMatrix.MultiplyPoint3x4(hit.hitPoint), Quaternion.identity);
            }
        }
        else
        {
            Debug.LogError("HitPrefab is not assigned!");
        }
    }
}
