using UnityEngine;
using Overhang;

public static class ClimbDetectionUtility
{
    /// <summary>
    /// Tries to raycast for a climbable mesh from a given origin and direction.
    /// </summary>
    /// <param name="raycastOriginTransform">The Transform the raycast origin is calculated from.</param>
    /// <param name="layerMask">LayerMask to filter climbable objects.</param>
    /// <param name="hit">Output RaycastHit if a hit occurred.</param>
    /// <param name="climbableMesh">Output ClimbableMesh if found on hit collider.</param>
    /// <returns>True if a climbable mesh was hit.</returns>
    public static bool TryGetClimbableMesh(Transform raycastOriginTransform, Vector3 previousRayCastOrigin, LayerMask layerMask, out RaycastHit hit, out ClimbableMesh climbableMesh)
    {
        float maxHitDistance = 0.2f;
        Vector3 raycastDirection = -raycastOriginTransform.up;
        Vector3 raycastOrigin = raycastOriginTransform.position + raycastOriginTransform.up * 0.1f;
        Vector3 previousRaycastOriginToNew = (previousRayCastOrigin - raycastOriginTransform.position).normalized;


        // If a raycast from the previous raycast position to the new raycast position hits a climbable mesh (To cover the distance last travelled so we don't clip through a model)
        // OR a raycast straight down hits a climbable mesh
        if (Physics.Raycast(previousRayCastOrigin, previousRaycastOriginToNew, out var raycastHit, Vector3.Distance(previousRayCastOrigin, raycastOriginTransform.position), layerMask)
         || Physics.Raycast(raycastOrigin, raycastDirection, out raycastHit, maxHitDistance, layerMask))
        {
            climbableMesh = raycastHit.collider.GetComponent<ClimbableMesh>();
            hit = raycastHit;
            return true;
        }
        else
        {
            climbableMesh = null;
            hit = default;
            return false;
        }
    }
}