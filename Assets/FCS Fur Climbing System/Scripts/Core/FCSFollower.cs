// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;
using UnityEngine.Animations;

namespace VinforlabTeam.FurClimbingSystem
{
    public class FCSFollower : MonoBehaviour
    {
        [HideInInspector]
        public SurfaceGrab target;

        [HideInInspector]
        public SurfaceWalker targetWalker;

        [HideInInspector]
        public ParentConstraint parentConstraint;

        [HideInInspector]
        public Transform child;

        private ConstraintSource constraintSource;

        public static FCSFollower CreateFollower()
        {
            GameObject obj = new GameObject();
            obj.name = "FCS Follower" + obj.GetHashCode();
            FCSFollower follower = obj.AddComponent<FCSFollower>();
            follower.parentConstraint = obj.AddComponent<ParentConstraint>();

            GameObject child = new GameObject();
            child.name = "FCS Follower Child" + child.GetHashCode();
            child.transform.parent = obj.transform;
            child.transform.localPosition = Vector3.zero;

            follower.child = child.transform;

            follower.constraintSource.sourceTransform = obj.transform;
            follower.constraintSource.weight = 1;
            follower.parentConstraint.AddSource(follower.constraintSource);
            return follower;
        }

        public void FollowBasic()
        {
            if (targetWalker == null)
            {
                transform.position = target.CurrentTriangleProvider.transform.position;
                transform.rotation = target.CurrentTriangleProvider.transform.rotation;
            }
            else if (target == null)
            {
                transform.position = targetWalker.CurrentTriangleProvider.transform.position;
                transform.rotation = targetWalker.CurrentTriangleProvider.transform.rotation;
            }
        }

        public void Follow()
        {
            if (targetWalker == null)
            {
                constraintSource.sourceTransform = target.CurrentTriangleProvider.transform;
                parentConstraint.constraintActive = true;
                parentConstraint.SetSource(0, constraintSource);
            }
            else
            {
                constraintSource.sourceTransform = targetWalker.CurrentTriangleProvider.transform;
                parentConstraint.constraintActive = true;
                parentConstraint.SetSource(0, constraintSource);
            }
        }
    }
}