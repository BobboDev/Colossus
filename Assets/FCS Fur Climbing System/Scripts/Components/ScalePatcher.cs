// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections;
using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class ScalePatcher : MonoBehaviour
    {
        [SerializeField]
        private Vector3 expectedScale = new Vector3(1,1,1);
        public bool useStartScale = true;

        public Vector3 ExpectedScale
        {
            get { return expectedScale; } set { expectedScale = value; }
        }

        public FCSTypes.UpdateMode updateMode = FCSTypes.UpdateMode.LATE_UPDATE;

        public void SetExpectedScale(Vector3 scale)
        {
            ExpectedScale = scale;
        }

        private void Start()
        {
            if (useStartScale)
            {
                ExpectedScale = transform.localScale;
            }

            StartCoroutine(IEPatchScale());
        }

        public void SetGlobalScale( Transform transform, Vector3 globalScale)
        {
            transform.localScale = Vector3.one;
            transform.localScale = new Vector3(globalScale.x / transform.lossyScale.x, globalScale.y / transform.lossyScale.y, globalScale.z / transform.lossyScale.z);
        }

        private void PatchScale()
        {
            SetGlobalScale(transform, ExpectedScale);
        }

        private void FixedUpdate()
        {
            if(updateMode == FCSTypes.UpdateMode.FIXED_UPDATE)
            {
                PatchScale();
            }
        }

        private void Update()
        {
            if (updateMode == FCSTypes.UpdateMode.UPDATE)
            {
                PatchScale();
            }
        }

        private void LateUpdate()
        {
            if (updateMode == FCSTypes.UpdateMode.LATE_UPDATE)
            {
                PatchScale();
            }
        }

        IEnumerator IEPatchScale()
        {
            while (true)
            {
                if (updateMode == FCSTypes.UpdateMode.IENUMERATOR)
                {
                    PatchScale();
                }
                yield return null;
            }
        }
    }
}
