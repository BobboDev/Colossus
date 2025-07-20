// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class FCSAux : MonoBehaviour
    {
        public static FCSAux CreateAux(string name, Transform parent = null)
        {
            GameObject obj = new GameObject();
            FCSAux aux = obj.AddComponent<FCSAux>();
            obj.name = name + " " + aux.GetHashCode();
            if(parent != null)
                obj.transform.SetParent(parent);
            return aux;
        }
    }
}