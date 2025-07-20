// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;
using UnityEditor;

namespace VinforlabTeam.FurClimbingSystem
{
    [CustomEditor(typeof(SurfaceGrab))]
    public class SurfaceGrabDrawer : Editor
    {
        public override void OnInspectorGUI()
        {
            if(GUILayout.Button("Talk to support", GUILayout.Height(30)))
            {
                Application.OpenURL("http://discord.gg/R5S7WeWyqc");
            }
            DrawDefaultInspector();       
        }
    }
}