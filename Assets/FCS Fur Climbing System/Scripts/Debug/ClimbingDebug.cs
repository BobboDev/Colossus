// (c) Copyright VinforLab Team. All rights reserved.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace VinforlabTeam.FurClimbingSystem
{
    public class ClimbingDebug : MonoBehaviour
    {
        public SurfaceGrab surfaceGrab;
        public TriangleProvider triangleProvider;
        public GameObject debugText;
        public Transform container;

        Dictionary<string, Text> texts = new Dictionary<string, Text>();

        private void CreateText(string title, params string[] values)
        {
            if (texts.ContainsKey(title))
            {
                texts[title].text = string.Format(Bold(title), values);
                return;
            }

            GameObject obj = Instantiate(debugText, container);
            Text text = obj.GetComponent<Text>();
            text.text = string.Format(Bold(title), values);
            texts.Add(title, text);
        }

        private string[] Vector3ToString(Vector3 vec)
        {
            return new string[] { vec.x.ToString(), vec.y.ToString(), vec.z.ToString()  };
        }

        string Bold(string str)
        {
            return "<b>" + str +"</b>";
        }

        public void Log(string txt)
        {
            Debug.Log(txt);
        }

        void Update()
        {
            if(surfaceGrab.CurrentTriangle == null || !surfaceGrab.IsOnClimbing)
            {
                CreateText("Climbing Status: {0}", "Not On Climbing");
                return;
            }

            CreateText("Climbing Status: {0}", "On Climbing");


            Vector3 normalPos = surfaceGrab.CurrentTriangle.ComputeNormalizePosition();
            Vector3 normalRot = surfaceGrab.CurrentTriangle.triangleRotation.eulerAngles;


            CreateText("Current Triangle Index: {0}", surfaceGrab.CurrentTriangleIndex.ToString());
            CreateText("Current Triangle Position:\n X: {0} Y: {1} Z: {2}", Vector3ToString(normalPos));
            CreateText("Current Triangle Rotation:\n X: {0} Y: {1} Z: {2}", Vector3ToString(normalRot));
        }
    }
}