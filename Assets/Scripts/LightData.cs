using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;

public class LightData : ScriptableObject
{

    [SerializeField] public string presetname = "";

    // *** Environment Settings ***
    
    [SerializeField] public Material skyboxMaterial;
    [SerializeField] public AmbientMode lightingSource;
    
    [SerializeField] [ColorUsage(false, true)] public Color ambientColour = new Color(54f/255f,58f/255f,56f/255f,1f);
    [SerializeField] [Range(0f, 8f)] public float intensityMultiplier = 1;

    [SerializeField] [ColorUsage(false, true)] public Color skyColour = new Color(54f/255f,58f/255f,66f/255f,1f);
    [SerializeField] [ColorUsage(false, true)] public Color equatorColour = new Color(29f/255f,32f/255f,34f/255f,1f);
    [SerializeField] [ColorUsage(false, true)] public Color groundColour = new Color(12f/255f,11f/255f,9f/255f,1f);

    // *** Directional Light Settings ***
    
    [SerializeField] public Vector3 directionalLightRotation;

    [SerializeField] public Light directionalLight = new Light();
    [SerializeField] public Color directionalLightColor = Color.white;
    [SerializeField] public float directionalLightIntensity = 1;
    [SerializeField] [Range(0f, 1f)] public float directionalLightShadowStrength = 1;

    // *** Point Light Settings ***
    
    [SerializeField] public Vector3 pointLightPosition;

    [SerializeField] public Light pointLight = new Light();
    [SerializeField] public Color pointLightColor = Color.white;
    [SerializeField] public float pointLightIntensity = 1;
    [SerializeField] [Range(0f, 1f)] public float pointLightShadowStrength = 1;
    [SerializeField] public float pointLightRange = 10;

    // *** Spot Light Settings ***

    [SerializeField] public Vector3 spotLightPosition;
    [SerializeField] public Vector3 spotLightRotation;

    [SerializeField] public Light spotLight = new Light();
    [SerializeField] public Color spotLightColor = Color.white;
    [SerializeField] public float spotLightIntensity = 1;
    [SerializeField] [Range(0f, 1f)] public float spotLightShadowStrength = 1;
    [SerializeField] public float spotLightRange = 10;
    [SerializeField] public float spotLightAngle = 30;
}

