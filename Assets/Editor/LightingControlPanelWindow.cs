using System.Collections;
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;
using UnityEditor.Presets;
using UnityEngine.Rendering;

public class LightingControlPanelWindow : EditorWindow
{

    Texture2D lightGreyTexture;
    Texture2D darkGreyTexture;

    Color darkGrey = new Color(0.2f,0.2f,0.2f,1f);
    Color lightGrey = new Color(0.25f,0.25f,0.25f,1f);

    Rect body;
    Rect headerSection;
    Rect environmentSection;
    Rect headerSectionDivider;
    Rect directionalLightSection;
    Rect pointLightSection;
    Rect spotLightSection;
    Rect footerSection;

    public static LightData lastData;
    public static LightData data;
    public static LightData currentLightSettings;
    public static LightingData lightingData;
    bool started;


    Light[] lights;
    Light directionalLight;
    Light pointLight;
    Light spotLight;

    Vector2 scrollPosition;

    bool changed;

    // This method will be called on load or recompile
    [InitializeOnLoadMethod]
    private static void OnLoad()
    {
        if (!lightingData)
        {
            lightingData = AssetDatabase.LoadAssetAtPath<LightingData>("Assets/Data/LightingPreferences.asset");
        }

        if (!data)
        {
            data = AssetDatabase.LoadAssetAtPath<LightData>("Assets/Data/CurrentLightSettings.asset");
            currentLightSettings = data;
            lastData = lightingData.currentLightPreset;

            if(data) return;

            data = CreateInstance<LightData>();

            AssetDatabase.CreateAsset(data,"Assets/Data/CurrentLightSettings.asset");
            AssetDatabase.Refresh();
        }
    }

    [MenuItem("Window/Lighting Control Panel")]
    static void OpenWindow()
    {
        LightingControlPanelWindow window = (LightingControlPanelWindow)GetWindow(typeof(LightingControlPanelWindow));
        window.minSize = new Vector2(100,100);
        window.Show();
    }

    void OnEnable()
    {
        InitTextures();
    }

    void Awake()
    {
        lights = FindObjectsOfType<Light>();
        directionalLight = lights.ToList().FirstOrDefault(light => light.type == LightType.Directional);
        pointLight = lights.ToList().FirstOrDefault(light => light.type == LightType.Point);
        spotLight = lights.ToList().FirstOrDefault(light => light.type == LightType.Spot);
    }


    void InitTextures()
    {
        lightGreyTexture = new Texture2D(1,1);
        lightGreyTexture.SetPixel(0,0,lightGrey);
        lightGreyTexture.Apply();
        darkGreyTexture = new Texture2D(1,1);
        darkGreyTexture.SetPixel(0,0,darkGrey);
        darkGreyTexture.Apply();
    }

    void OnGUI()
    {   
        float scrollheight = body.width > 450 ? headerSection.height + directionalLightSection.height + 10 : headerSection.height + directionalLightSection.height + pointLightSection.height + spotLightSection.height;

        scrollPosition = GUILayout.BeginScrollView(scrollPosition, GUILayout.Width(Screen.width), GUILayout.Height(Screen.height - headerSection.height));
            GUILayout.BeginArea(body);
                DrawLayouts();
                DrawEnvironmentSettings(data);
                DrawDirectionalLightSettings(data);
                DrawPointLightSettings(data);
                DrawSpotLightSettings(data);

                if (GUI.changed || changed)
                {
                    ApplyEnvironmentSettings();
                    ApplyDirectionalLightSettings();
                    ApplyPointLightSettings();
                    ApplySpotLightSettings();
                }
                else
                {
                    GetEnvironmentSettings();
                    GetDirectionalLightSettings();
                    GetPointLightSettings();
                    GetSpotLightSettings();
                }
                changed = false;
            GUILayout.EndArea();
            GUILayout.Label("", GUILayout.Width(0), GUILayout.Height(scrollheight));
        GUILayout.EndScrollView();
        DrawHeader();

        Repaint();
    }
    


    void DrawLayouts()
    {
        body.x = 0;
        body.y = 0;
        if (Screen.width > 450 )
        {
            body.width = Screen.height > environmentSection.height + directionalLightSection.height + headerSection.height ? Screen.width : Screen.width - 10;
        }
        else
        {
            body.width = Screen.height < environmentSection.height + directionalLightSection.height + pointLightSection.height + spotLightSection.height + headerSection.height - 10 ? Screen.width - 10 : Screen.width;
        }
        body.height = 720;

        headerSection.x = 5;
        headerSection.y = Screen.height - 120;
        headerSection.width = Screen.width - 10;
        headerSection.height = 120;
        
        environmentSection.x = 5;
        environmentSection.y = 0;
        environmentSection.width = body.width - 10;
        environmentSection.height = currentLightSettings.lightingSource == AmbientMode.Trilight ? 135 : 85;
        
        headerSectionDivider.x = 0;
        headerSectionDivider.y = environmentSection.height - 5;
        headerSectionDivider.width = body.width;
        headerSectionDivider.height = directionalLightSection.height + 5;

        if (body.width > 450)
        {
            directionalLightSection.x = 5;
            directionalLightSection.y = environmentSection.height;
            directionalLightSection.width = body.width / 3f - 7.5f;
            directionalLightSection.height = 200;

            pointLightSection.x = body.width/3 + 2.5f;
            pointLightSection.y = environmentSection.height;
            pointLightSection.width = body.width / 3f - 5;
            pointLightSection.height = 200;

            spotLightSection.x = (body.width / 3f) * 2 + 2.5f;
            spotLightSection.y = environmentSection.height;
            spotLightSection.width = body.width / 3f - 7.5f;
            spotLightSection.height = 200;
        }
        else
        {
            directionalLightSection.x = 5;
            directionalLightSection.y = environmentSection.height;
            directionalLightSection.width = body.width - 10;
            directionalLightSection.height = 180;

            pointLightSection.x = 5;
            pointLightSection.y = directionalLightSection.height + environmentSection.height;
            pointLightSection.width = body.width - 10;
            pointLightSection.height = 200;

            spotLightSection.x = 5;
            spotLightSection.y = directionalLightSection.height + pointLightSection.height + environmentSection.height;
            spotLightSection.width = body.width -10;
            spotLightSection.height = 220;
        }
        footerSection.x = 0;
        footerSection.y = directionalLightSection.height + environmentSection.height;
        footerSection.width = body.width;
        footerSection.height = 1000;


        GUI.DrawTexture(headerSectionDivider,lightGreyTexture);
        GUI.DrawTexture(directionalLightSection,lightGreyTexture);
        GUI.DrawTexture(pointLightSection,darkGreyTexture);
        GUI.DrawTexture(spotLightSection,lightGreyTexture);
    }

    void DrawHeader()
    {
        
        var serializedObject = new SerializedObject(lightingData);
        serializedObject.Update();

        var preset = serializedObject.FindProperty("currentLightPreset");
        
        GUI.DrawTexture(headerSection,darkGreyTexture);
            GUILayout.BeginArea(headerSection);
                GUILayout.Space(10);

                EditorGUILayout.BeginHorizontal();

                    GUILayout.Label("Current Preset");
                    EditorGUILayout.PropertyField(preset, GUIContent.none);

                    if (lightingData.currentLightPreset != lastData)
                    {
                        data = Instantiate<LightData>(lightingData.currentLightPreset);
                        AssetDatabase.CreateAsset(data,"Assets/Data/CurrentLightSettings.asset");
                        currentLightSettings = data;
                        changed = true;
                    }
                    if (data != currentLightSettings)
                    {
                        data = currentLightSettings;
                    }
                    lastData = lightingData.currentLightPreset;

                EditorGUILayout.EndHorizontal();
                GUILayout.Space(3);
                if (GUILayout.Button("Reset"))
                {
                    ResetSettings();
                }
                if (GUILayout.Button("Save"))
                {
                    data = Instantiate<LightData>(data);
                    AssetDatabase.CreateAsset(data,"Assets/Data/Presets/" + lightingData.currentLightPreset.name + ".asset");
                    currentLightSettings = Instantiate<LightData>(data);
                    AssetDatabase.CreateAsset(currentLightSettings,"Assets/Data/CurrentLightSettings.asset");
                    AssetDatabase.Refresh();
                }
                if (GUILayout.Button("Save New Preset"))
                {
                    SaveWindow.OpenSaveWindow();
                }

            GUILayout.EndArea();
        serializedObject.ApplyModifiedProperties();
    }

    void ResetSettings()
    {
        currentLightSettings = Instantiate<LightData>(lightingData.currentLightPreset);
        AssetDatabase.CreateAsset(currentLightSettings,"Assets/Data/CurrentLightSettings.asset");
        AssetDatabase.Refresh();
    }

    void DrawEnvironmentSettings(LightData lightData)
    {
        var serializedObject = new SerializedObject(lightData);
        serializedObject.Update();

        var skybox = serializedObject.FindProperty("skyboxMaterial");
        var lightingSource = serializedObject.FindProperty("lightingSource");
        var ambientColour = serializedObject.FindProperty("ambientColour");
        var intensityMultiplier = serializedObject.FindProperty("intensityMultiplier");
        var skyColour = serializedObject.FindProperty("skyColour");
        var equatorColour = serializedObject.FindProperty("equatorColour");
        var groundColour = serializedObject.FindProperty("groundColour");

            GUILayout.BeginArea(environmentSection);
                GUILayout.Space(10);

                EditorGUILayout.BeginHorizontal();
                    GUILayout.Label("Skybox Material");
                    EditorGUILayout.PropertyField(skybox,GUIContent.none);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                    GUILayout.Label("Lighting Source");
                    EditorGUILayout.PropertyField(lightingSource,GUIContent.none);
                EditorGUILayout.EndHorizontal();

                if (data.lightingSource == AmbientMode.Skybox)
                {
                    if (data.skyboxMaterial == null)
                    {
                        EditorGUILayout.BeginHorizontal();
                            GUILayout.Label("Ambient Colour");
                            EditorGUILayout.PropertyField(ambientColour,GUIContent.none);
                        EditorGUILayout.EndHorizontal();
                    }
                    else
                    {
                        EditorGUILayout.BeginHorizontal();
                            GUILayout.Label("Intensity Multiplier");
                            EditorGUILayout.PropertyField(intensityMultiplier,GUIContent.none);
                        EditorGUILayout.EndHorizontal();
                    }
                }

                if (data.lightingSource == AmbientMode.Trilight)
                {
                    GUILayout.Space(10);

                    EditorGUILayout.BeginHorizontal();
                        GUILayout.Label("Sky Colour");
                        EditorGUILayout.PropertyField(skyColour,GUIContent.none);
                    EditorGUILayout.EndHorizontal();
                    
                    EditorGUILayout.BeginHorizontal();
                        GUILayout.Label("Equator Colour");
                        EditorGUILayout.PropertyField(equatorColour,GUIContent.none);
                    EditorGUILayout.EndHorizontal();

                    EditorGUILayout.BeginHorizontal();
                        GUILayout.Label("Ground Colour");
                        EditorGUILayout.PropertyField(groundColour,GUIContent.none);
                    EditorGUILayout.EndHorizontal();
                }

                if (data.lightingSource == AmbientMode.Flat)
                {
                    EditorGUILayout.BeginHorizontal();
                        GUILayout.Label("Ambient Colour");
                        EditorGUILayout.PropertyField(ambientColour,GUIContent.none);
                    EditorGUILayout.EndHorizontal();
                }

                GUILayout.Space(10);

            GUILayout.EndArea();
        serializedObject.ApplyModifiedProperties();
    }

    void ApplyEnvironmentSettings()
    {
        RenderSettings.skybox = data.skyboxMaterial;
        RenderSettings.ambientMode = data.lightingSource;
        RenderSettings.ambientIntensity = data.intensityMultiplier;
        
        if (data.skyboxMaterial == null)
        { 
            if (data.lightingSource == AmbientMode.Trilight)
                RenderSettings.ambientSkyColor = data.skyColour;
            else
                RenderSettings.ambientSkyColor = data.ambientColour;
        }
        else
        {
            RenderSettings.ambientSkyColor = data.ambientColour;
        }

        RenderSettings.ambientEquatorColor = data.equatorColour;
        RenderSettings.ambientGroundColor = data.groundColour; 
    }

    void GetEnvironmentSettings()
    {
        data.skyboxMaterial = RenderSettings.skybox;
        data.lightingSource = RenderSettings.ambientMode;
        data.intensityMultiplier = RenderSettings.ambientIntensity;
    
        if (data.skyboxMaterial == null)
        {
            if (data.lightingSource == AmbientMode.Trilight)
                data.skyColour = RenderSettings.ambientSkyColor;
            else
                data.ambientColour = RenderSettings.ambientSkyColor;
        }
        else
        {
            data.ambientColour = RenderSettings.ambientSkyColor;
        }

        data.equatorColour = RenderSettings.ambientEquatorColor;
        data.groundColour = RenderSettings.ambientGroundColor; 
    
    }

    void ApplyDirectionalLightSettings()
    {
        if (directionalLight != null)
        {
            directionalLight.transform.localRotation = Quaternion.Euler(data.directionalLightRotation);
            
            directionalLight.color = data.directionalLightColor;
            directionalLight.intensity = data.directionalLightIntensity;
            directionalLight.shadowStrength = data.directionalLightShadowStrength;
        }
    }

    void GetDirectionalLightSettings()
    {
        if (directionalLight != null)
        {
            data.directionalLightRotation = directionalLight.transform.localRotation.eulerAngles;

            data.directionalLightColor = directionalLight.color;
            data.directionalLightIntensity = directionalLight.intensity;
            data.directionalLightShadowStrength = directionalLight.shadowStrength;
        }
    }

    void ApplyPointLightSettings()
    {
        if (pointLight != null)
        {
            pointLight.transform.position = data.pointLightPosition;

            pointLight.color = data.pointLightColor;
            pointLight.intensity = data.pointLightIntensity;
            pointLight.shadowStrength = data.pointLightShadowStrength;
            pointLight.range = data.pointLightRange;
        }
    }

    void GetPointLightSettings()
    {
        if (pointLight != null)
        {
            data.pointLightPosition = pointLight.transform.position;

            data.pointLightColor = pointLight.color;
            data.pointLightIntensity = pointLight.intensity;
            data.pointLightShadowStrength = pointLight.shadowStrength;
            data.pointLightRange = pointLight.range;
        }
    }

    void ApplySpotLightSettings()
    {
        if (spotLight != null)
        {
            spotLight.transform.position = data.spotLightPosition;
            spotLight.transform.localRotation = Quaternion.Euler(data.spotLightRotation);

            spotLight.color = data.spotLightColor;
            spotLight.intensity = data.spotLightIntensity;
            spotLight.shadowStrength = data.spotLightShadowStrength;
            spotLight.range = data.spotLightRange;
            spotLight.spotAngle = data.spotLightAngle;
        }
    }

    void GetSpotLightSettings()
    {
        if (spotLight != null)
        {
            data.spotLightPosition = spotLight.transform.position;
            data.spotLightRotation = spotLight.transform.localRotation.eulerAngles;

            data.spotLightColor = spotLight.color;
            data.spotLightIntensity = spotLight.intensity;
            data.spotLightShadowStrength = spotLight.shadowStrength;
            data.spotLightRange = spotLight.range;
            data.spotLightAngle = spotLight.spotAngle;
        }
    }

    public void DrawDirectionalLightSettings(LightData lightData)
    {
        var serializedObject = new SerializedObject(lightData);
        serializedObject.Update();

        var rot = serializedObject.FindProperty("directionalLightRotation");

        var color = serializedObject.FindProperty("directionalLightColor");
        var intensity = serializedObject.FindProperty("directionalLightIntensity");
        var shadowStrength = serializedObject.FindProperty("directionalLightShadowStrength");

        GUILayout.BeginArea(directionalLightSection);

        

        GUILayout.Label("Directional Light");

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Light");
            EditorGUILayout.ObjectField(directionalLight, typeof(Light), true);
        EditorGUILayout.EndHorizontal();
        
        GUILayout.Space(10);

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Rotation");
            EditorGUILayout.PropertyField(rot, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        GUILayout.Space(32);

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Color");
            EditorGUILayout.PropertyField(color, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PropertyField(intensity, new GUIContent("Intensity"));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Shadow Strength");
            EditorGUILayout.PropertyField(shadowStrength, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        GUILayout.EndArea();

        serializedObject.ApplyModifiedProperties();
    }
    
    public void DrawPointLightSettings(LightData lightData)
    {
        var serializedObject = new SerializedObject(lightData);
        serializedObject.Update();

        var pos = serializedObject.FindProperty("pointLightPosition");

        var color = serializedObject.FindProperty("pointLightColor");
        var intensity = serializedObject.FindProperty("pointLightIntensity");
        var shadowStrength = serializedObject.FindProperty("pointLightShadowStrength");
        var range = serializedObject.FindProperty("pointLightRange");

        GUILayout.BeginArea(pointLightSection);
    
        GUILayout.Label("Point Light");

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Light");
            EditorGUILayout.ObjectField(pointLight, typeof(Light), true);
        EditorGUILayout.EndHorizontal();
        
        GUILayout.Space(10);

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Position");
            EditorGUILayout.PropertyField(pos, GUIContent.none);
        EditorGUILayout.EndHorizontal();
        
        GUILayout.Space(32);

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Color");
            EditorGUILayout.PropertyField(color, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PropertyField(intensity, new GUIContent("Intensity"));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Shadow Strength");
            EditorGUILayout.PropertyField(shadowStrength, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PropertyField(range, new GUIContent("Range"));
        EditorGUILayout.EndHorizontal(); 
        
        GUILayout.EndArea();

        serializedObject.ApplyModifiedProperties();

    }

    public void DrawSpotLightSettings(LightData lightData)
    {
        var serializedObject = new SerializedObject(lightData);
        serializedObject.Update();

        var pos = serializedObject.FindProperty("spotLightPosition");
        var rot = serializedObject.FindProperty("spotLightRotation");

        var color = serializedObject.FindProperty("spotLightColor");
        var intensity = serializedObject.FindProperty("spotLightIntensity");
        var shadowStrength = serializedObject.FindProperty("spotLightShadowStrength");
        var range = serializedObject.FindProperty("spotLightRange");
        var spotAngle = serializedObject.FindProperty("spotLightAngle");

        GUILayout.BeginArea(spotLightSection);
    
        GUILayout.Label("Spot Light");

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Light");
            EditorGUILayout.ObjectField(spotLight, typeof(Light), true);
        EditorGUILayout.EndHorizontal();

        GUILayout.Space(10);

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Position");
            EditorGUILayout.PropertyField(pos, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Rotation");
            EditorGUILayout.PropertyField(rot, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        GUILayout.Space(10);

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Color");
            EditorGUILayout.PropertyField(color, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PropertyField(intensity, new GUIContent("Intensity"));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Shadow Strength");
            EditorGUILayout.PropertyField(shadowStrength, GUIContent.none);
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PropertyField(range, new GUIContent("Range"));
        EditorGUILayout.EndHorizontal(); 

        EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PropertyField(spotAngle, new GUIContent("Spot Angle"));
        EditorGUILayout.EndHorizontal(); 

        GUILayout.EndArea();

        serializedObject.ApplyModifiedProperties();
    }
}

public class SaveWindow : EditorWindow
{

    static SaveWindow window;

    static string presetName = "";

    LightData data = LightingControlPanelWindow.data;

    public static void OpenSaveWindow()
    {
        window = (SaveWindow)GetWindow(typeof(SaveWindow));
        window.minSize = new Vector2(400,50);
        window.maxSize = new Vector2(400,50);
        window.Show();
    }

    void OnGUI()
    {
        GUILayout.Space(10);
        EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Preset Name");
            presetName = EditorGUILayout.TextField("", presetName);
        EditorGUILayout.EndHorizontal();

        if (presetName != "")
        {
            EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Save New Preset"))
                {
                    data = LightingControlPanelWindow.currentLightSettings;
                    data = Instantiate<LightData>(data);
                    AssetDatabase.CreateAsset(data,"Assets/Data/Presets/" + presetName + ".asset");
                    LightingControlPanelWindow.lightingData.currentLightPreset = data;
                    data = Instantiate<LightData>(data);
                    AssetDatabase.CreateAsset(data,"Assets/Data/CurrentLightSettings.asset");
                    LightingControlPanelWindow.currentLightSettings = data;

                    AssetDatabase.Refresh();
                }
            EditorGUILayout.EndHorizontal();
        }
    
    }
}
