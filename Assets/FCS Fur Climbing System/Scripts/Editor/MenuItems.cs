// (c) Copyright VinforLab Team. All rights reserved.

using UnityEditor;
using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class MenuItems : MonoBehaviour
    {
        public const string EDITOR_PATH = "Component/Fur Climbing System/";

        static TriangleProvider lastTriangleProvider;

        [MenuItem(EDITOR_PATH + "Configure Player")]
        static void ConfigurePlayer()
        {
            string[] guids = AssetDatabase.FindAssets( "t:Prefab" );
            GameObject fcs = null;

            foreach (var guid in guids)
            {
                var path = AssetDatabase.GUIDToAssetPath(guid);
                GameObject go = AssetDatabase.LoadAssetAtPath<GameObject>(path);
                if(go.name == "Fur Climbing System")
                {
                    fcs = go;
                }
            }

            GameObject[] objs = Selection.gameObjects;
            foreach (GameObject obj in objs)
            {
                GameObject fcsInstance = Instantiate(fcs, obj.transform);

                Undo.RegisterCreatedObjectUndo(fcsInstance, "Created FCS Instance");
                Undo.RegisterFullObjectHierarchyUndo(fcsInstance, "FCS Instance");

                SurfaceGrab surfaceGrab = ObjectFactory.AddComponent<SurfaceGrab>(obj);
                surfaceGrab.raycastSettings.raycaster = fcsInstance.transform.GetChild(0).GetChild(0).GetComponent<Raycaster>();
                surfaceGrab.raycastSettings.raycasterDown = fcsInstance.transform.GetChild(0).GetChild(1).GetComponent<Raycaster>();

                SurfaceWalker surfaceWalker = ObjectFactory.AddComponent<SurfaceWalker>(obj);
                surfaceWalker.raycastSettings.raycaster = fcsInstance.transform.GetChild(0).GetChild(1).GetComponent<Raycaster>();

                if(obj.GetComponent<PlayerDemo>() != null)
                {
                    obj.GetComponent<PlayerDemo>().surfaceGrab = surfaceGrab;
                    obj.GetComponent<PlayerDemo>().sRight = fcsInstance.transform.GetChild(0).GetChild(2).GetChild(1).GetComponent<Sensor>();
                    obj.GetComponent<PlayerDemo>().sLeft = fcsInstance.transform.GetChild(0).GetChild(2).GetChild(0).GetComponent<Sensor>();
                    obj.GetComponent<PlayerDemo>().sUp = fcsInstance.transform.GetChild(0).GetChild(2).GetChild(2).GetComponent<Sensor>();
                    obj.GetComponent<PlayerDemo>().sDown = fcsInstance.transform.GetChild(0).GetChild(2).GetChild(3).GetComponent<Sensor>();
                }



                PrefabUtility.RecordPrefabInstancePropertyModifications(obj);
            }

            if (objs.Length > 0) {
                EditorUtility.DisplayDialog("Success", "Player configured successfully, now you can configure SurfaceGrab and SurfaceWalker properties.", "Okay");
            }
        }

        static void MarkObjectAsReadWrite(Object obj)
        {
            if (obj == null)
            {
                Debug.Log("Null mesh object detected.");
                return;
            }

            string path = AssetDatabase.GetAssetPath(obj);
            if(path == "Library/unity default resources")
            {
                Debug.Log("Builtin Unity object detected.");
                return;
            }
            ModelImporter importer = ModelImporter.GetAtPath(path) as ModelImporter;
            if (!importer.isReadable)
            {
                importer.isReadable = true;
                Debug.Log("Marking " + path + " as Read/Write...");
                importer.SaveAndReimport();
            }
            else
            {
                Debug.Log(path + " already is Read/Write");

            }
        }


        [MenuItem(EDITOR_PATH + "Configure Climbable Mesh")]
        static void AddComponents()
        {
            if (EditorUtility.DisplayDialog("Continue?", "The mesh will have the \"Read/Write\" setting changed to True. To reverse this, select the model file and uncheck the option \"Read/Write\".", "Okay", "Cancel"))
            {
                //bool isTutorialScene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name == "Tutorial";

                GameObject[] objs = Selection.gameObjects;

                foreach (GameObject obj in objs)
                {
                    bool isDemoTentacle = obj.name == "Tentacle_Climb_Demo_a91mfpqei1";

                    if(!isDemoTentacle)
                        DestroyImmediate(obj.GetComponent<Collider>());

                    TriangleProvider triangleProvider = obj.GetComponent<TriangleProvider>();

                    if (triangleProvider == null)
                    {
                        triangleProvider = ObjectFactory.AddComponent<TriangleProvider>(obj);
                    }

                    lastTriangleProvider = triangleProvider;

                    if (obj.GetComponent<DynamicCollider>() == null)
                    {
                        ObjectFactory.AddComponent<DynamicCollider>(obj);
                    }

                    Mesh sharedMesh = null;


                    if (obj.GetComponent<SkinnedMeshRenderer>() != null)
                    {
                        sharedMesh = obj.GetComponent<SkinnedMeshRenderer>().sharedMesh;
                    }
                    else if (obj.GetComponent<MeshFilter>() != null)
                    {
                        sharedMesh = obj.GetComponent<MeshFilter>().sharedMesh;
                    }

                    MarkObjectAsReadWrite(sharedMesh);

                    if (obj.GetComponent<MeshCollider>() == null)
                    {
                        MeshCollider meshCollider = ObjectFactory.AddComponent<MeshCollider>(obj);
                        meshCollider.sharedMesh = sharedMesh;
                    }

                    triangleProvider.targetMeshCollider = obj.GetComponent<MeshCollider>();

                    try
                    {
                        obj.layer = LayerMask.NameToLayer("Climbing");
                    }
                    catch (System.Exception e)
                    {
                        obj.layer = 0;
                    }

                    PrefabUtility.RecordPrefabInstancePropertyModifications(obj);
                }
            }
        }
    }
}