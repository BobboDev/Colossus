using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    #region Singleton

    public static GameManager instance;

    void Awake()
    {
        instance = this;
    }

    #endregion

    public MeshManager meshManager;
    public float deltaTime;
    public float fixedDeltaTime;

    void Update()
    {
        deltaTime = Time.deltaTime;
        if (Input.GetKeyDown(KeyCode.T))
        {
            if (Time.timeScale == 1)
            {
                Time.timeScale = 0.1f;
            }
            else
            {
                Time.timeScale = 1;
            }
        }
    }

    void FixedUpdate()
    {
        fixedDeltaTime = Time.fixedDeltaTime;
    }
}
