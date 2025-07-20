using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CrossTest : MonoBehaviour
{
    public Transform V1;
    public Transform V2;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        // Debug.Log(Vector3.Dot(V1.forward, V2.forward * 2));
        Debug.DrawLine(V1.forward, transform.position, Color.red);
        Debug.DrawLine(V2.forward, transform.position, Color.blue);
    }
}
