using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TEST : MonoBehaviour
{
    public Transform a;
    public Transform b;

    public Transform cube;

    public float speed = 2;

    bool reverse;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void LateUpdate()
    {

        if (reverse)
        {
            if (cube.position.z > b.position.z)
            {
                reverse = false;
            }
        }
        if (!reverse)
        {
            if (cube.position.z < a.position.z)
            {
                reverse = true;
            }
        }

        if (reverse)
        {
            cube.position += Vector3.forward * speed * Time.deltaTime;
        }
        else
        {
            cube.position -= Vector3.forward * speed * Time.deltaTime;
        }
    }
}
