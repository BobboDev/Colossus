using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraControl : MonoBehaviour
{
    public Transform target;
    public float speed = 10f;

    Vector2 movement;

    // Update is called once per frame
    void Update()
    {
        movement.x = 0;
        movement.y = 0;

        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            movement.x += Time.deltaTime * speed;

        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            movement.x -= Time.deltaTime * speed;

        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            movement.y += Time.deltaTime * speed;

        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            movement.y -= Time.deltaTime * speed;

        transform.Rotate(target.up, movement.x, Space.World);
        transform.Rotate(Vector3.Cross(transform.forward, target.up), movement.y, Space.World);
    }
}
