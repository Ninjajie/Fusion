using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingSphere : MonoBehaviour {

    public Transform sphereTransform;

    Vector3 left = new Vector3(-0.1f, 0.0f, 0.0f);
    Vector3 right = new Vector3(0.1f, 0.0f, 0.0f);
    Vector3 up = new Vector3(0.0f, 0.1f, 0.0f);
    Vector3 down = new Vector3(0.0f, -0.1f, 0.0f);
    // Use this for initialization
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

        if (Input.GetKey("i"))
        {
            sphereTransform.Translate(up);

        }
        if (Input.GetKey("k"))
        {
            sphereTransform.Translate(down);

        }
        if (Input.GetKey("j"))
        {      
            sphereTransform.Translate(left);
        }
        if (Input.GetKey("l"))
        {
            sphereTransform.Translate(right);
        }
    }
}
