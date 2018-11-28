using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingPlane : MonoBehaviour {
    public Transform PlaneTransform;
	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {

        if (Input.GetKey("w"))
        {
            Vector3 up = new Vector3(0.0f, 0.1f, 0.0f);
            PlaneTransform.Translate(up);

        }
        if (Input.GetKey("s"))
        {
            Vector3 down = new Vector3(0.0f, -0.1f, 0.0f);
            PlaneTransform.Translate(down);

        }
    }
}
