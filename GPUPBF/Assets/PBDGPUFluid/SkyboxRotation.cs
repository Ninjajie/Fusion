using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SkyboxRotation : MonoBehaviour {

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        //get the skybox component, rotate its material(texture) at a constant speed
        GetComponent<Skybox>().material.SetFloat("_Rotation", Time.time * 5.1f);
    }
}
