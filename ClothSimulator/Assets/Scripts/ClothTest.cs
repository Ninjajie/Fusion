using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClothTest : MonoBehaviour {
    public GameObject target;

    // Use this for initialization
    void Start () {

    }
	
	// Update is called once per frame
	void Update () {
        if (Input.GetKeyDown(KeyCode.Space)) {
            target.transform.position += Vector3.up * 5;
        }
    }
}
