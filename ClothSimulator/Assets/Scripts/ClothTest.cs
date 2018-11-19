using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClothTest : MonoBehaviour {
    private Mesh mesh;
    private Vector3[] baseVertices;
    public float speed;
    public int index;

    // Use this for initialization
    void Start () {
        mesh = GetComponent<MeshFilter>().mesh;
        mesh.MarkDynamic();
        baseVertices = mesh.vertices;
    }
	
	// Update is called once per frame
	void Update () {
        Vector3[] vertices = new Vector3[baseVertices.Length];

        for (int i = 0; i < vertices.Length; i++) {
            Vector3 vertex = baseVertices[i];

            if (i == index) {
                vertex.y += 3;
            }

            vertices[i] = vertex;
        }
        
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
}
