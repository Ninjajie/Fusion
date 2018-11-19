using UnityEngine;

// This script is placed in public domain. The author takes no responsibility for any possible harm.
public class CrumpleMesh : MonoBehaviour
{
    public float scale = 1.0f;
    public float speed = 1.0f;
    public bool recalculateNormals = false;

    private Perlin noise;
    private Vector3[] baseVertices;

    private void Start()
    {
        noise = new Perlin();
    }

    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;

        if (baseVertices == null)
        {
            baseVertices = mesh.vertices;
        }
        
        Vector3[] vertices = new Vector3[baseVertices.Length];
        
        float timeX = Time.time * speed + 0.1365143f;
        float timeY = Time.time * speed + 1.21688f;
        float timeZ = Time.time * speed + 2.5564f;

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 vertex = baseVertices[i];
            vertex.x += noise.Noise(timeX + vertex.x, timeX + vertex.y, timeX + vertex.z) * scale;
            vertex.y += noise.Noise(timeY + vertex.x, timeY + vertex.y, timeY + vertex.z) * scale;
            vertex.z += noise.Noise(timeZ + vertex.x, timeZ + vertex.y, timeZ + vertex.z) * scale;

            vertices[i] = vertex;
        }

        mesh.vertices = vertices;

        if (recalculateNormals)
        {
            mesh.RecalculateNormals();
        }
        mesh.RecalculateBounds();
    }
}