using UnityEngine;

public class PaintVertices : MonoBehaviour
{
    public float radius = 1.0f;
    public float pull = 10.0f;
    private MeshFilter unappliedMesh;
    public FallOff fallOff = FallOff.Gauss;

    void Start()
    {
    }

    void Update()
    {
        // When no button is pressed we update the mesh collider
        if (!Input.GetMouseButton(0))
        {
            // Apply collision mesh when we let go of button
            ApplyMeshCollider();
            return;
        }


        // Did we hit the surface?
        RaycastHit hit;
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        if (Physics.Raycast(ray, out hit))
        {
            MeshFilter filter = hit.collider.GetComponent<MeshFilter>();
            if (filter)
            {
                // Don't update mesh collider every frame since physX
                // does some heavy processing to optimize the collision mesh.
                // So this is not fast enough for real time updating every frame
                if (filter != unappliedMesh)
                {
                    ApplyMeshCollider();
                    unappliedMesh = filter;
                }

                // Deform mesh
                Vector3 relativePoint = filter.transform.InverseTransformPoint(hit.point);
                DeformMesh(filter.mesh, relativePoint, pull * Time.deltaTime, radius);
            }
        }
    }

    private void ApplyMeshCollider()
    {
        if (unappliedMesh && unappliedMesh.GetComponent<MeshCollider>())
        {
            unappliedMesh.GetComponent<MeshCollider>().sharedMesh = unappliedMesh.sharedMesh;
        }

        unappliedMesh = null;
    }

    private float NeedleFalloff(float dist, float inRadius)
    {
        return -(dist * dist) / (inRadius * inRadius) + 1.0f;
    }

    private void DeformMesh(Mesh mesh, Vector3 position, float power, float inRadius)
    {
        Vector3[] vertices = mesh.vertices;
        Vector3[] normals = mesh.normals;
        float sqrRadius = inRadius * inRadius;

        // Calculate averaged normal of all surrounding vertices	
        Vector3 averageNormal = Vector3.zero;
        for (int i = 0; i < vertices.Length; i++)
        {
            float sqrMagnitude = (vertices[i] - position).sqrMagnitude;
            // Early out if too far away
            if (sqrMagnitude > sqrRadius)
            {
                continue;
            }

            float distance = Mathf.Sqrt(sqrMagnitude);
            float falloff = LinearFalloff(distance, inRadius);
            averageNormal += falloff * normals[i];
        }

        averageNormal = averageNormal.normalized;

        // Deform vertices along averaged normal
        for (int i = 0; i < vertices.Length; i++)
        {
            float sqrMagnitude = (vertices[i] - position).sqrMagnitude;
            // Early out if too far away
            if (sqrMagnitude > sqrRadius)
            {
                continue;
            }

            float distance = Mathf.Sqrt(sqrMagnitude);
            float falloff;
            switch (fallOff)
            {
                case FallOff.Gauss:
                    falloff = GaussFalloff(distance, inRadius);
                    break;
                case FallOff.Needle:
                    falloff = NeedleFalloff(distance, inRadius);
                    break;
                default:
                    falloff = LinearFalloff(distance, inRadius);
                    break;
            }

            vertices[i] += averageNormal * falloff * power;
        }

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    private static float LinearFalloff(float distance, float inRadius)
    {
        return Mathf.Clamp01(1.0f - distance / inRadius);
    }

    private static float GaussFalloff(float distance, float inRadius)
    {
        return Mathf.Clamp01(Mathf.Pow(360.0f, -Mathf.Pow(distance / inRadius, 2.5f) - 0.01f));
    }
}

public enum FallOff
{
    Gauss,
    Linear,
    Needle
}