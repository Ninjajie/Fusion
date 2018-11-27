using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 Generates a trail that is always facing upwards using the scriptable mesh interface.
 vertex colors and uv's are generated similar to the builtin Trail Renderer.
 To use it
 1. create an empty game object
 2. attach this script and a MeshRenderer
 3. Then assign a particle material to the mesh renderer
*/
[RequireComponent(typeof(MeshFilter))]
public class TronTrail : MonoBehaviour
{
    public float height = 2.0f;
    public float time = 2.0f;
    public bool alwaysUp = false;
    public float minDistance = 0.1f;

    public Color startColor = Color.white;
    public Color endColor = new Color(1, 1, 1, 0);

    private List<TronTrailSection> sections = new List<TronTrailSection>();

    private void LateUpdate()
    {
        Vector3 position = transform.position;
        float now = Time.time;

        // Remove old sections
        while (sections.Count > 0 && now > sections[sections.Count - 1].time + time)
        {
            sections.RemoveAt(sections.Count - 1);
        }

        // Add a new trail section
        if (sections.Count == 0 || (sections[0].point - position).sqrMagnitude > minDistance * minDistance)
        {
            TronTrailSection section = new TronTrailSection();
            section.point = position;
            if (alwaysUp)
            {
                section.upDir = Vector3.up;
            }
            else
            {
                section.upDir = transform.TransformDirection(Vector3.up);
            }

            section.time = now;

            sections.Insert(0, section);
        }

        // Rebuild the mesh
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.Clear();

        // We need at least 2 sections to create the line
        if (sections.Count < 2)
        {
            return;
        }

        Vector3[] vertices = new Vector3[sections.Count * 2];
        Color[] colors = new Color[sections.Count * 2];
        Vector2[] uv = new Vector2[sections.Count * 2];

        TronTrailSection previousSection = sections[0];
        TronTrailSection currentSection = sections[0];

        // Use matrix instead of transform.TransformPoint for performance reasons
        Matrix4x4 localSpaceTransform = transform.worldToLocalMatrix;

        // Generate vertex, uv and colors
        for (int i = 0; i < sections.Count; i++)
        {
            previousSection = currentSection;
            currentSection = sections[i];
            // Calculate u for texture uv and color interpolation
            float u = 0.0f;
            if (i != 0)
            {
                u = Mathf.Clamp01((Time.time - currentSection.time) / time);
            }

            // Calculate upwards direction
            Vector3 upDir = currentSection.upDir;

            // Generate vertices
            vertices[i * 2 + 0] = localSpaceTransform.MultiplyPoint(currentSection.point);
            vertices[i * 2 + 1] = localSpaceTransform.MultiplyPoint(currentSection.point + upDir * height);

            uv[i * 2 + 0] = new Vector2(u, 0);
            uv[i * 2 + 1] = new Vector2(u, 1);

            // fade colors out over time
            Color interpolatedColor = Color.Lerp(startColor, endColor, u);
            colors[i * 2 + 0] = interpolatedColor;
            colors[i * 2 + 1] = interpolatedColor;
        }

        // Generate triangles indices
        int[] triangles = new int[(sections.Count - 1) * 2 * 3];
        for (int i = 0; i < triangles.Length / 6; i++)
        {
            triangles[i * 6 + 0] = i * 2;
            triangles[i * 6 + 1] = i * 2 + 1;
            triangles[i * 6 + 2] = i * 2 + 2;

            triangles[i * 6 + 3] = i * 2 + 2;
            triangles[i * 6 + 4] = i * 2 + 1;
            triangles[i * 6 + 5] = i * 2 + 3;
        }

        // Assign to mesh	
        mesh.vertices = vertices;
        mesh.colors = colors;
        mesh.uv = uv;
        mesh.triangles = triangles;
    }
}

class TronTrailSection
{
    public Vector3 point;
    public Vector3 upDir;
    public float time;
}