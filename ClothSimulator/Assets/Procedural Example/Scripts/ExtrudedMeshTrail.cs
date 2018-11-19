using System.Collections.Generic;
using UnityEngine;

// Generates an extrusion trail from the attached mesh
// Uses the MeshExtrusion algorithm in MeshExtrusion.cs to generate and preprocess the mesh.
[RequireComponent(typeof(MeshFilter))]
public class ExtrudedMeshTrail : MonoBehaviour
{
    public float time = 2.0f;
    public bool autoCalculateOrientation = true;
    public float minDistance = 0.1f;
    public bool invertFaces = false;

    private Mesh srcMesh;
    private MeshExtrusion.Edge[] precomputedEdges;
    private List<ExtrudedTrailSection> sections = new List<ExtrudedTrailSection>();

    void Start()
    {
        srcMesh = GetComponent<MeshFilter>().sharedMesh;
        precomputedEdges = MeshExtrusion.BuildManifoldEdges(srcMesh);
    }

    void LateUpdate()
    {
        Vector3 position = transform.position;
        float now = Time.time;

        while (sections.Count > 0 && now > sections[sections.Count - 1].time + time)
        {
            sections.RemoveAt(sections.Count - 1);
        }

        // Add a new trail section to beginning of array
        if (sections.Count == 0 || (sections[0].point - position).sqrMagnitude > minDistance * minDistance)
        {
            ExtrudedTrailSection section = new ExtrudedTrailSection();
            section.point = position;
            section.matrix = transform.localToWorldMatrix;
            section.time = now;
            sections.Insert(0, section);
        }

        // We need at least 2 sections to create the line
        if (sections.Count < 2)
        {
            return;
        }

        Matrix4x4 worldToLocal = transform.worldToLocalMatrix;
        Matrix4x4[] finalSections = new Matrix4x4[sections.Count];
        Quaternion previousRotation = new Quaternion();

        for (int i = 0; i < sections.Count; i++)
        {
            if (autoCalculateOrientation)
            {
                if (i == 0)
                {
                    Vector3 direction = sections[0].point - sections[1].point;
                    Quaternion rotation = Quaternion.LookRotation(direction, Vector3.up);
                    previousRotation = rotation;
                    finalSections[i] = worldToLocal * Matrix4x4.TRS(position, rotation, Vector3.one);
                }
                // all elements get the direction by looking up the next section
                else if (i != sections.Count - 1)
                {
                    Vector3 direction = sections[i].point - sections[i + 1].point;
                    Quaternion rotation = Quaternion.LookRotation(direction, Vector3.up);

                    // When the angle of the rotation compared to the last segment is too high
                    // smooth the rotation a little bit. Optimally we would smooth the entire sections array.
                    if (Quaternion.Angle(previousRotation, rotation) > 20)
                    {
                        rotation = Quaternion.Slerp(previousRotation, rotation, 0.5f);
                    }

                    previousRotation = rotation;
                    finalSections[i] = worldToLocal * Matrix4x4.TRS(sections[i].point, rotation, Vector3.one);
                }
                // except the last one, which just copies the previous one
                else
                {
                    finalSections[i] = finalSections[i - 1];
                }
            }
            else
            {
                if (i == 0)
                {
                    finalSections[i] = Matrix4x4.identity;
                }
                else
                {
                    finalSections[i] = worldToLocal * sections[i].matrix;
                }
            }
        }
        MeshExtrusion.ExtrudeMesh (srcMesh, GetComponent<MeshFilter>().mesh, finalSections, precomputedEdges, invertFaces);
    }
    
    class ExtrudedTrailSection
    {
        public Vector3 point;
        public Matrix4x4 matrix;
        public float time;
    }
}