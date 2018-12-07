using System.Linq;
using System.Collections.Generic;
using UnityEngine;

public class ClothSimulator : MonoBehaviour {
    // simulation variables
    public float timestep = 0.02f;
    private float nextFrameTime = 0f;
    public int iterationNum = 5;
    public float vertexMass = 2;
    public Vector3 gravity;
    public int rows = 10;
    public int columns = 10;
    public bool useSmartDamping;

    // constraint weights
    public float distanceCompressionStiffness = 0.8f;
    public float distanceStretchStiffness = 0.8f;
    public float bendingStiffness = 0.1f;
    public DampingMethod dampingMethod;
    public float dampingStiffness = 0.02f;

    // constraint data
    public bool usePointConstraint;
    public float groundPlane;
    public bool useGroundConstraint;
    public bool useDistanceConstraint;
    public BendingMethod bendingMethod;

    // simulation data
    private Vector3[] positions; 
    private Vector3[] projectedPositions;
    private Vector3[] velocities;
    private float[] frictions;
    private Triangle[] triangles;
    private float invMass;
    private List<Constraint> constraints = new List<Constraint>();
    private List<Constraint> collisionConstraints = new List<Constraint>();
    private List<PointConstraint> pointConstraints = new List<PointConstraint>();
    private GroundConstraint ground = null;
    private int numParticles;

    // unity data
    Mesh mesh;
    Mesh reverseMesh;

    // collision stuff TODO: hacky solution
    public LayerMask collisionLayers;

    // Use this for initialization
    void Start () {
        // create new mesh
        mesh = Utility.CreateClothMesh(rows, columns);
        transform.GetChild(0).GetComponent<MeshFilter>().mesh = mesh;

        numParticles = mesh.vertexCount;
        Vector3[] baseVertices = mesh.vertices;

        positions = new Vector3[numParticles];
        projectedPositions = new Vector3[numParticles];
        velocities = new Vector3[numParticles];
        frictions = new float[numParticles];
        //vertexMass = extent.x * extent.y * rows * columns / 10000f;

        // make new mesh for the opposite side
        GameObject newCloth = Instantiate(transform.GetChild(0).gameObject, transform);
        newCloth.name = "Back";
        reverseMesh = transform.GetChild(1).GetComponent<MeshFilter>().mesh;
        for (int m = 0; m < reverseMesh.subMeshCount; m++) {
            int[] triangles = reverseMesh.GetTriangles(m);
            for (int i = 0; i < triangles.Length; i += 3) {
                int temp = triangles[i + 0];
                triangles[i + 0] = triangles[i + 1];
                triangles[i + 1] = temp;
            }
            reverseMesh.SetTriangles(triangles, m);
        }

        // step 1-3: initialize position, velocity and weight
        for (int i = 0; i < numParticles; i++) {
            //positions[i] = transform.TransformPoint(baseVertices[i]); 
            positions[i] = baseVertices[i];
            projectedPositions[i] = positions[i];
            velocities[i] = Vector3.zero;
            frictions[i] = 1;
        }
        invMass = 1.0f / vertexMass;

        // initialize triangles
        int[] triangleIndices = mesh.GetTriangles(0);
        triangles = new Triangle[triangleIndices.Length / 3];
        for (int i = 0; i < triangles.Length; i++) {
            triangles[i] = new Triangle(triangleIndices[i * 3], triangleIndices[i * 3 + 1], triangleIndices[i * 3 + 2]);
        }

        // add constraints
        if (usePointConstraint) {
            // top left and right
            pointConstraints.Add(new PointConstraint(rows * (columns + 1)));
            pointConstraints.Add(new PointConstraint((rows + 1) * (columns + 1) - 1));
        }
        if (useDistanceConstraint) {
            // use a set to get unique edges
            HashSet<Edge> edgeSet = new HashSet<Edge>(new EdgeComparer());
            for (int i = 0; i < triangles.Length; i++) {
                edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[1]));
                edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[2]));
                edgeSet.Add(new Edge(triangles[i].vertices[1], triangles[i].vertices[2]));
            };

            foreach(Edge e in edgeSet) {
                constraints.Add(new DistanceConstraint(e, positions, distanceCompressionStiffness, distanceStretchStiffness));
            }
        }
        if (bendingMethod != BendingMethod.noBending) {
            Dictionary<Edge, List<Triangle>> wingEdges = new Dictionary<Edge, List<Triangle>>(new EdgeComparer());

            // map edges to all of the faces to which they are connected
            // TODO: check later if we're double adding things here. maybe we should sort vertices afterall
            foreach (Triangle tri in triangles) {
                Edge e1 = new Edge(tri.vertices[0], tri.vertices[1]);
                if (wingEdges.ContainsKey(e1) && !wingEdges[e1].Contains(tri)) {
                    wingEdges[e1].Add(tri);
                }
                else {
                    List<Triangle> tris = new List<Triangle>();
                    tris.Add(tri);
                    wingEdges.Add(e1, tris);
                }

                Edge e2 = new Edge(tri.vertices[0], tri.vertices[2]);
                if (wingEdges.ContainsKey(e2) && !wingEdges[e2].Contains(tri)) {
                    wingEdges[e2].Add(tri);
                }
                else {
                    List<Triangle> tris = new List<Triangle>();
                    tris.Add(tri);
                    wingEdges.Add(e2, tris);
                }

                Edge e3 = new Edge(tri.vertices[1], tri.vertices[2]);
                if (wingEdges.ContainsKey(e3) && !wingEdges[e3].Contains(tri)) {
                    wingEdges[e3].Add(tri);
                }
                else {
                    List<Triangle> tris = new List<Triangle>();
                    tris.Add(tri);
                    wingEdges.Add(e3, tris);
                }
            }

            // wingEdges are edges with 2 occurences,
            // so we need to remove the lower frequency ones
            List<Edge> keyList = wingEdges.Keys.ToList();
            foreach (Edge e in keyList) {
                if (wingEdges[e].Count < 2) {
                    wingEdges.Remove(e);
                }
            }

            foreach (Edge wingEdge in wingEdges.Keys){
                /* wingEdges are indexed like in the Bridson,
                 * Simulation of Clothing with Folds and Wrinkles paper
                 *    3
                 *    ^
                 * 0  |  1
                 *    2
                 */
                
                int[] indices = new int[4];
                indices[2] = wingEdge.startIndex;
                indices[3] = wingEdge.endIndex;

                // TODO: this code could be prettier
                int b = 0;
                foreach (Triangle tri in wingEdges[wingEdge]) {
                    for (int i = 0; i < 3; i++) {
                        int point = tri.vertices[i];
                        if (point != indices[2] && point != indices[3]) {
                            //tri #1
                            if (b == 0) {
                                indices[0] = point;
                                break;
                            }
                            //tri #2
                            else if (b == 1) {
                                indices[1] = point;
                                break;
                            }
                        }
                    }
                    b++;
                }

                if (bendingMethod == BendingMethod.isometricBending) {
                    constraints.Add(new IsometricBendingConstraint(indices, positions, bendingStiffness));

                }
                else {
                    constraints.Add(new BendingConstraint(indices, positions, bendingStiffness));
                }
            }
        }
        if (useGroundConstraint) {
            ground = new GroundConstraint(groundPlane);
        }

        mesh.MarkDynamic();
        reverseMesh.MarkDynamic();
    }


    void Update () {
        // modify data to world coordinates
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.TransformPoint(positions[i]);
            projectedPositions[i] = transform.TransformPoint(projectedPositions[i]);
            velocities[i] = transform.TransformVector(velocities[i]);
        }

        // calculate the timestep 
        nextFrameTime += Time.deltaTime;
        int iter = 0;
        while (nextFrameTime > 0) {
            if (nextFrameTime < timestep) {
                break;
            }

            float dt = Mathf.Min(nextFrameTime, timestep);
            nextFrameTime -= dt;
            iter++;

            // step 5: apply external forces
            ApplyExternalForce(gravity, dt);

            // step 6: damp velocity
            if (dampingMethod != DampingMethod.noDamping) {
                DampVelocity(dampingStiffness, dampingMethod);
            }

            // step 7: apply explicit Euler to positions based on velocity
            ApplyExplicitEuler(dt);

            // step 8: TODO: clear current collisions and generate new collisions
            ClearCollisionConstraints();
            GenerateCollisionConstraints();

            // step 9-11: project constraints iterationNum times
            for (int j = 0; j < iterationNum; j++) {
                // satisfy all constraints
                SatisfyConstraints();
            }

            // satisfy pointConstraints
            SatisfyPointConstraints(dt);

            // step 13 & 14: apply projected positions to actual vertices
            UpdateVertices(dt);

            // step 16: update all velocities using friction
            ApplyFriction();
        }

        // recalculate the center of the mesh
        Vector3 newCenter = GetComponentInChildren<Renderer>().bounds.center;
        Vector3 delta = newCenter - transform.position;

        // modify data to back to local coordinates
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.InverseTransformPoint(positions[i] - delta);
            projectedPositions[i] = transform.InverseTransformPoint(projectedPositions[i] - delta);
            velocities[i] = transform.InverseTransformVector(velocities[i] - delta);
        }
        transform.position = newCenter;

        // update everything into Unity
        mesh.vertices = positions;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        reverseMesh.vertices = positions;
        Vector3[] reverseNormals = mesh.normals;
        for (int i = 0; i < reverseNormals.Length; i++) {
            reverseNormals[i] *= -1;
        }
        reverseMesh.normals = reverseNormals;

        transform.GetChild(0).GetComponent<MeshCollider>().sharedMesh = mesh;
        transform.GetChild(1).GetComponent<MeshCollider>().sharedMesh = reverseMesh;

        print(Time.deltaTime + ", " + iter);
    }

    public void ApplyExternalForce(Vector3 gravity, float dt) {
        for (int i = 0; i < numParticles; i++) {
            velocities[i] += gravity * invMass * dt;
        }
    }

    public void DampVelocity(float dampingStiffness, DampingMethod method) {
        // TODO: there's a smarter way of doing this
        if (method == DampingMethod.simpleDamping) { 
            for (int i = 0; i < numParticles; i++) {
                velocities[i] *= 0.998f;
            }
        }
        else {
            // first compute the center of mass's position and velocity
            Vector3 centerMassPosition = Vector3.zero;
            Vector3 centerMassVelocity = Vector3.zero;
            float massSum = 0;
            float mass = invMass;

            for (int i = 0; i < numParticles; i++) {
                massSum += mass;
                centerMassPosition += positions[i] * mass;
                centerMassVelocity += velocities[i] * mass;
            }
            centerMassPosition /= massSum;
            centerMassVelocity /= massSum;

            // now compute L = sum of all r cross mass * velocity
            // also compute I = sum of rs * rs_transpose * mass
            Vector3 L = Vector3.zero;
            Matrix4x4 I = Matrix4x4.zero;

            for (int i = 0; i < numParticles; i++) {
                //  r is position - center of mass
                Vector3 r = positions[i] - centerMassPosition;
                L += Vector3.Cross(r, mass * velocities[i]);
                // rs = [ 0      -r.z     r.y  ]
                //        r.z     0      -r.x
                //       -r.y     r.x     0
                Matrix4x4 rs = Matrix4x4.zero;
                rs[0, 1] = -r[2];
                rs[0, 2] = r[1];
                rs[1, 0] = r[2];
                rs[1, 2] = -r[0];
                rs[2, 0] = -r[1];
                rs[2, 1] = r[0];
                rs[3, 3] = 1;
                Matrix4x4 temp = Utility.ScaleMatrixByFloat(rs * rs.transpose, mass);
                I = Utility.AddMatrices(I, temp);
            }

            // w = I_inverse * L
            I[3, 3] = 1;
            Matrix4x4 I_inv = I.inverse;
            Vector3 w = I_inv * L;

            // apply w back into velocities
            for (int i = 0; i < numParticles; i++) {
                Vector3 r = positions[i] - centerMassPosition;
                Vector3 dv = centerMassVelocity + Vector3.Cross(w, r) - velocities[i];
                velocities[i] += dampingStiffness * dv;
            }
        }
    }

    public void ClearCollisionConstraints() {
        collisionConstraints.Clear();
        for (int i = 0; i < numParticles; i++) {
            frictions[i] = 1;
        }
    }

    public void GenerateCollisionConstraints() {
        Collider[] colliders = FindObjectsOfType<Collider>();

        for (int i = 0; i < numParticles; i++) {
            for (int j = 0; j < colliders.Length; j++) {
                bool collided = false;

                if (colliders[j].GetType() == typeof(SphereCollider)) {
                    Vector3 center = colliders[j].GetComponent<SphereCollider>().center + colliders[j].transform.position;
                    float radius = colliders[j].GetComponent<SphereCollider>().radius * colliders[j].transform.lossyScale.x;
                    if ((projectedPositions[i] - center).magnitude < radius) {
                        collided = true;
                        collisionConstraints.Add(new SphereCollisionConstraint(i, center, radius, positions[i], projectedPositions[i]));
                    }
                }
                else if (colliders[j].GetType() == typeof(BoxCollider)) {
                    Vector3 extent = 0.5f * colliders[j].GetComponent<BoxCollider>().size;
                    Vector3 localProjectedPosition = colliders[j].transform.InverseTransformPoint(projectedPositions[i]);
                    if (Utility.IsPointInCube(localProjectedPosition, extent)) {
                        collided = true;
                        Vector3 localPosition = colliders[j].transform.InverseTransformPoint(positions[i]);
                        collisionConstraints.Add(new CubeCollisionConstraint(i, localPosition, localProjectedPosition, extent, colliders[j].transform));
                    }
                }
                else if (colliders[j].GetType() == typeof(CapsuleCollider)) {
                    // TODO
                }
                else if (colliders[j].GetType() == typeof(MeshCollider)) {
                    Ray r = new Ray(positions[i], (projectedPositions[i] - positions[i]).normalized);
                    float maxDist = (projectedPositions[i] - positions[i]).magnitude;
                    RaycastHit hitInfo;
                    if (colliders[j].Raycast(r, out hitInfo, maxDist)) {
                        collided = true;
                        Mesh mesh = colliders[j].GetComponent<MeshCollider>().sharedMesh;
                        int[] vertexIndices = {
                            mesh.triangles[hitInfo.triangleIndex * 3],
                            mesh.triangles[hitInfo.triangleIndex * 3 + 1],
                            mesh.triangles[hitInfo.triangleIndex * 3 + 2] };
                        Vector3[] triangleVertices = {
                            colliders[j].transform.TransformPoint(mesh.vertices[vertexIndices[0]]),
                            colliders[j].transform.TransformPoint(mesh.vertices[vertexIndices[1]]),
                            colliders[j].transform.TransformPoint(mesh.vertices[vertexIndices[2]]) };
                        collisionConstraints.Add(new MeshCollisionConstraint(i, hitInfo.point, hitInfo.normal, triangleVertices));
                    }
                }

                if (collided) { 
                    ClothFrictionCollider frictionCollider = colliders[j].gameObject.GetComponent<ClothFrictionCollider>();
                    if (frictionCollider != null) {
                        frictions[i] = Mathf.Min(frictions[i], 1f - frictionCollider.friction);
                    }
                }
            }
        }
    }

    public void ApplyExplicitEuler(float dt) {
        for (int i = 0; i < numParticles; i++) {
            projectedPositions[i] = positions[i] + velocities[i] * dt;
        }
    }

    public void SatisfyConstraints() {
        // randomly shuffle constraints
        constraints.Shuffle();
        collisionConstraints.Shuffle();

        // satisfy normal constraints first
        for (int i = 0; i < constraints.Count; i++) {
            constraints[i].Satisfy(projectedPositions, invMass);
        }

        // then satisfy collision constraints
        for (int i = 0; i < collisionConstraints.Count; i++) {
            collisionConstraints[i].Satisfy(projectedPositions, invMass);
        }

        //finally, satisfy ground constraints
        if (ground != null) {
            ground.Satisfy(projectedPositions, velocities);
        }
    }

    public void SatisfyPointConstraints(float dt) {
        for (int i = 0; i < pointConstraints.Count; i++) {
            pointConstraints[i].Satisfy(projectedPositions, positions);
        }
    }

    public void UpdateVertices(float dt) {
        for (int i = 0; i < numParticles; i++) {
            // step 13: velocity = (projectedPos - currentPos) / dt
            velocities[i] = (projectedPositions[i] - positions[i]) / dt;
            // step 14: currentPos = projectedPos
            positions[i] = projectedPositions[i];
        }
    }

    public void ApplyFriction() {
        // do simple air resistance by reducing speed of every vertex
        for (int i = 0; i < numParticles; i++) {
            velocities[i] *= 0.998f;
        }
        // do simple ground friction by setting speed of objects on ground to zero
        for (int i = 0; i < numParticles; i++) {
            if (frictions[i] < 1) {
                velocities[i] *= frictions[i];
                //velocities[i] = new Vector3(Mathf.Abs(velocities[i][0]) < 0.2f ? 0 : velocities[i][0],
                //                            Mathf.Abs(velocities[i][1]) < 0.2f ? 0 : velocities[i][1],
                //                            Mathf.Abs(velocities[i][2]) < 0.2f ? 0 : velocities[i][2]);
            }
        }
    }

    
}


public class Triangle {
    public int[] vertices;

    public Triangle(int v0, int v1, int v2) {
        vertices = new int[3];
        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;
    }
}

public class Edge {
    public int startIndex;
    public int endIndex;

    public Edge(int start, int end) {
        startIndex = Mathf.Min(start, end);
        endIndex = Mathf.Max(start, end);
    }
}

public class EdgeComparer: EqualityComparer<Edge> { 
    public override int GetHashCode(Edge obj) {
        return obj.startIndex * 10000 + obj.endIndex;
    }

    public override bool Equals(Edge x, Edge y) {
        return x.startIndex == y.startIndex && x.endIndex == y.endIndex;
    }
}

public enum DampingMethod { noDamping, simpleDamping, smartDamping }
public enum BendingMethod { noBending, DihedralBending, isometricBending }