using System.Linq;
using System.Collections.Generic;
using UnityEngine;


public class ClothSimulator : MonoBehaviour {
    [Header("Simulation Parameters")]
    public float timestep = 0.02f;
    public int iterationNum = 5;
    public float vertexMass = 2;

    [Header("External Forces")]
    public Vector3 gravity;

    [Header("Cloth Parameters")]
    public int rows = 10;
    public int columns = 10;
    
    [Header("Distance Constraint")]
    public float distanceCompressionStiffness = 0.8f;
    public float distanceStretchStiffness = 0.8f;

    [Header("Bending Constraint")]
    public float bendingStiffness = 0.1f;
    public BendingMethod bendingMethod;

    [Header("Velocity Damping")]
    public DampingMethod dampingMethod;
    public float dampingStiffness = 0.02f;

    [Header("Point Constraints")]
    public PointConstraintType pointConstraintType;
    public int[] pointConstraintCustomIndices;

    [Header("Collision")]
    public GameObject[] collidableObjects;

    // simulation data
    private float nextFrameTime = 0f;
    private Vector3[] positions; 
    private Vector3[] projectedPositions;
    private Vector3[] velocities;
    private float[] frictions;
    private Triangle[] triangles;
    private float invMass;
    private List<Constraint> constraints = new List<Constraint>();
    private List<Constraint> collisionConstraints = new List<Constraint>();
    private List<PointConstraint> pointConstraints = new List<PointConstraint>();
    private int numParticles;

    // unity data
    private Mesh mesh;
    private Mesh reverseMesh;
    

    private void Start () {
        // create new mesh
        mesh = Utility.CreateClothMesh(rows, columns);
        mesh.MarkDynamic();
        transform.GetComponent<MeshFilter>().mesh = mesh;

        numParticles = mesh.vertexCount;
        Vector3[] baseVertices = mesh.vertices;

        positions = new Vector3[numParticles];
        projectedPositions = new Vector3[numParticles];
        velocities = new Vector3[numParticles];
        frictions = new float[numParticles];

        // create a new mesh for the opposite side
        CreateBackSide();

        // step 1-3: initialize position, velocity and weight
        for (int i = 0; i < numParticles; i++) {
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

        // modify positions to world coordinates before calculating constraint restlengths
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.TransformPoint(positions[i]);
        }

        // add constraints
        AddDistanceConstraints();
        AddPointConstraints();
        AddBendingConstraints();

        // modify positions to world coordinates before calculating constraint restlengths
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.InverseTransformPoint(positions[i]);
        }
    }
    

    private void Update () {
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

            // step 8: clear current collisions and generate new collisions
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
            velocities[i] = transform.InverseTransformVector(velocities[i]);
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

        //print(Time.deltaTime + ", " + iter);
    }


    private void CreateBackSide() {
        GameObject newCloth = new GameObject("back");
        newCloth.transform.parent = transform;
        newCloth.transform.localPosition = Vector3.zero;
        newCloth.transform.localRotation = Quaternion.identity;
        newCloth.transform.localScale = new Vector3(1, 1, 1);
        newCloth.AddComponent<MeshRenderer>();
        newCloth.GetComponent<MeshRenderer>().material = GetComponent<MeshRenderer>().material;
        newCloth.AddComponent<MeshFilter>();
        reverseMesh = Utility.DeepCopyMesh(mesh);
        reverseMesh.MarkDynamic();

        // reverse the triangle order
        for (int m = 0; m < reverseMesh.subMeshCount; m++) {
            int[] triangles = reverseMesh.GetTriangles(m);
            for (int i = 0; i < triangles.Length; i += 3) {
                int temp = triangles[i + 0];
                triangles[i + 0] = triangles[i + 1];
                triangles[i + 1] = temp;
            }
            reverseMesh.SetTriangles(triangles, m);
        }
        newCloth.GetComponent<MeshFilter>().mesh = reverseMesh;
    }


    private void ApplyExternalForce(Vector3 gravity, float dt) {
        for (int i = 0; i < numParticles; i++) {
            velocities[i] += gravity * invMass * dt;
        }
    }


    private void DampVelocity(float dampingStiffness, DampingMethod method) {
        // dumb and simple way
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


    private void ClearCollisionConstraints() {
        collisionConstraints.Clear();
        for (int i = 0; i < numParticles; i++) {
            frictions[i] = 1;
        }
    }

    
    private void GenerateCollisionConstraints() {
        for (int i = 0; i < numParticles; i++) {
            for (int j = 0; j < collidableObjects.Length; j++) {
                if (!collidableObjects[j].activeSelf) continue;

                bool collided = false;
                Collider collider = collidableObjects[j].GetComponent<Collider>();

                if (collider.GetType() == typeof(SphereCollider)) {
                    Vector3 center = collider.GetComponent<SphereCollider>().center + collider.transform.position;
                    float radius = collider.GetComponent<SphereCollider>().radius * collider.transform.lossyScale.x;
                    if ((projectedPositions[i] - center).magnitude < radius) {
                        collided = true;
                        collisionConstraints.Add(new SphereCollisionConstraint(i, center, radius, positions[i], projectedPositions[i]));
                    }
                }
                else if (collider.GetType() == typeof(BoxCollider)) {
                    Vector3 extent = 0.5f * collider.GetComponent<BoxCollider>().size;
                    Vector3 localProjectedPosition = collider.transform.InverseTransformPoint(projectedPositions[i]);
                    if (Utility.IsPointInCube(localProjectedPosition, extent)) {
                        collided = true;
                        Vector3 localPosition = collider.transform.InverseTransformPoint(positions[i]);
                        collisionConstraints.Add(new CubeCollisionConstraint(i, localPosition, localProjectedPosition, extent, collider.transform));
                    }
                }
                else if (collider.GetType() == typeof(CapsuleCollider)) {
                    // TODO
                }
                else if (collider.GetType() == typeof(MeshCollider)) {
                    Ray r = new Ray(positions[i], (projectedPositions[i] - positions[i]).normalized);
                    float maxDist = (projectedPositions[i] - positions[i]).magnitude;
                    RaycastHit hitInfo;
                    if (collider.Raycast(r, out hitInfo, maxDist)) {
                        collided = true;
                        Mesh mesh = collider.GetComponent<MeshCollider>().sharedMesh;
                        int[] vertexIndices = {
                            mesh.triangles[hitInfo.triangleIndex * 3],
                            mesh.triangles[hitInfo.triangleIndex * 3 + 1],
                            mesh.triangles[hitInfo.triangleIndex * 3 + 2] };
                        Vector3[] triangleVertices = {
                            collider.transform.TransformPoint(mesh.vertices[vertexIndices[0]]),
                            collider.transform.TransformPoint(mesh.vertices[vertexIndices[1]]),
                            collider.transform.TransformPoint(mesh.vertices[vertexIndices[2]]) };
                        collisionConstraints.Add(new MeshCollisionConstraint(i, hitInfo.point, hitInfo.normal, triangleVertices));
                    }
                }

                if (collided) { 
                    ClothFrictionCollider frictionCollider = collider.gameObject.GetComponent<ClothFrictionCollider>();
                    if (frictionCollider != null) {
                        frictions[i] = Mathf.Min(frictions[i], 1f - frictionCollider.friction);
                    }
                }
            }
        }
    }


    private void AddDistanceConstraints() {
        // use a set to get unique edges
        HashSet<Edge> edgeSet = new HashSet<Edge>(new EdgeComparer());
        for (int i = 0; i < triangles.Length; i++) {
            edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[1]));
            edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[2]));
            edgeSet.Add(new Edge(triangles[i].vertices[1], triangles[i].vertices[2]));
        };

        foreach (Edge e in edgeSet) {
            constraints.Add(new DistanceConstraint(e, positions, distanceCompressionStiffness, distanceStretchStiffness));
        }
    }


    private void AddBendingConstraints() {
        if (bendingMethod != BendingMethod.noBending) {
            return;
        }

        Dictionary<Edge, List<Triangle>> wingEdges = new Dictionary<Edge, List<Triangle>>(new EdgeComparer());

        // map edges to all of the faces to which they are connected
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

        foreach (Edge wingEdge in wingEdges.Keys) {
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


    private void AddPointConstraints() {
        if (pointConstraintType == PointConstraintType.none) {
            return;
        }
        else if (pointConstraintType == PointConstraintType.topCorners) {
            pointConstraints.Add(new PointConstraint(rows * (columns + 1)));
            pointConstraints.Add(new PointConstraint((rows + 1) * (columns + 1) - 1));
        }
        else if (pointConstraintType == PointConstraintType.topRow) {
            for (int i = 0; i <= columns; i++) {
                pointConstraints.Add(new PointConstraint(rows * (columns + 1) + i));
            }
        }
        else if (pointConstraintType == PointConstraintType.leftCorners) {
            pointConstraints.Add(new PointConstraint(0));
            pointConstraints.Add(new PointConstraint(rows * (columns + 1)));
        }
        else if (pointConstraintType == PointConstraintType.leftRow) {
            for (int i = 0; i <= rows; i++) {
                pointConstraints.Add(new PointConstraint(i * (columns + 1)));
            }
        }
        else if (pointConstraintType == PointConstraintType.custom) {
            for (int i = 0; i < pointConstraintCustomIndices.Length; i++) {
                int index = pointConstraintCustomIndices[i];
                if (index >= 0 && index < numParticles) {
                    pointConstraints.Add(new PointConstraint(index));
                }
            }
        }
    }


    private void ApplyExplicitEuler(float dt) {
        for (int i = 0; i < numParticles; i++) {
            projectedPositions[i] = positions[i] + velocities[i] * dt;
        }
    }


    private void SatisfyConstraints() {
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
    }


    private void SatisfyPointConstraints(float dt) {
        for (int i = 0; i < pointConstraints.Count; i++) {
            pointConstraints[i].Satisfy(projectedPositions, positions);
        }
    }


    private void UpdateVertices(float dt) {
        for (int i = 0; i < numParticles; i++) {
            // step 13: velocity = (projectedPos - currentPos) / dt
            velocities[i] = (projectedPositions[i] - positions[i]) / dt;
            // step 14: currentPos = projectedPos
            positions[i] = projectedPositions[i];
        }
    }


    private void ApplyFriction() {
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

