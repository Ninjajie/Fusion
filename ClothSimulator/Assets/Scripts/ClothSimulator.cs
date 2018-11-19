using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClothSimulator : MonoBehaviour {
    // simulation variables
    public float timestep = 0.02f;
    public int iterationNum = 5;
    public float vertexMass = 2;
    public Vector3 gravity;
    public int rows = 10;
    public int columns = 10;

    // constraint weights
    public float distanceWeight = 0.8f;
    public float bendingWeight = 0.5f;
    public float dampingStiffness = 0.02f;

    // constraint data
    public int[] pointConstraintIndices;
    public bool useDistanceConstraint;
    public bool useBendingConstraint;

    // simulation data
    private Vector3[] positions; 
    private Vector3[] projectedPositions;
    private Vector3[] velocities;
    private Triangle[] triangles;
    private float invMass;
    private List<Constraint> constraints = new List<Constraint>();
    private List<Constraint> collisionConstraints = new List<Constraint>();
    private List<Constraint> pointConstraints = new List<Constraint>();
    private int numParticles;

    // unity data
    Mesh mesh;

    // Use this for initialization
    void Start () {
        mesh = GetComponent<MeshFilter>().mesh;
        numParticles = mesh.vertexCount;
        Vector3[] baseVertices = mesh.vertices;
        positions = new Vector3[numParticles];
        projectedPositions = new Vector3[numParticles];
        velocities = new Vector3[numParticles];

        // step 1-3: initialize position, velocity and weight
        for (int i = 0; i < numParticles; i++) {
            positions[i] = baseVertices[i];
            projectedPositions[i] = baseVertices[i];
            velocities[i] = Vector3.zero;
        }
        invMass = 1.0f / vertexMass;

        // initialize triangles
        int[] triangleIndices = mesh.GetTriangles(0);
        triangles = new Triangle[triangleIndices.Length / 3];
        for (int i = 0; i < triangles.Length; i++) {
            triangles[i] = new Triangle(triangleIndices[i * 3], triangleIndices[i * 3 + 1], triangleIndices[i * 3 + 2]);
        }

        // add constraints
        if (pointConstraintIndices.Length > 0) {
            for (int i = 0; i < pointConstraintIndices.Length; i++) {
                pointConstraints.Add(new PointConstraint(pointConstraintIndices[i]));
            }
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
                Vector3 startPos = positions[e.startIndex];
                Vector3 endPos = positions[e.endIndex];
                float restLength = (startPos - endPos).magnitude;

                constraints.Add(new DistanceConstraint(e, restLength, distanceWeight));
            }
        }
        if (useBendingConstraint) {
            // TODO: implement this
        }
    }

    void FixedUpdate () {
        // TODO: set dt to deltaTime for now. change this later for more customization
        float dt = Time.fixedDeltaTime;

        // step 5: apply external forces
        ApplyExternalForce(gravity, dt);

        // step 6: damp velocity
        DampVelocity(dampingStiffness);

        // step 7: apply explicit Euler to positions based on velocity
        ApplyExplicitEuler(dt);

        // step 8: TODO: clear current collisions and generate new collisions
        //clearCollisionConstraints();
        //for (auto & s : spheres) {
        //    s.move(dt);
        //    generateCollisionConstraints(s);
        //}

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

        // update everything into Unity
        mesh.vertices = positions;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        GetComponent<MeshCollider>().sharedMesh = mesh;
    }

    public void ApplyExternalForce(Vector3 gravity, float dt) {
        for (int i = 0; i < numParticles; i++) {
            velocities[i] += gravity * invMass * dt;
        }
    }

    public void DampVelocity(float dampingStiffness) {
        // TODO: there's a smarter way of doing this
        for (int i = 0; i < numParticles; i++) { 
            velocities[i] *= 0.98f;
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
            constraints[i].Satisfy(projectedPositions, positions, invMass);
        }

        // then satisfy collision constraints
        for (int i = 0; i < collisionConstraints.Count; i++) {
            collisionConstraints[i].Satisfy(projectedPositions, positions, invMass);
        }

        // finally, satisfy ground constraints
        //if (groundConstraint != nullptr) {
        //    groundConstraint->satisfy(projectedPositions, velocities, masses);
        //}
    }

    public void SatisfyPointConstraints(float dt) {
        for (int i = 0; i < pointConstraints.Count; i++) {
            pointConstraints[i].Satisfy(projectedPositions, positions, invMass);
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
        //// do simple ground friction by setting speed of objects on ground to zero
        //for (int i = 0; i < numParticles; i++) {
        //    if (positions[i][1] == 0) {
        //        velocities[i][0] *= .9;
        //        velocities[i][2] *= .9;
        //        if (std::abs(velocities[i][0] < 0.2)) velocities[i][0] = 0;
        //        if (std::abs(velocities[i][2] < 0.2)) velocities[i][2] = 0;
        //    }
        //}
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
        startIndex = start;
        endIndex = end;
    }
}

public class EdgeComparer: EqualityComparer<Edge> { 
    public override int GetHashCode(Edge obj) {
        return obj.startIndex * 10000 + obj.endIndex;
    }

    public override bool Equals(Edge x, Edge y) {
        return (x.startIndex == y.startIndex && x.endIndex == y.endIndex)
            || (x.endIndex == y.startIndex && x.startIndex == y.endIndex);
    }
}

