using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GPUClothSimulator : MonoBehaviour {
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

    [Header("Compute Shader")]
    public ComputeShader PBDClothSolver;
    public int workGroupSize = 8; // TODO: figure out what the best value for this is

    // simulation data
    private float nextFrameTime = 0f;
    private Vector3[] positions;
    private Vector3[] velocities;
    private float[] frictions;
    private float invMass;
    private int numParticles, numEdges;
    // TODO: can we remove the following 3?
    private Vector3[] deltaPositionArray;    // The array that stores all the deltas
    private int[] deltaCounterArray;              // The array to initialize delta count buffer
    private UInt3Struct[] deltaPosUintArray;      // The array to initialize deltaposInt buffer

    // constraints
    //private List<Constraint> constraints = new List<Constraint>();
    //private List<Constraint> collisionConstraints = new List<Constraint>();
    private DistanceConstraintStruct[] distanceConstraints;
    private List<PointConstraint> pointConstraints = new List<PointConstraint>();

    // compute buffers
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer projectedPositionsBuffer;
    private ComputeBuffer velocitiesBuffer;
    private ComputeBuffer distanceConstraintsBuffer;
    private ComputeBuffer deltaPositionsBuffer;
    private ComputeBuffer deltaPositionsUIntBuffer;
    private ComputeBuffer deltaCounterBuffer;

    // kernel IDs
    private int applyExternalForcesKernel;
    private int dampVelocitiesKernel;
    private int applyExplicitEulerKernel;
    private int projectConstraintDeltasKernel;
    private int averageConstraintDeltasKernel;
    private int updatePositionsKernel;

    // num of work groups
    private int numGroups_Vertices;
    private int numGroups_Edges;
    private int numGroups_VE;

    // mesh data
    private Mesh mesh;
    private Mesh reverseMesh;
    private Triangle[] triangles;



    private void Start () {
        // create new mesh
        mesh = Utility.CreateClothMesh(rows, columns);
        mesh.MarkDynamic();
        transform.GetComponent<MeshFilter>().mesh = mesh;

        numParticles = mesh.vertexCount;
        Vector3[] baseVertices = mesh.vertices;

        positions = new Vector3[numParticles];
        velocities = new Vector3[numParticles];
        frictions = new float[numParticles];
        deltaPositionArray = new Vector3[numParticles];
        deltaPosUintArray = new UInt3Struct[numParticles];
        deltaCounterArray = new int[numParticles];

        // create a new mesh for the opposite side
        CreateBackSide();

        // step 1-3: initialize position, velocity and weight
        for (int i = 0; i < numParticles; i++) {
            positions[i] = baseVertices[i];
            velocities[i] = Vector3.zero;
            frictions[i] = 1;

            // initialize delta Pos array
            deltaPositionArray[i] = Vector3.zero;

            // initialize delta Pos int array
            deltaPosUintArray[i].deltaXInt = 0;
            deltaPosUintArray[i].deltaYInt = 0;
            deltaPosUintArray[i].deltaZInt = 0;

            // initialize delta counter array
            deltaCounterArray[i] = 0;
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
        //AddBendingConstraints();

        // modify positions to world coordinates before calculating constraint restlengths
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.InverseTransformPoint(positions[i]);
        }

        SetupComputeBuffers();
    }
  

    private void Update() {
        // TODO: do this on the GPU or figure out way to keep everything in local coor on the GPU (by setting external and collisions to local)
        // modify data to world coordinates
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.TransformPoint(positions[i]);
            velocities[i] = transform.TransformVector(velocities[i]);
        }
        positionsBuffer.SetData(positions);
        velocitiesBuffer.SetData(velocities);

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

            // send the dt data to the GPU
            PBDClothSolver.SetFloat("dt", dt);

            // step 5: apply external forces
            PBDClothSolver.Dispatch(applyExternalForcesKernel, numGroups_Vertices, 1, 1);

            // step 6: damp velocity
            if (dampingMethod != DampingMethod.noDamping) {
                PBDClothSolver.Dispatch(dampVelocitiesKernel, numGroups_Vertices, 1, 1);
            }

            // step 7: apply explicit Euler to positions based on velocity
            PBDClothSolver.Dispatch(applyExplicitEulerKernel, numGroups_Vertices, 1, 1);

            // step 8: clear current collisions and generate new collisions
            //ClearCollisionConstraints();
            //GenerateCollisionConstraints();

            // step 9-11: project constraints iterationNum times
            //for (int j = 0; j < iterationNum; j++) {
            //    // satisfy all constraints
            //    SatisfyConstraints();
            //}

            //// satisfy pointConstraints
            //SatisfyPointConstraints(dt);

            // step 13 & 14: apply projected positions to actual vertices
            PBDClothSolver.Dispatch(updatePositionsKernel, numGroups_Vertices, 1, 1);

            // step 16: update all velocities using friction
            //ApplyFriction();
        }

        // get data from GPU back to CPU
        positionsBuffer.GetData(positions);
        velocitiesBuffer.GetData(velocities);

        // recalculate the center of the mesh
        Vector3 newCenter = GetComponentInChildren<Renderer>().bounds.center;
        Vector3 delta = newCenter - transform.position;

        // modify data to back to local coordinates
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.InverseTransformPoint(positions[i] - delta);
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


    private void AddDistanceConstraints() {
        // use a set to get unique edges
        HashSet<Edge> edgeSet = new HashSet<Edge>(new EdgeComparer());
        for (int i = 0; i < triangles.Length; i++) {
            edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[1]));
            edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[2]));
            edgeSet.Add(new Edge(triangles[i].vertices[1], triangles[i].vertices[2]));
        };

        numEdges = edgeSet.Count;
        distanceConstraints = new DistanceConstraintStruct[numEdges];
        int j = 0;
        foreach (Edge e in edgeSet) {
            EdgeStruct edge;
            edge.startIndex = e.startIndex;
            edge.endIndex = e.endIndex;
            distanceConstraints[j].edge = edge;
            distanceConstraints[j].restLength = (positions[edge.startIndex] - positions[edge.endIndex]).magnitude;
            distanceConstraints[j].compressionStiffness = distanceCompressionStiffness;
            distanceConstraints[j].stretchStiffNess = distanceStretchStiffness;
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

    private void SetupComputeBuffers() {
        // create the compute buffers
        positionsBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3);
        projectedPositionsBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3);
        velocitiesBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3);
        distanceConstraintsBuffer = new ComputeBuffer(numEdges, sizeof(float) * 3 + sizeof(int) * 2);
        deltaPositionsBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3);
        deltaPositionsUIntBuffer = new ComputeBuffer(numParticles, sizeof(uint) * 3);
        deltaCounterBuffer = new ComputeBuffer(numParticles, sizeof(int));

        // fill buffers with initial data
        positionsBuffer.SetData(positions);
        projectedPositionsBuffer.SetData(positions);
        velocitiesBuffer.SetData(velocities);
        distanceConstraintsBuffer.SetData(distanceConstraints);
        deltaPositionsBuffer.SetData(deltaPositionArray);
        deltaPositionsUIntBuffer.SetData(deltaPosUintArray);
        deltaCounterBuffer.SetData(deltaCounterArray);

        // identify the kernels
        applyExternalForcesKernel = PBDClothSolver.FindKernel("ApplyExternalForces");
        dampVelocitiesKernel = PBDClothSolver.FindKernel("DampVelocities");
        applyExplicitEulerKernel = PBDClothSolver.FindKernel("ApplyExplicitEuler");
        projectConstraintDeltasKernel = PBDClothSolver.FindKernel("ProjectConstraintDeltas");
        averageConstraintDeltasKernel = PBDClothSolver.FindKernel("AverageConstraintDeltas");
        updatePositionsKernel = PBDClothSolver.FindKernel("UpdatePositions");

        // set uniform data for kernels
        PBDClothSolver.SetInt("numParticles", numParticles);
        PBDClothSolver.SetVector("gravity", gravity);
        PBDClothSolver.SetFloat("invMass", invMass);

        // bind buffer data to each kernel
        PBDClothSolver.SetBuffer(applyExternalForcesKernel, "velocities", velocitiesBuffer);

        PBDClothSolver.SetBuffer(dampVelocitiesKernel, "velocities", velocitiesBuffer);

        PBDClothSolver.SetBuffer(applyExplicitEulerKernel, "positions", positionsBuffer);
        PBDClothSolver.SetBuffer(applyExplicitEulerKernel, "projectedPositions", projectedPositionsBuffer);
        PBDClothSolver.SetBuffer(applyExplicitEulerKernel, "velocities", velocitiesBuffer);

        PBDClothSolver.SetBuffer(updatePositionsKernel, "positions", positionsBuffer);
        PBDClothSolver.SetBuffer(updatePositionsKernel, "projectedPositions", projectedPositionsBuffer);
        PBDClothSolver.SetBuffer(updatePositionsKernel, "velocities", velocitiesBuffer);


        //calculate and set the work group size
        numGroups_Vertices = Mathf.CeilToInt((float)numParticles / workGroupSize);
        numGroups_Edges = Mathf.CeilToInt((float)numEdges / workGroupSize);
    }

    private void OnDestroy() {
        if (positionsBuffer != null)
            positionsBuffer.Release();

        if (projectedPositionsBuffer != null)
            projectedPositionsBuffer.Release();

        if (velocitiesBuffer != null)
            velocitiesBuffer.Release();

        if (distanceConstraintsBuffer != null)
            distanceConstraintsBuffer.Release();

        if (deltaPositionsBuffer != null)
            deltaPositionsBuffer.Release();

        if (deltaPositionsUIntBuffer != null)
            deltaPositionsUIntBuffer.Release();

        if (deltaCounterBuffer != null)
            deltaCounterBuffer.Release();
    }
}
