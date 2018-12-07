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
    public ComputeShader PBDSolver;

    // simulation data
    private float nextFrameTime = 0f;
    private VectorStruct[] positions;
    private VectorStruct[] projectedPositions;
    private VectorStruct[] velocities;
    private float[] frictions;
    private Triangle[] triangles;
    private float invMass;
    private List<Constraint> constraints = new List<Constraint>();
    private List<Constraint> collisionConstraints = new List<Constraint>();
    private List<PointConstraint> pointConstraints = new List<PointConstraint>();
    private GroundConstraint ground = null;
    private int numParticles;

    DistanceConstraintStruct[] distanceConstraints;
    VectorStruct[] deltaPositionArray;    // The array that stores all the deltas
    int[] deltaCounterArray;              // The array to initialize delta count buffer
    UInt3Struct[] deltaPosUintArray;      // The array to initialize deltaposInt buffer

    // unity data
    private Mesh mesh;
    private Mesh reverseMesh;


    private void Start () {
		
	}


    private void Update () {
		
	}
}
