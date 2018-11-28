using System.Collections;
using System.Collections.Generic;
using UnityEngine;

struct PointElement
{
    // TODO: change the Point Element structure when needed
    public Vector3 worldPosition;
    public Vector3 worldVelocity;
    public float invMass;
};

//struct PointPositionInt
//{
//    // TODO: change the Point Element structure when needed
//    public int intPosX;
//    public int intPosY;
//    public int intPosZ;
//};

struct DeltaPos
{
    public float deltaX;
    public float deltaY;
    public float deltaZ;
}

struct DeltaPosUInt
{
    public uint deltaXInt;
    public uint deltaYInt;
    public uint deltaZInt;
}

struct LineElement
{
    public int startIndex;
    public int endIndex;
};

struct DistanceConstraintStruct
{
    public LineElement edge;
    public float restLength;
    public float weight;
};



public class ClothGPU : MonoBehaviour
{
    // the unity mesh
    private Mesh mesh;
    // the positions of mesh vertices
    private Vector3[] baseVertices;
    // the set of triangles in original mesh
    private Triangle[] triangles;
    // test cast variable
    public float speed;
    // index of fixed vertex
    public int pointConstraintIndex1 = 110;
    public int pointConstraintIndex2 = 120;

    // compute shaders
    public ComputeShader applyExtForce;
    public ComputeShader dampVelocity;
    public ComputeShader positionPrediction;
    public ComputeShader solvingConstraints;
    public ComputeShader pointConstraints;
    public ComputeShader updatePosVel;

    // Compute buffers for point Element
    ComputeBuffer PointElementBuffer;
    ComputeBuffer PointElementOldBuffer;
    // Compute buffers for Distance Constraints
    ComputeBuffer distanceConstraintsBuffer;
    ComputeBuffer DeltaPosBuffer;
    ComputeBuffer DeltaCounterBuffer;
    ComputeBuffer DeltaPosIntBuffer;

    // kernel IDs
    int PBD_ExtForceKernelID;
    int PBD_SolvingConstraintsKernelID;
    int PBD_AveragingDeltasKernelID;
    int PBD_PointConstraintsKernelID;
    int PBD_UpdataPosVelKernelID;

    // size of PointElement struct
    int _size_PointElement = 28;
    // size of Line element struct
    int _size_LineElement = 8;
    // size of distance constraint struct
    int _size_DistanceConstraint = 16;
    // size of DeltaPos struct
    int _size_DeltaPos = 12;
    // size of DeltaPosInt struct
    int _size_DeltaPosInt = 12;

    // num of work groups
    int numGroups_Vertices;
    int numGroups_Edges;
    int numGroups_VE;
    // size of a group
    int _size_WorkGroup = 8;

    int numEdges = 0;
    int numVertices = 0;

    //******Booleans for using certain constraints*************//
    public bool DistanceConstraintsOn = true;
    //******Booleans for using certain constraints end*********//

    // iteration num
    public int iterationNum = 10;

    // constraint weights
    public float distanceWeight = 0.8f;

    // collision sphere
    public float[] center;
    public float radius = 1.0f;
    public Transform sphereTransform;
    Vector3 centerVec3;
    // collision plane
    float yPlane = 0.0f;
    public Transform planeTransform;
    // moving the cloth
    float[] clothTranslate;

    //******Compute buffer data arrays*************//
    // the array that stores data of all point elements
    PointElement[] pointElements;
    // the array that stores the old pos/vel data
    PointElement[] pointElements_Old;
    // the array that stores the distance constraints info
    DistanceConstraintStruct[] distanceConstraints;
    // The array that stores all the deltas
    DeltaPos[] deltaPosArray;
    // The array to initialize delta count buffer
    int[] deltaCounterArray;
    // The array to initialize deltaposInt buffer
    DeltaPosUInt[] deltaPosUintArray;
    //******Compute buffer data arrays end*********//

    // Use this for initialization
    void Start()
    {
        mesh = GetComponent<MeshFilter>().mesh;
        mesh.MarkDynamic();
        baseVertices = mesh.vertices;

        //initialize sphere
        centerVec3 = sphereTransform.position;

        center = new float[3];
        center[0] = centerVec3.x;
        center[1] = centerVec3.y;
        center[2] = centerVec3.z;
        radius = sphereTransform.localScale.y / 2.0f;

        //initialize plane
        yPlane = planeTransform.position.y;
        //initialize cloth translate
        clothTranslate = new float[3];
        clothTranslate[0] = 0.0f;
        clothTranslate[1] = 0.0f;
        clothTranslate[2] = 0.0f;

        // initialize pointElements array
        numVertices = baseVertices.Length;
        pointElements = new PointElement[numVertices];
        pointElements_Old = new PointElement[numVertices];
        // initialize delta positions
        deltaPosArray = new DeltaPos[numVertices];
        // initialize delta counters
        deltaCounterArray = new int[numVertices];
        //initialize deltapos int array
        deltaPosUintArray = new DeltaPosUInt[numVertices];

        for (int i = 0; i < numVertices; ++i)
        {
            pointElements[i].worldPosition = baseVertices[i];
            pointElements[i].worldVelocity = Vector3.zero;
            pointElements[i].invMass = 1.0f;

            pointElements_Old[i].worldPosition = baseVertices[i];
            pointElements_Old[i].worldVelocity = Vector3.zero;
            pointElements_Old[i].invMass = 1.0f;

            // initialize delta Pos array
            deltaPosArray[i].deltaX = 0.0f;
            deltaPosArray[i].deltaY = 0.0f;
            deltaPosArray[i].deltaZ = 0.0f;

            // initialize delta Pos int array
            deltaPosUintArray[i].deltaXInt = 0;
            deltaPosUintArray[i].deltaYInt = 0;
            deltaPosUintArray[i].deltaZInt = 0;

            // initialize delta counter array
            deltaCounterArray[i] = 0;
        }

        // initialize triangles
        int[] triangleIndices = mesh.GetTriangles(0);
        triangles = new Triangle[triangleIndices.Length / 3];
        for (int i = 0; i < triangles.Length; i++)
        {
            triangles[i] = new Triangle(triangleIndices[i * 3], triangleIndices[i * 3 + 1], triangleIndices[i * 3 + 2]);
        }

        //initialize edge set and distance constraints array if necessary
        numEdges = 0;
        if (DistanceConstraintsOn)
        {
            // use a set to get unique edges
            HashSet<Edge> edgeSet = new HashSet<Edge>(new EdgeComparer());
            for (int i = 0; i < triangles.Length; i++)
            {
                edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[1]));
                edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[2]));
                edgeSet.Add(new Edge(triangles[i].vertices[1], triangles[i].vertices[2]));
            };

            numEdges = edgeSet.Count;
            distanceConstraints = new DistanceConstraintStruct[numEdges];
            int count = 0;
            foreach (Edge e in edgeSet)
            {
                Vector3 startPos = baseVertices[e.startIndex];
                Vector3 endPos = baseVertices[e.endIndex];
                float restLength = (startPos - endPos).magnitude;
                LineElement l;
                l.startIndex = e.startIndex;
                l.endIndex = e.endIndex;
                distanceConstraints[count].edge = l;
                distanceConstraints[count].restLength = restLength;
                distanceConstraints[count].weight = distanceWeight;
                count++;
            }
            Debug.Log(count);
        }

        // create computebuffer
        PointElementBuffer = new ComputeBuffer(numVertices, _size_PointElement);
        PointElementOldBuffer = new ComputeBuffer(numVertices, _size_PointElement);
        distanceConstraintsBuffer = new ComputeBuffer(numEdges, _size_DistanceConstraint);
        DeltaPosBuffer = new ComputeBuffer(numVertices, _size_DeltaPos);
        DeltaCounterBuffer = new ComputeBuffer(numVertices, 4);
        DeltaPosIntBuffer = new ComputeBuffer(numVertices, _size_DeltaPosInt);

        //set data for PointElementOldBuffer
        PointElementOldBuffer.SetData(pointElements_Old);
        // TODO: do I need to set data for PointElementBuffer?

        //identify the kernels
        PBD_ExtForceKernelID = applyExtForce.FindKernel("CSMain");
        PBD_SolvingConstraintsKernelID = solvingConstraints.FindKernel("CalculateDeltas");
        PBD_AveragingDeltasKernelID = solvingConstraints.FindKernel("AveragingDeltas");
        PBD_PointConstraintsKernelID = pointConstraints.FindKernel("CSMain");
        PBD_UpdataPosVelKernelID = updatePosVel.FindKernel("UpdatePosVel");

        //bind computebuffer
        applyExtForce.SetBuffer(PBD_ExtForceKernelID, "pointElements", PointElementBuffer);

        solvingConstraints.SetBuffer(PBD_SolvingConstraintsKernelID, "distanceConstraintData", distanceConstraintsBuffer);
        solvingConstraints.SetBuffer(PBD_SolvingConstraintsKernelID, "pointElements", PointElementBuffer);
        solvingConstraints.SetBuffer(PBD_SolvingConstraintsKernelID, "pointElements_Old", PointElementOldBuffer);
        solvingConstraints.SetBuffer(PBD_SolvingConstraintsKernelID, "deltaPos", DeltaPosBuffer);
        solvingConstraints.SetBuffer(PBD_SolvingConstraintsKernelID, "deltaCount", DeltaCounterBuffer);
        solvingConstraints.SetBuffer(PBD_SolvingConstraintsKernelID, "deltaPosASInt", DeltaPosIntBuffer);
        //solvingConstraints.SetBuffer(PBD_SolvingConstraintsKernelID, "pointElements_Old", PointElementOldBuffer);
        solvingConstraints.SetBuffer(PBD_AveragingDeltasKernelID, "pointElements", PointElementBuffer);
        solvingConstraints.SetBuffer(PBD_AveragingDeltasKernelID, "deltaPos", DeltaPosBuffer);
        solvingConstraints.SetBuffer(PBD_AveragingDeltasKernelID, "deltaCount", DeltaCounterBuffer);
        solvingConstraints.SetBuffer(PBD_AveragingDeltasKernelID, "deltaPosASInt", DeltaPosIntBuffer);

        pointConstraints.SetBuffer(PBD_PointConstraintsKernelID, "pointElements", PointElementBuffer);
        pointConstraints.SetBuffer(PBD_PointConstraintsKernelID, "pointElements_Old", PointElementOldBuffer);

        updatePosVel.SetBuffer(PBD_UpdataPosVelKernelID, "pointElements", PointElementBuffer);
        updatePosVel.SetBuffer(PBD_UpdataPosVelKernelID, "pointElements_Old", PointElementOldBuffer);

        //set data for compute buffers (data that only needs to be set once in the beginning)
        distanceConstraintsBuffer.SetData(distanceConstraints);
        DeltaPosBuffer.SetData(deltaPosArray);
        DeltaCounterBuffer.SetData(deltaCounterArray);
        DeltaPosIntBuffer.SetData(deltaPosUintArray);

        //calculate and set the work group size
        numGroups_Vertices = Mathf.CeilToInt((float)numVertices / _size_WorkGroup);
        Debug.Log(numGroups_Vertices);
        numGroups_Edges = Mathf.CeilToInt((float)numEdges / _size_WorkGroup);
        Debug.Log(numGroups_Edges);
        numGroups_VE = Mathf.CeilToInt((float)(numEdges + numVertices) / _size_WorkGroup);
    }

    private void OnDestroy()
    {
        if (PointElementBuffer != null)
            PointElementBuffer.Release();

        if (PointElementOldBuffer != null)
            PointElementOldBuffer.Release();

        if (distanceConstraintsBuffer != null)
            distanceConstraintsBuffer.Release();

        if (DeltaPosBuffer != null)
            DeltaPosBuffer.Release();

        if (DeltaCounterBuffer != null)
            DeltaCounterBuffer.Release();

        if (DeltaPosIntBuffer != null)
            DeltaPosIntBuffer.Release();
    }

    // Update is called once per frame
    void Update()
    {
       
        //send data to compute shader;
        PointElementBuffer.SetData(pointElements);

        //monitor key presses and read transformations
        PressingKeys();

        ApplyExtForce();

        // send data to compute shader
        //distanceConstraintsBuffer.SetData(distanceConstraints);
        //DeltaPosBuffer.SetData(deltaPosArray);
        //DeltaCounterBuffer.SetData(deltaCounterArray);

        SolvingConstraints();

        //DeltaPosBuffer.GetData(deltaPosArray);
        //DeltaCounterBuffer.GetData(deltaCounterArray);

        //Debug.Log(deltaPosArray[0].deltaX);
        //Debug.Log(deltaCounterArray[0]);

        SatisfyPointConstraints(120,110);

        UpdatePosVel();

        // after compute, read modified data from buffer
        PointElementBuffer.GetData(pointElements);

        Debug.Log(pointElements[0].worldPosition.y);

        Vector3[] vertices = new Vector3[baseVertices.Length];

        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = pointElements[i].worldPosition;
        }

        // TODO: reset the old point positons data with updated positions?
        // ping-pong old and new buffer of point positions
        PointElementOldBuffer.SetData(pointElements);

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    void ApplyExtForce()
    {
        // Send uniform data to compute shader
        applyExtForce.SetInt("numVertices", numVertices);

        // Send uniform data to compute shader
        applyExtForce.SetFloat("deltaTime", Time.deltaTime);

        // run the kernel to update pointElements
        applyExtForce.Dispatch(PBD_ExtForceKernelID, numGroups_Vertices, 1, 1);

    }

    void SolvingConstraints()
    {
        // send the iteration num variable to compute shader
        solvingConstraints.SetInt("numIterations", iterationNum);

        // send the num of edges to compute shader
        solvingConstraints.SetInt("numEdges", numEdges);

        // send the num of edges to compute shader
        solvingConstraints.SetInt("numVertices", numVertices);

        // send the center and radius of the sphere
        //solvingConstraints.SetFloat("radius", radius);
        //solvingConstraints.SetFloats("center", center);

        for(int index = 0; index < 10; index++)
        {


            solvingConstraints.Dispatch(PBD_SolvingConstraintsKernelID, numGroups_Edges, 1, 1);

            solvingConstraints.Dispatch(PBD_AveragingDeltasKernelID, numGroups_Vertices, 1, 1);
        }
    }

    void SatisfyPointConstraints(int fixedPointIndex1, int fixedPointIndex2)
    {
        //no need to set pointElementBuffer's data, just directly use the buffer coming out from previous kernels

        //set the index of pinned vertex
        pointConstraints.SetInt("fixedPointIndex1", fixedPointIndex1);
        pointConstraints.SetInt("fixedPointIndex2", fixedPointIndex2);

        // send the num of edges to compute shader
        pointConstraints.SetInt("numVertices", numVertices);

        // send the center and radius of the sphere
        pointConstraints.SetFloat("radius", radius);
        pointConstraints.SetFloats("center", center);

        pointConstraints.SetFloats("clothTranslate", clothTranslate);

        pointConstraints.SetFloat("yPlane", yPlane);
        //run the kernel
        pointConstraints.Dispatch(PBD_PointConstraintsKernelID, numGroups_Vertices, 1, 1);
    }

    void UpdatePosVel()
    {
        updatePosVel.SetInt("numVertices", numVertices);
        updatePosVel.SetFloat("deltaTime", Time.deltaTime);

        updatePosVel.Dispatch(PBD_UpdataPosVelKernelID, numGroups_Vertices, 1, 1);
    }

    void PressingKeys()
    {
        //update yplane for interaction
        yPlane = planeTransform.position.y;

        centerVec3 = sphereTransform.position;

        center[0] = centerVec3.x;
        center[1] = centerVec3.y;
        center[2] = centerVec3.z;

        //update cloth translate
        if (Input.GetKey("up"))
        {
            clothTranslate[1] += 0.001f;
        }
        if (Input.GetKey("down"))
        {
            clothTranslate[1] -= 0.001f;
        }
        if (Input.GetKey("left"))
        {
            clothTranslate[0] += 0.001f;
        }
        if (Input.GetKey("right"))
        {
            clothTranslate[0] -= 0.001f;
        }
        if (Input.GetKey("z"))
        {
            clothTranslate[2] += 0.001f;
        }
        if (Input.GetKey("c"))
        {
            clothTranslate[2] -= 0.001f;
        }

    }

}
