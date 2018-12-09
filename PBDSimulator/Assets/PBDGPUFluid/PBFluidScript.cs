using UnityEngine;
using System;
using System.Collections.Generic;

namespace PBFluid
{

    public enum PARTICLE_AMOUNT { LOW, MEDIUM, HIGH }

    public class PBFluidScript : MonoBehaviour
    {
        //fixed timestep, GUI element
        public float timeStep = 1.0f / 30.0f;
        //particle's material, defined by a GPU instance shader
        public Material fluidParticleMat;
        //define the amount of fluid particles
        public PARTICLE_AMOUNT particleAmount = PARTICLE_AMOUNT.MEDIUM;
        //the bool variable to control the simulation: pause and run
        public bool SystemRun = true;
        //the mesh used for rendering one single particle
        public Mesh renderSphereMesh;
        //the fluid body object
        private FluidBody fusion_FluidBody;
        //the fluid boundary object
        private FluidBoundary fusion_FluidBoundary;
        //the PBD fluid solver object
        private FluidSolver fusion_FluidSolver;
        //the boundaries of the container
        //Bounds fusion_FluidBodySource, fusion_outerSource, fusion_innerSource;
        //bool used for checking system errors
        private bool wasError;
        //the blast created by mouse click
        private Vector3 blastPoint;
        //resize the cube by pressing key
        private Vector3 containerScale;
        //the container's transformation
        public Transform containerTransform;
        //vector3 for the position of the container's center
        private Vector3 containerPos;
        //the particle's radius defining the amount of particles you have in the system
        private float particleRadius;
        //the two public vector3 for the ratio of fluid body(deprecated solution)
        //public Vector3 FluidBodyRatioStart;
        //public Vector3 FluidBodyRatioEnd;
        //the transform that defines the original size of the fluid chuck
        public Transform FluidChunkTransform;
        public Renderer FluidChunkRenderer;

        //the initial velocity of the fluid particles
        public Vector3 FluidInitialVelocity;
        //the number of iterations
        public int DensityComputeIterations = 2;
        public int ConstraintComputeIterations = 2;
        //the thickness of fluid boundary
        public float BoundaryThickness = 1.0f;

        private void Start()
        {
            //define the particle radius here in order to create fluid body particles
            particleRadius = 0.1f;
            float density = 1000.0f;

            blastPoint = new Vector3(0, 10000, 0);
            //containerScale = new Vector3(1, 1, 1);
            containerPos = containerTransform.position;
            containerScale = containerTransform.localScale;

            Vector3 fluidBodyPos = FluidChunkTransform.position;
            Vector3 fluidBodyScale = FluidChunkTransform.localScale;

            //NOTE1: the particle amount may raise issues in Bitonic Sorting

            //NOTE2: A smaller radius may also requre more solver steps 

            switch (particleAmount)
            {
                case PARTICLE_AMOUNT.LOW:
                    particleRadius = 0.1f;
                    break;

                case PARTICLE_AMOUNT.MEDIUM:
                    particleRadius = 0.08f;
                    break;

                case PARTICLE_AMOUNT.HIGH:
                    particleRadius = 0.06f;
                    break;
            }

            try
            {
                CreateBoundary(particleRadius, density, containerPos, containerScale);
                CreateFluid(particleRadius, density, containerPos, containerScale, fluidBodyPos, fluidBodyScale);

                fusion_FluidBody.Bounds = fusion_FluidBoundary.Bounds;

                fusion_FluidSolver = new FluidSolver(fusion_FluidBody, fusion_FluidBoundary, DensityComputeIterations, ConstraintComputeIterations);

                //fusion_volume = new RenderVolume(fusion_FluidBoundary.Bounds, radius);
                //fusion_volume.CreateMesh(fusion_volumeMat);
            }
            catch
            {
                wasError = true;
                throw;
            }
            FluidChunkRenderer.enabled = false;
        }

        private void Update()
        {
            SystemStateOnKeyPresses();
            

            if (wasError) return;

            if (SystemRun)
            {
                if (Input.GetMouseButtonDown(0)) // if left button is pressed down, shoot ray
                {
                    blastPoint = GetMousePosition();
                }
                else
                {
                    blastPoint = new Vector3(0, 10000, 0);
                }

                fusion_FluidSolver.StepPhysics(timeStep, blastPoint);
               
            }
            //draw particles using GPU instancing
            //the draw function is defined in fluidbody class
            fusion_FluidBody.Draw(Camera.main, renderSphereMesh, fluidParticleMat, 0);
        }
        //dispose the objects
        private void OnDestroy()
        {
            fusion_FluidBoundary.Dispose();
            fusion_FluidBody.Dispose();
            fusion_FluidSolver.Dispose();
        }
        //need to get the current camera in order to render 
        //the fluid objects
        private void OnRenderObject()
        {
            Camera camera = Camera.current;
            if (camera != Camera.main) return;
        }
        //given a cube region, generate boundary particles around it
        private void CreateBoundary(float radius, float density, Vector3 containerPos, Vector3 resizeFactor)
        {
            //innerBounds defines the region that fluid particles could move 
            Bounds innerBounds = new Bounds();
            //create the fluid boundary according to the position and size of the container 
            //the information about the container is passed in via the Transformation
            Vector3 min = new Vector3(containerPos[0] -0.5f * resizeFactor[0], containerPos[1] - 0.5f * resizeFactor[1], containerPos[2] - 0.5f * resizeFactor[2]);
            Vector3 max = new Vector3(containerPos[0] + 0.5f * resizeFactor[0], containerPos[1] + 0.5f * resizeFactor[1], containerPos[2] + 0.5f * resizeFactor[2]);
            innerBounds.SetMinMax(min, max);

            //Make the boundary 1 particle thick.
            //The multiple by 1.2 adds a little of extra
            //thickness incase the radius does not evenly
            //divide into the bounds size. You might have
            //particles missing from one side of the source
            //bounds other wise.
            float BoundaryThickness = 1;
            float diameter = radius * 2;
            min.x -= diameter * BoundaryThickness * 1.2f;
            min.y -= diameter * BoundaryThickness * 1.2f;
            min.z -= diameter * BoundaryThickness * 1.2f;

            max.x += diameter * BoundaryThickness * 1.2f;
            max.y += diameter * BoundaryThickness * 1.2f;
            max.z += diameter * BoundaryThickness * 1.2f;
            //outerBounds is the outmost bound of all particles
            //A.K.A the boundary of the entire simulation
            Bounds outerBounds = new Bounds();
            outerBounds.SetMinMax(min, max);

            //The source will create a array of particles
            //evenly spaced between the inner and outer bounds.
            ParticleSource source = new ParticlesFromBounds(diameter, outerBounds, innerBounds);
            //print out the number of particles
            Debug.Log("Boundary Particles = " + source.NumParticles);
            //given the particle positions contained in "source" object
            //create the fluid boundary object
            fusion_FluidBoundary = new FluidBoundary(source, radius, density, Matrix4x4.identity);
            //pass bounds objects
            //fusion_innerSource = innerBounds;
            //fusion_outerSource = outerBounds;
        }
        //given a cube region, create a fluid body 
        //the fluid body's size is defined relative to the size of the container
        private void CreateFluid(float radius, float density, Vector3 containerPos, Vector3 resizeFactor,
            Vector3 fluidBodyPos, Vector3 fluidBodyScale)
        {
            //the bounds of the (initial) fluid region
            Bounds bounds = new Bounds();
            //Vector3 min = new Vector3(-8, 0, -1);
            //Vector3 max = new Vector3(0, 8, 1);

            Bounds fluidChunkBound = new Bounds(fluidBodyPos, fluidBodyScale);
            // Vector3 min = new Vector3(fluidBodyPos[0] - 0.5f * fluidBodyScale[0], fluidBodyPos[1] - 0.5f * fluidBodyScale[1], fluidBodyPos[2] - 0.5f * fluidBodyScale[2]);
            //  Vector3 max = new Vector3(fluidBodyPos[0] + 0.5f * fluidBodyScale[0], fluidBodyPos[1] + 0.5f * fluidBodyScale[1], fluidBodyPos[2] + 0.5f * fluidBodyScale[2]);
            Vector3 min = fluidChunkBound.min;
            Vector3 max = fluidChunkBound.max;

            //create the fluid body according to the size and position of the container
            //Vector3 ContainerMin = new Vector3(containerPos[0] - 0.5f * resizeFactor[0], containerPos[1] - 0.4f * resizeFactor[1], containerPos[2] - 0.25f * resizeFactor[2]);
            //Vector3 ContainerMax = new Vector3(containerPos[0] + 0.02f * resizeFactor[0], containerPos[1] + 0.4f * resizeFactor[1], containerPos[2] + 0.25f * resizeFactor[2]);
            //need to minus/plus a radius since the particles are defined as spheres
            min.x += radius;
            min.y += radius;
            min.z += radius;

            max.x -= radius;
            max.y -= radius;
            max.z -= radius;
            //set the bound
            bounds.SetMinMax(min, max);

            //The source will create a array of particles evenly spaced inside the bounds. 
            //Multiple the spacing by 0.9 to pack more particles into bounds.
            //create particles from the bound
            float diameter = radius * 2;
            ParticlesFromBounds source = new ParticlesFromBounds(diameter * 0.9f, bounds);
            Debug.Log("Fluid Particles = " + source.NumParticles);
            //create a new fluid body object given the particles contained in "source"
            fusion_FluidBody = new FluidBody(source, radius, density, Matrix4x4.identity, FluidInitialVelocity);

           // fusion_FluidBodySource = bounds;
        }


        private Vector3 GetMousePosition()
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit = new RaycastHit();
            if (Physics.Raycast(ray, out hit))
                return hit.point;
            return new Vector3(0,10000,0);
        }

        private void SystemStateOnKeyPresses()
        {
            //press the Space key, system got paused
            //At paused state, the fusion_run variable should be set to false
            //until the Enter key is pressed, then fusion_run variable get back to true
            //then the fluid boundary is reconstructed and simulation resumes
            if (Input.GetKeyDown(KeyCode.Space) && SystemRun == true) 
            {
                SystemRun = false;
                Debug.Log("space pressed");
            }
            //if the system is paused and you press Space
            //the fluidboundary object need to be reconstructed
            //then system resumes
            if (Input.GetKeyDown(KeyCode.B) && SystemRun == false) 
            {
                
               
                float density = 1000.0f;
                try
                {
                    CreateBoundary(particleRadius, density, containerPos, containerScale);
                    fusion_FluidBody.Bounds = fusion_FluidBoundary.Bounds;

                    fusion_FluidSolver = new FluidSolver(fusion_FluidBody, fusion_FluidBoundary, DensityComputeIterations, ConstraintComputeIterations);
                }
                catch
                {
                    wasError = true;
                    throw;
                }

                if (wasError) return;

                //fusion_FluidSolver.UpdateBoundary(fusion_FluidBoundary);

                SystemRun = true;
            }

            if (SystemRun == false) //when system paused, move the wall of the cube
            {
                if (Input.GetKey(KeyCode.L))
                {
                    containerScale[0] += 0.05f;
                }
                if (Input.GetKey(KeyCode.J))
                {
                    containerScale[0] -= 0.05f;
                }
                if (Input.GetKey(KeyCode.I))
                {
                    containerScale[1] -= 0.05f;
                }
                if (Input.GetKey(KeyCode.K))
                {
                    containerScale[1] += 0.05f;
                }
                if (Input.GetKey(KeyCode.U))
                {
                    containerScale[2] -= 0.05f;
                }
                if (Input.GetKey(KeyCode.O))
                {
                    containerScale[2] += 0.05f;
                }

                containerTransform.localScale = containerScale;
            }
        }

    }

}
