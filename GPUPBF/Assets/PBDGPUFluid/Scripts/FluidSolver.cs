using System;
using System.Collections.Generic;
using UnityEngine;

namespace PBFluid
{

    public class FluidSolver : IDisposable
    {
        //group size
        private const int THREADS = 128;
        //Macros
        private const int READ = 0;
        private const int WRITE = 1;
        //number of groups
        public int Groups { get; private set; }
        //the fluid boundary object
        public FluidBoundary Boundary { get; private set; }
        //the fluid body object
        public FluidBody Body { get; private set; }
        //the hashgrid object
        public GridHash Hash { get; private set; }
        //the number of iterations
        public int DensityComputeIterations { get; set; }
        //the number of constraint iterations
        public int ConstraintComputeIterations { get; set; }
        //the smoothing Kernel object, contains the mathematical calculations of various Kernels
        public SmoothingKernel Kernel { get; private set; }
        //the handle to the compute shader to be run
        private ComputeShader fluidSolverShader;


        public FluidSolver(FluidBody body, FluidBoundary boundary, int densityComputeIteration, int constraintComputeIteration)
        {
            DensityComputeIterations = densityComputeIteration;
            ConstraintComputeIterations = constraintComputeIteration;

            Body = body;
            Boundary = boundary;

            float cellSize = Body.ParticleRadius * 4.0f;
            int total = Body.NumParticles + Boundary.NumParticles;
            Hash = new GridHash(Boundary.Bounds, total, cellSize);
            Kernel = new SmoothingKernel(cellSize);

            int numParticles = Body.NumParticles;
            Groups = numParticles / THREADS;
            if (numParticles % THREADS != 0) Groups++;

            fluidSolverShader = Resources.Load("FluidSolver") as ComputeShader;


        }

        public void UpdateBoundary(FluidBoundary newBoundary)
        {
            Boundary = newBoundary;
            float cellSize = Body.ParticleRadius * 4.0f;
            int total = Body.NumParticles + Boundary.NumParticles;
            Hash.Dispose();
            Hash = new GridHash(Boundary.Bounds, total, cellSize);
        }

        public void Dispose()
        {
            Hash.Dispose();
        }

        public void StepPhysics(float dt, Vector3 blastPoint)
        {

            if (dt <= 0.0) return;
            if (DensityComputeIterations <= 0 || ConstraintComputeIterations <= 0) return;

            dt /= DensityComputeIterations;

            fluidSolverShader.SetInt("NumParticles", Body.NumParticles);
            fluidSolverShader.SetVector("Gravity", new Vector3(0.0f, -9.81f, 0.0f));
            fluidSolverShader.SetFloat("Dampning", Body.DampingCoeff);
            fluidSolverShader.SetFloat("DeltaTime", dt);
            fluidSolverShader.SetFloat("Density", Body.Density);
            fluidSolverShader.SetFloat("Viscosity", Body.ViscosityCoeff);
            fluidSolverShader.SetFloat("ParticleMass", Body.ParticleMass);

            fluidSolverShader.SetFloat("KernelRadius", Kernel.Radius);
            fluidSolverShader.SetFloat("KernelRadius2", Kernel.Radius2);
            fluidSolverShader.SetFloat("Poly6Zero", Kernel.Poly6(Vector3.zero));
            fluidSolverShader.SetFloat("Poly6", Kernel.POLY6);
            fluidSolverShader.SetFloat("SpikyGrad", Kernel.SPIKY_GRAD);
            fluidSolverShader.SetFloat("ViscLap", Kernel.VISC_LAP);

            fluidSolverShader.SetFloat("HashScale", Hash.InvCellSize);
            fluidSolverShader.SetVector("HashSize", Hash.Bounds.size);
            fluidSolverShader.SetVector("HashTranslate", Hash.Bounds.min);

            fluidSolverShader.SetVector("BoundMin", Boundary.Bounds.min);
            fluidSolverShader.SetVector("BoundMax", Boundary.Bounds.max);

            //Predicted and velocities use a double buffer as solver step
            //needs to read from many locations of buffer and write the result
            //in same pass. Could be removed if needed as long as buffer writes 
            //are atomic. Not sure if they are.

            for (int i = 0; i < DensityComputeIterations; i++)
            {
                PredictPositions(dt, blastPoint);

                Hash.Process(Body.PredictedBuffer[READ], Boundary.PositionsBuffer);

                ConstrainPositions();

                UpdateVelocities(dt);

                SolveViscosity();

                UpdatePositions();
            }

        }

        private void PredictPositions(float dt, Vector3 blastPoint)
        {
            int kernel = fluidSolverShader.FindKernel("PredictPositions");

            fluidSolverShader.SetVector("blastPoint", blastPoint);

            fluidSolverShader.SetBuffer(kernel, "Positions", Body.PositionsBuffer);
            fluidSolverShader.SetBuffer(kernel, "PredictedWRITE", Body.PredictedBuffer[WRITE]);
            fluidSolverShader.SetBuffer(kernel, "VelocitiesREAD", Body.VelocitiesBuffer[READ]);
            fluidSolverShader.SetBuffer(kernel, "VelocitiesWRITE", Body.VelocitiesBuffer[WRITE]);

            fluidSolverShader.Dispatch(kernel, Groups, 1, 1);

            Swap(Body.PredictedBuffer);
            Swap(Body.VelocitiesBuffer);
        }

        public void ConstrainPositions()
        {

            int computeDensityKernelID = fluidSolverShader.FindKernel("ComputeDensity");
            int solveKernel = fluidSolverShader.FindKernel("SolveConstraint");

            fluidSolverShader.SetBuffer(computeDensityKernelID, "Densities", Body.DensitiesBuffer);
            fluidSolverShader.SetBuffer(computeDensityKernelID, "Pressures", Body.PressuresBuffer);
            fluidSolverShader.SetBuffer(computeDensityKernelID, "Boundary", Boundary.PositionsBuffer);
            fluidSolverShader.SetBuffer(computeDensityKernelID, "IndexMap", Hash.IndexMap);
            fluidSolverShader.SetBuffer(computeDensityKernelID, "Table", Hash.Table);

            fluidSolverShader.SetBuffer(solveKernel, "Pressures", Body.PressuresBuffer);
            fluidSolverShader.SetBuffer(solveKernel, "Boundary", Boundary.PositionsBuffer);
            fluidSolverShader.SetBuffer(solveKernel, "IndexMap", Hash.IndexMap);
            fluidSolverShader.SetBuffer(solveKernel, "Table", Hash.Table);

            for (int i = 0; i < ConstraintComputeIterations; i++)
            {
                fluidSolverShader.SetBuffer(computeDensityKernelID, "PredictedREAD", Body.PredictedBuffer[READ]);
                fluidSolverShader.Dispatch(computeDensityKernelID, Groups, 1, 1);

                fluidSolverShader.SetBuffer(solveKernel, "PredictedREAD", Body.PredictedBuffer[READ]);
                fluidSolverShader.SetBuffer(solveKernel, "PredictedWRITE", Body.PredictedBuffer[WRITE]);
                fluidSolverShader.Dispatch(solveKernel, Groups, 1, 1);

                Swap(Body.PredictedBuffer);
            }
        }

        private void UpdateVelocities(float dt)
        {
            int kernel = fluidSolverShader.FindKernel("UpdateVelocities");

            fluidSolverShader.SetBuffer(kernel, "Positions", Body.PositionsBuffer);
            fluidSolverShader.SetBuffer(kernel, "PredictedREAD", Body.PredictedBuffer[READ]);
            fluidSolverShader.SetBuffer(kernel, "VelocitiesWRITE", Body.VelocitiesBuffer[WRITE]);

            fluidSolverShader.Dispatch(kernel, Groups, 1, 1);

            Swap(Body.VelocitiesBuffer);
        }

        private void SolveViscosity()
        {
            int kernel = fluidSolverShader.FindKernel("SolveViscosity");

            fluidSolverShader.SetBuffer(kernel, "Densities", Body.DensitiesBuffer);
            fluidSolverShader.SetBuffer(kernel, "Boundary", Boundary.PositionsBuffer);
            fluidSolverShader.SetBuffer(kernel, "IndexMap", Hash.IndexMap);
            fluidSolverShader.SetBuffer(kernel, "Table", Hash.Table);

            fluidSolverShader.SetBuffer(kernel, "PredictedREAD", Body.PredictedBuffer[READ]);
            fluidSolverShader.SetBuffer(kernel, "VelocitiesREAD", Body.VelocitiesBuffer[READ]);
            fluidSolverShader.SetBuffer(kernel, "VelocitiesWRITE", Body.VelocitiesBuffer[WRITE]);

            fluidSolverShader.Dispatch(kernel, Groups, 1, 1);

            Swap(Body.VelocitiesBuffer);
        }

        private void UpdatePositions()
        {
            int kernel = fluidSolverShader.FindKernel("UpdatePositions");

            fluidSolverShader.SetBuffer(kernel, "Positions", Body.PositionsBuffer);
            fluidSolverShader.SetBuffer(kernel, "PredictedREAD", Body.PredictedBuffer[READ]);

            fluidSolverShader.Dispatch(kernel, Groups, 1, 1);
        }

        private void Swap(ComputeBuffer[] buffers)
        {
            ComputeBuffer tmp = buffers[0];
            buffers[0] = buffers[1];
            buffers[1] = tmp;
        }
    }

}