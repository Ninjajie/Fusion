using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace PBFluid
{

    public class FluidBody : IDisposable
    {
        //the number of particles the fluid contains
        public int NumParticles { get; private set; }
        //the boundary of the fluid
        public Bounds Bounds;
        //Density constant
        public float Density { get; set; }
        //Viscosity constant
        public float ViscosityCoeff { get; set; }
        //velocity damping coefficient
        public float DampingCoeff { get; set; }
        //particle radius
        public float ParticleRadius { get; private set; }
        //diameter
        public float ParticleDiameter { get { return ParticleRadius * 2.0f; } }
        //mass
        public float ParticleMass { get; set; }
        //particle volume. treat particle as sphere to compute the volume. Used for mass computing
        public float ParticleVolume { get; private set; }
        //Compute buffers as names suggest
        public ComputeBuffer PressuresBuffer { get; private set; }
        public ComputeBuffer DensitiesBuffer { get; private set; }
        public ComputeBuffer PositionsBuffer { get; private set; }
        //predictedpositions and velocities are READ/WRITE, for swap
        public ComputeBuffer[] PredictedBuffer { get; private set; }
        public ComputeBuffer[] VelocitiesBuffer { get; private set; }
        //argument buffer for GPU instancing
        private ComputeBuffer m_argsBuffer;

        public FluidBody(ParticleSource source, float radius, float density, Matrix4x4 RTS, Vector3 initialVel)
        {
            NumParticles = source.NumParticles;
            Density = density;
            ViscosityCoeff = 0.002f;
            DampingCoeff = 0.0f;

            ParticleRadius = radius;
            ParticleVolume = (4.0f / 3.0f) * Mathf.PI * Mathf.Pow(radius, 3);
            ParticleMass = ParticleVolume * Density;

            DensitiesBuffer = new ComputeBuffer(NumParticles, sizeof(float));
            PressuresBuffer = new ComputeBuffer(NumParticles, sizeof(float));

            CreateParticles(source, RTS, initialVel);
        }

        /// <summary>
        /// Draws the mesh spheres when draw particles is enabled.
        /// </summary>
        public void Draw(Camera cam, Mesh mesh, Material material, int layer)
        {
            if (m_argsBuffer == null)
                CreateArgBuffer(mesh.GetIndexCount(0));

            Color waterColor = new Color(0.238f, 0.7f, 0.7f);
            material.SetBuffer("positions", PositionsBuffer);
            material.SetColor("color", waterColor);
            material.SetFloat("diameter", ParticleDiameter);

            ShadowCastingMode castShadow = ShadowCastingMode.On;
            bool recieveShadow = true;

            Graphics.DrawMeshInstancedIndirect(mesh, 0, material, Bounds, m_argsBuffer, 0, null, castShadow, recieveShadow, layer, cam);

        }

        public void Dispose()
        {

            if (PositionsBuffer != null)
            {
                PositionsBuffer.Release();
                PositionsBuffer = null;
            }

            if (DensitiesBuffer != null)
            {
                DensitiesBuffer.Release();
                DensitiesBuffer = null;
            }

            if (PressuresBuffer != null)
            {
                PressuresBuffer.Release();
                PressuresBuffer = null;
            }

            FusionUtilities.Release(PredictedBuffer);
            FusionUtilities.Release(VelocitiesBuffer);
            FusionUtilities.Release(ref m_argsBuffer);
        }

        private void CreateParticles(ParticleSource source, Matrix4x4 RTS, Vector3 initialVel)
        {
            Vector4[] positions = new Vector4[NumParticles];
            Vector4[] predicted = new Vector4[NumParticles];
            Vector4[] velocities = new Vector4[NumParticles];

            float inf = float.PositiveInfinity;
            Vector3 min = new Vector3(inf, inf, inf);
            Vector3 max = new Vector3(-inf, -inf, -inf);

            for (int i = 0; i < NumParticles; i++)
            {
                Vector4 pos = RTS * source.Positions[i];
                positions[i] = pos;
                predicted[i] = pos;

                //modified: add initial velocity to the particles
                //velocities[i] = new Vector3(5.0f, 0.0f, 0.0f);
                velocities[i] = initialVel;

                if (pos.x < min.x) min.x = pos.x;
                if (pos.y < min.y) min.y = pos.y;
                if (pos.z < min.z) min.z = pos.z;

                if (pos.x > max.x) max.x = pos.x;
                if (pos.y > max.y) max.y = pos.y;
                if (pos.z > max.z) max.z = pos.z;
            }

            min.x -= ParticleRadius;
            min.y -= ParticleRadius;
            min.z -= ParticleRadius;

            max.x += ParticleRadius;
            max.y += ParticleRadius;
            max.z += ParticleRadius;

            Bounds = new Bounds();
            Bounds.SetMinMax(min, max);

            PositionsBuffer = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            PositionsBuffer.SetData(positions);

            //Predicted and velocities use a double buffer as solver step
            //needs to read from many locations of buffer and write the result
            //in same pass. Could be removed if needed as long as buffer writes 
            //are atomic. Not sure if they are.

            PredictedBuffer = new ComputeBuffer[2];
            PredictedBuffer[0] = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            PredictedBuffer[0].SetData(predicted);
            PredictedBuffer[1] = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            PredictedBuffer[1].SetData(predicted);

            VelocitiesBuffer = new ComputeBuffer[2];
            VelocitiesBuffer[0] = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            VelocitiesBuffer[0].SetData(velocities);
            VelocitiesBuffer[1] = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            VelocitiesBuffer[1].SetData(velocities);
        }

        private void CreateArgBuffer(uint indexCount)
        {
            uint[] args = new uint[5] { 0, 0, 0, 0, 0 };
            args[0] = indexCount;
            args[1] = (uint)NumParticles;

            m_argsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            m_argsBuffer.SetData(args);
        }

    }
     

}