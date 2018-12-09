using System;
using System.Collections.Generic;
using UnityEngine;

namespace PBFluid
{

    public class GridHash : IDisposable
    {
        //num of threads in a work group
        private const int THREADS = 128;
        //Macros 
        private const int READ = 0;
        private const int WRITE = 1;
        //Total particle number
        public int TotalParticles { get; private set; }
        //boundary of hashgrid
        public Bounds Bounds;
        //dimension of one single grid
        public float CellSize { get; private set; }
        //inverse of cell size
        public float InvCellSize { get; private set; }
        //number of work groups
        public int Groups { get; private set; }

        /// <summary>
        /// Compute buffer: Which cell the particle is in (x) and
        /// the particles index in its position array (y)
        /// </summary>
        public ComputeBuffer IndexMap { get; private set; }

        /// <summary>
        /// Compute Buffer: cell Id mapping to
        /// start and end index of particles (after sorting)
        /// </summary>
        public ComputeBuffer Table { get; private set; }

        private BitonicSort sortEngine;

        private ComputeShader GridHashingShader;

        private int hashKernelID, clearKernelID, mappingKernelID;

        public GridHash(Bounds bounds, int numParticles, float cellSize)
        {
            TotalParticles = numParticles;
            CellSize = cellSize;
            InvCellSize = 1.0f / CellSize;

            Groups = TotalParticles / THREADS;
            if (TotalParticles % THREADS != 0) Groups++;

            Vector3 min, max;
            min = bounds.min;

            max.x = min.x + (float)Math.Ceiling(bounds.size.x / CellSize);
            max.y = min.y + (float)Math.Ceiling(bounds.size.y / CellSize);
            max.z = min.z + (float)Math.Ceiling(bounds.size.z / CellSize);

            Bounds = new Bounds();
            Bounds.SetMinMax(min, max);

            int width = (int)Bounds.size.x;
            int height = (int)Bounds.size.y;
            int depth = (int)Bounds.size.z;

            int size = width * height * depth;

            IndexMap = new ComputeBuffer(TotalParticles, 2 * sizeof(int));
            Table = new ComputeBuffer(size, 2 * sizeof(int));

            sortEngine = new BitonicSort(TotalParticles);

            GridHashingShader = Resources.Load("GridHash") as ComputeShader;
            hashKernelID = GridHashingShader.FindKernel("HashParticles");
            clearKernelID = GridHashingShader.FindKernel("ClearTable");
            mappingKernelID = GridHashingShader.FindKernel("MapTable");
        }

        public Bounds WorldBounds
        {
            get
            {
                Vector3 min = Bounds.min;
                Vector3 max = min + Bounds.size * CellSize;

                Bounds bounds = new Bounds();
                bounds.SetMinMax(min, max);

                return bounds;
            }
        }

        public void Dispose()
        {
            sortEngine.Dispose();

            if (IndexMap != null)
            {
                IndexMap.Release();
                IndexMap = null;
            }

            if (Table != null)
            {
                Table.Release();
                Table = null;
            }
        }

        public void Process(ComputeBuffer particles)
        {
            if (particles.count != TotalParticles)
                throw new ArgumentException("particles.Length != TotalParticles");

            GridHashingShader.SetInt("NumParticles", TotalParticles);
            GridHashingShader.SetInt("TotalParticles", TotalParticles);
            GridHashingShader.SetFloat("HashScale", InvCellSize);
            GridHashingShader.SetVector("HashSize", Bounds.size);
            GridHashingShader.SetVector("HashTranslate", Bounds.min);

            GridHashingShader.SetBuffer(hashKernelID, "Particles", particles);
            GridHashingShader.SetBuffer(hashKernelID, "Boundary", particles);
            GridHashingShader.SetBuffer(hashKernelID, "IndexMap", IndexMap);

            //Assign the particles hash to x and index to y.
            GridHashingShader.Dispatch(hashKernelID, Groups, 1, 1);

            MapTable();
        }

        public void Process(ComputeBuffer particles, ComputeBuffer boundary)
        {
            int numParticles = particles.count;
            int numBoundary = boundary.count;

            if (numParticles + numBoundary != TotalParticles)
                throw new ArgumentException("numParticles + numBoundary != TotalParticles");

            GridHashingShader.SetInt("NumParticles", numParticles);
            GridHashingShader.SetInt("TotalParticles", TotalParticles);
            GridHashingShader.SetFloat("HashScale", InvCellSize);
            GridHashingShader.SetVector("HashSize", Bounds.size);
            GridHashingShader.SetVector("HashTranslate", Bounds.min);

            GridHashingShader.SetBuffer(hashKernelID, "Particles", particles);
            GridHashingShader.SetBuffer(hashKernelID, "Boundary", boundary);
            GridHashingShader.SetBuffer(hashKernelID, "IndexMap", IndexMap);

            //Assign the particles hash to x and index to y.
            GridHashingShader.Dispatch(hashKernelID, Groups, 1, 1);

            MapTable();
        }

        private void MapTable()
        {
            //First sort by the hash values in x.
            //Uses bitonic sort but any other method will work.
            sortEngine.Sort(IndexMap);

            GridHashingShader.SetInt("TotalParticles", TotalParticles);
            GridHashingShader.SetBuffer(clearKernelID, "Table", Table);

            //Clear the previous tables values as not all
            //locations will be written to when mapped.
            GridHashingShader.Dispatch(clearKernelID, Groups, 1, 1);

            GridHashingShader.SetBuffer(mappingKernelID, "IndexMap", IndexMap);
            GridHashingShader.SetBuffer(mappingKernelID, "Table", Table);

            //For each hash cell find where the index map
            //start and ends for that hash value.
            GridHashingShader.Dispatch(mappingKernelID, Groups, 1, 1);
        }
    }
}
