using System;
using System.Collections.Generic;
using UnityEngine;

namespace PBFluid
{

    public class BitonicSort : IDisposable
    {
        //Num threads for the copy and fill kernels.
        private const int THREADS = 128;
        //size of the work group
        private const int BITONIC_BLOCK_SIZE = 512;
        private const int TRANSPOSE_BLOCK_SIZE = 16;
        //the range of element amount that the sorting algorithm can handle
        public const int MAX_ELEMENTS = BITONIC_BLOCK_SIZE * BITONIC_BLOCK_SIZE;
        public const int MIN_ELEMENTS = BITONIC_BLOCK_SIZE * TRANSPOSE_BLOCK_SIZE;

        private const int MATRIX_WIDTH = BITONIC_BLOCK_SIZE;
        //number of elements
        public int NumElements { get; private set; }
        //compute buffers to store the temporary data
        private ComputeBuffer tempBuffer1, tempBuffer2;
        //compute shader handle
        private ComputeShader BitonicSortShader;
        //kernel IDs
        int bitonicKernelID, transposeKernelID;
        int fillKernelID, copyKernelID;

        public BitonicSort(int count)
        {
            NumElements = FindNumElements(count);
            tempBuffer1 = new ComputeBuffer(NumElements, 2 * sizeof(int));
            tempBuffer2 = new ComputeBuffer(NumElements, 2 * sizeof(int));

            BitonicSortShader = Resources.Load("BitonicSort") as ComputeShader;
            bitonicKernelID = BitonicSortShader.FindKernel("BitonicSort");
            transposeKernelID = BitonicSortShader.FindKernel("MatrixTranspose");
            fillKernelID = BitonicSortShader.FindKernel("Fill");
            copyKernelID = BitonicSortShader.FindKernel("Copy");
        }

        public void Dispose()
        {
            FusionUtilities.Release(ref tempBuffer1);
            FusionUtilities.Release(ref tempBuffer2);
        }

        public void Sort(ComputeBuffer input)
        {

            int count = input.count;
            if (count < MIN_ELEMENTS)
                throw new ArgumentException("count < MIN_ELEMENTS");

            if (count > NumElements)
                throw new ArgumentException("count > NumElements");

            BitonicSortShader.SetInt("Width", count);
            BitonicSortShader.SetBuffer(fillKernelID, "Input", input);
            BitonicSortShader.SetBuffer(fillKernelID, "Data", tempBuffer1);
            BitonicSortShader.Dispatch(fillKernelID, NumElements / THREADS, 1, 1);

            int MATRIX_HEIGHT = NumElements / BITONIC_BLOCK_SIZE;

            BitonicSortShader.SetInt("Width", MATRIX_HEIGHT);
            BitonicSortShader.SetInt("Height", MATRIX_WIDTH);
            BitonicSortShader.SetBuffer(bitonicKernelID, "Data", tempBuffer1);

            // Sort the data
            // First sort the rows for the levels <= to the block size
            for (int level = 2; level <= BITONIC_BLOCK_SIZE; level = level * 2)
            {
                // Sort the row data
                BitonicSortShader.SetInt("Level", level);
                BitonicSortShader.SetInt("LevelMask", level);
                BitonicSortShader.Dispatch(bitonicKernelID, NumElements / BITONIC_BLOCK_SIZE, 1, 1);
            }

            // Then sort the rows and columns for the levels > than the block size
            // Transpose. Sort the Columns. Transpose. Sort the Rows.
            for (int level = (BITONIC_BLOCK_SIZE * 2); level <= NumElements; level = level * 2)
            {
                // Transpose the data from buffer 1 into buffer 2
                BitonicSortShader.SetInt("Level", level / BITONIC_BLOCK_SIZE);
                BitonicSortShader.SetInt("LevelMask", (level & ~NumElements) / BITONIC_BLOCK_SIZE);
                BitonicSortShader.SetInt("Width", MATRIX_WIDTH);
                BitonicSortShader.SetInt("Height", MATRIX_HEIGHT);
                BitonicSortShader.SetBuffer(transposeKernelID, "Input", tempBuffer1);
                BitonicSortShader.SetBuffer(transposeKernelID, "Data", tempBuffer2);
                BitonicSortShader.Dispatch(transposeKernelID, MATRIX_WIDTH / TRANSPOSE_BLOCK_SIZE, MATRIX_HEIGHT / TRANSPOSE_BLOCK_SIZE, 1);

                // Sort the transposed column data
                BitonicSortShader.SetBuffer(bitonicKernelID, "Data", tempBuffer2);
                BitonicSortShader.Dispatch(bitonicKernelID, NumElements / BITONIC_BLOCK_SIZE, 1, 1);

                // Transpose the data from buffer 2 back into buffer 1
                BitonicSortShader.SetInt("Level", BITONIC_BLOCK_SIZE);
                BitonicSortShader.SetInt("LevelMask", level);
                BitonicSortShader.SetInt("Width", MATRIX_HEIGHT);
                BitonicSortShader.SetInt("Height", MATRIX_WIDTH);
                BitonicSortShader.SetBuffer(transposeKernelID, "Input", tempBuffer2);
                BitonicSortShader.SetBuffer(transposeKernelID, "Data", tempBuffer1);
                BitonicSortShader.Dispatch(transposeKernelID, MATRIX_HEIGHT / TRANSPOSE_BLOCK_SIZE, MATRIX_WIDTH / TRANSPOSE_BLOCK_SIZE, 1);

                // Sort the row data
                BitonicSortShader.SetBuffer(bitonicKernelID, "Data", tempBuffer1);
                BitonicSortShader.Dispatch(bitonicKernelID, NumElements / BITONIC_BLOCK_SIZE, 1, 1);
            }

            BitonicSortShader.SetInt("Width", count);
            BitonicSortShader.SetBuffer(copyKernelID, "Input", tempBuffer1);
            BitonicSortShader.SetBuffer(copyKernelID, "Data", input);
            BitonicSortShader.Dispatch(copyKernelID, NumElements / THREADS, 1, 1);
        }

        private int FindNumElements(int count)
        {
            if (count < MIN_ELEMENTS)
                throw new ArgumentException("Data != MIN_ELEMENTS. Need to decrease Bitonic size.");

            if (count > MAX_ELEMENTS)
                throw new ArgumentException("Data > MAX_ELEMENTS. Need to increase Bitonic size");

            int NumElements;

            int level = TRANSPOSE_BLOCK_SIZE;
            do
            {
                NumElements = BITONIC_BLOCK_SIZE * level;
                level *= 2;
            }
            while (NumElements < count);

            return NumElements;
        }

    }

}
