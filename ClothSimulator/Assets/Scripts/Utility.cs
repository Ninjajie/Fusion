using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utility {

    private static System.Random rng = new System.Random();

    public static void Shuffle<T>(this IList<T> list) {
        int n = list.Count;
        while (n > 1) {
            n--;
            int k = rng.Next(n + 1);
            T value = list[k];
            list[k] = list[n];
            list[n] = value;
        }
    }

    public static Matrix4x4 ScaleMatrixByFloat(Matrix4x4 m, float f) {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 16; i++) {
            result[i] = m[i] * f;
        }
        return result;
    }

    public static Matrix4x4 AddMatrices(Matrix4x4 lhs, Matrix4x4 rhs) {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 16; i++) {
            result[i] = lhs[i] + rhs[i];
        }
        return result;
    }
}
