using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utility {

    private static System.Random rng = new System.Random();

    public static float CotTheda(Vector3 v, Vector3 w) {
        float cosTheda = Vector3.Dot(v, w);
        float sinTheda = Vector3.Cross(v, w).magnitude;
        return cosTheda / sinTheda;
    }

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

    public static bool IsPointInCube(Vector3 point, Vector3 extent) {
        return Mathf.Abs(point.x) < extent.x && Mathf.Abs(point.y) < extent.y && Mathf.Abs(point.z) < extent.z;
    }

    public static float RayBoxIntersect(Vector3 rpos, Vector3 rdir, Vector3 vmin, Vector3 vmax) {
        float t1 = (vmin.x - rpos.x) / rdir.x;
        float t2 = (vmax.x - rpos.x) / rdir.x;
        float t3 = (vmin.y - rpos.y) / rdir.y;
        float t4 = (vmax.y - rpos.y) / rdir.y;
        float t5 = (vmin.z - rpos.z) / rdir.z;
        float t6 = (vmax.z - rpos.z) / rdir.z;

        float aMin = t1 < t2 ? t1 : t2;
        float bMin = t3 < t4 ? t3 : t4;
        float cMin = t5 < t6 ? t5 : t6;

        float aMax = t1 > t2 ? t1 : t2;
        float bMax = t3 > t4 ? t3 : t4;
        float cMax = t5 > t6 ? t5 : t6;

        float fMax = aMin > bMin ? aMin : bMin;
        float fMin = aMax < bMax ? aMax : bMax;

        float t7 = fMax > cMin ? fMax : cMin;
        float t8 = fMin < cMax ? fMin : cMax;

        float t9 = (t8 < 0 || t7 > t8) ? -1 : t7;

        return t9;
    }

    public static Mesh DeepCopyMesh(Mesh target) {
        Mesh newMesh = new Mesh();
        newMesh.vertices = target.vertices;
        newMesh.normals = target.normals;
        newMesh.uv = target.uv;
        newMesh.triangles = target.triangles;
        newMesh.tangents = target.tangents;
        return newMesh;
    }
}
