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

    // generates a cloth mesh where each point is vertexDist apart and in the shape of
    // 11 -- 12 -- 13
    // | 1 / | 3 / |
    // |  / 2|  /4 |
    // 0 --- 1 --- 2 --- 3 --- 4 --- 5 --- 6 --- 7 --- 8 --- 9 --- 10
    // width is the number of blocks across. height is the number of blocks down.
    // starting position dictates the starting position of vertex 1
    public static Mesh CreateClothMesh(int rows, int columns) {
        Vector2 vertexDist = new Vector2(10f / columns, 10f / rows);
        Vector3 offset = new Vector3(5f, 0, 5f);
        int numVertices = (rows + 1) * (columns + 1);
        int numTris = rows * columns * 2;
        Vector3[] vertices = new Vector3[numVertices];
        int[] triangles = new int[numTris * 3];
        Mesh newMesh = new Mesh();

        int index = 0;
        for (int i = 0; i <= rows; i++) {
            for (int j = 0; j <= columns; j++) {
                vertices[index++] = new Vector3(-j * vertexDist.x, 0, -i * vertexDist.y) + offset;
            }
        }

        index = 0;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                triangles[index++] = i * (columns + 1) + j;
                triangles[index++] = (i + 1) * (columns + 1) + j;
                triangles[index++] = (i + 1) * (columns + 1) + (j + 1);
                triangles[index++] = i * (columns + 1) + j;
                triangles[index++] = (i + 1) * (columns + 1) + (j + 1);
                triangles[index++] = i * (columns + 1) + (j + 1);
            }
        }

        newMesh.vertices = vertices;
        newMesh.triangles = triangles;
        newMesh.RecalculateBounds();
        newMesh.RecalculateNormals();
        newMesh.RecalculateTangents();
        return newMesh;
    }

    public static Mesh DeepCopyMesh(Mesh mesh) {
        Mesh newMesh = new Mesh();
        newMesh.vertices = mesh.vertices;
        newMesh.triangles = mesh.triangles;
        newMesh.RecalculateBounds();
        newMesh.RecalculateNormals();
        newMesh.RecalculateTangents();
        return newMesh;
    }

    public static Vector2 GetMeshExtent(Mesh mesh, Vector3 scale) {
        float maxX = float.MinValue;
        float maxZ = float.MinValue;
        float minX = float.MaxValue;
        float minZ = float.MaxValue;
        Vector3[] vertices = mesh.vertices;

        for (int i = 0; i < vertices.Length; i++) {
            Vector3 pos = vertices[i];
            maxX = Mathf.Max(maxX, pos.x);
            maxZ = Mathf.Max(maxX, pos.z);
            minX = Mathf.Min(minX, pos.x);
            minZ = Mathf.Min(minZ, pos.z);
        }

        return new Vector2(maxX - minX, maxZ - minZ);
    }

}