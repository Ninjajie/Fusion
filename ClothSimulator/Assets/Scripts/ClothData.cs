using UnityEngine;
using System.Linq;
using System.Collections.Generic;


public class Triangle {
    public int[] vertices;

    public Triangle(int v0, int v1, int v2) {
        vertices = new int[3];
        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;
    }
}

public class Edge {
    public int startIndex;
    public int endIndex;

    public Edge(int start, int end) {
        startIndex = Mathf.Min(start, end);
        endIndex = Mathf.Max(start, end);
    }
}

public class EdgeComparer : EqualityComparer<Edge> {
    public override int GetHashCode(Edge obj) {
        return obj.startIndex * 10000 + obj.endIndex;
    }

    public override bool Equals(Edge x, Edge y) {
        return x.startIndex == y.startIndex && x.endIndex == y.endIndex;
    }
}

public enum DampingMethod { noDamping, simpleDamping, smartDamping }
public enum BendingMethod { noBending, DihedralBending, isometricBending }
public enum PointConstraintType { none, topRow, topCorners, leftRow, leftCorners, custom }