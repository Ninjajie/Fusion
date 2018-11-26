using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Constraint {

    public abstract void Satisfy(Vector3[] projectedPositions, float mass);
}

public class DistanceConstraint : Constraint {
    private Edge edge;
    private float restLength;
    private float compressionStiffness, stretchStiffness;

    public DistanceConstraint(Edge e, Vector3[] positions, float compressionStiffness, float stretchStiffness) {
        edge = e;
        this.compressionStiffness = compressionStiffness;
        this.stretchStiffness = stretchStiffness;

        Vector3 startPos = positions[e.startIndex];
        Vector3 endPos = positions[e.endIndex];
        restLength = (startPos - endPos).magnitude;
    }

    public override void Satisfy(Vector3[] projectedPositions, float mass) {
        //get positions
        Vector3 pi = projectedPositions[edge.startIndex];
        Vector3 pj = projectedPositions[edge.endIndex];

        //make edge vector
        Vector3 n = pi - pj;

        //get current length
        float d = n.magnitude;

        //normalize edge vector
        n.Normalize();

        float wi = mass;
        float wj = mass;

        Vector3 corr = n * (d - restLength) / (wi + wj);
        corr *= d < restLength ? compressionStiffness : stretchStiffness;

        //projectedPositions[edge.startIndex] = pi + wi * corr;
        //projectedPositions[edge.endIndex] = pi - wi * corr;
        projectedPositions[edge.startIndex] = pi - compressionStiffness * wi
                                              / (wi + wj) * (d - restLength) * n;
        projectedPositions[edge.endIndex] = pj + compressionStiffness * wj
                                              / (wi + wj) * (d - restLength) * n;
    }
}

public class BendingConstraint : Constraint {
    /* index of each point in projectedPositions and masses
     * this is indexed like the Bridson, Simulation of Clothing with Folds
     *     and Wrinkles paper
     *    3
     *    ^
     * 0  |  1
     *    2
     */
    int[] vertexIndeces;
    float restAngle;
    float stiffness;

    public BendingConstraint(int[] indices, Vector3[] positions, float stiffness) {
        vertexIndeces = indices;
        this.stiffness = stiffness;

        Vector3 p0 = positions[indices[0]];
        Vector3 p1 = positions[indices[1]];
        Vector3 p2 = positions[indices[2]]; // TODO: does the way the edge is wound matter?
        Vector3 p3 = positions[indices[3]];

        Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
        Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;

        float d = Vector3.Dot(n1, n2);
        d = Mathf.Clamp(d, -1.0f, 1.0f);
        restAngle = Mathf.Acos(d);
    }

    public override void Satisfy(Vector3[] projectedPositions, float mass) {
        /* this is indexed like the Bridson, Simulation of Clothing with Folds
     *     and Wrinkles paper
     *    3
     *    ^
     * 0  |  1
     *    2 
     */

        Vector3 p0 = projectedPositions[vertexIndeces[0]];
        Vector3 p1 = projectedPositions[vertexIndeces[1]];
        Vector3 p2 = projectedPositions[vertexIndeces[2]];
        Vector3 p3 = projectedPositions[vertexIndeces[3]];

        Vector3 wing = p3 - p2;
        float wingLength = wing.magnitude;

        if (wingLength >= 1e-7) {
            //get normals
            Vector3 n1 = Vector3.Cross(p2 - p0, p3 - p0);
            n1 /= n1.sqrMagnitude;

            Vector3 n2 = Vector3.Cross(p3 - p1, p2 - p1);
            n2 /= n2.sqrMagnitude;
            //unlike in the original PBD paper,
            // both normals point in same direction

            //getting constraints along gradients (gradpC)
            float invWingLength = 1.0f / wingLength;

            Vector3 q0 = wingLength * n1;
            Vector3 q1 = wingLength * n2;
            Vector3 q2 = Vector3.Dot(p0 - p3, wing) * invWingLength * n1
                        + Vector3.Dot(p1 - p3, wing) * invWingLength * n2;
            Vector3 q3 = Vector3.Dot(p2 - p0, wing) * invWingLength * n1
                        + Vector3.Dot(p2 - p1, wing) * invWingLength * n2;

            //find current angle
            n1.Normalize();
            n2.Normalize();

            float d = Vector3.Dot(n1, n2);
            d = Mathf.Clamp(d, -1.0f, 1.0f);
            float currentAngle = Mathf.Acos(d);

            //find lamda ( where deltap = lamda*wi*gradConstraint )
            float lamda = 0;
            lamda += mass * q0.sqrMagnitude;
            lamda += mass * q1.sqrMagnitude;
            lamda += mass * q2.sqrMagnitude;
            lamda += mass * q3.sqrMagnitude;

            if (lamda != 0.0f) {
                lamda = (currentAngle - restAngle) / lamda * stiffness;

                if (Vector3.Dot(Vector3.Cross(n1, n2), wing) > 0.0) {
                    lamda = -lamda;
                }

                projectedPositions[vertexIndeces[0]] -= mass * lamda * q0;
                projectedPositions[vertexIndeces[1]] -= mass * lamda * q1;
                projectedPositions[vertexIndeces[2]] -= mass * lamda * q2;
                projectedPositions[vertexIndeces[3]] -= mass * lamda * q3;
            }
        }
    }
}

public class IsometricBendingConstraint : Constraint {
    int[] vertexIndeces;
    float restAngle;
    float stiffness;
    Matrix4x4 Q;

    public IsometricBendingConstraint(int[] indices, Vector3[] positions, float stiffness) {
        vertexIndeces = indices;
        this.stiffness = stiffness;
        Q = Matrix4x4.zero;

        Vector3 x0 = positions[indices[2]];
        Vector3 x1 = positions[indices[3]];
        Vector3 x2 = positions[indices[0]];
        Vector3 x3 = positions[indices[1]];

        Vector3 e0 = x1 - x0;
        Vector3 e1 = x2 - x0;
        Vector3 e2 = x3 - x0;
        Vector3 e3 = x2 - x1;
        Vector3 e4 = x3 - x1;

        float c01 = Utility.CotTheda(e0, e1);
        float c02 = Utility.CotTheda(e0, e2);
        float c03 = Utility.CotTheda(-e0, e3);
        float c04 = Utility.CotTheda(-e0, e4);

        float A0 = 0.5f * Vector3.Cross(e0, e1).magnitude;
        float A1 = 0.5f * Vector3.Cross(e0, e2).magnitude;

        float coef = -3f / (2f * (A0 + A1));
        float[] K = {c03 + c04, c01 + c02, -c01 - c03, -c02 - c04};
        float[] K2 = { coef * K[0], coef * K[1], coef * K[2], coef * K[3] };

        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < j; k++) {
                Q[j, k] = Q[k, j] = K[j] * K2[k];
            }
            Q[j, j] = K[j] * K2[j];
        }
    }

    public override void Satisfy(Vector3[] projectedPositions, float mass) {
        Vector3[] x = {projectedPositions[vertexIndeces[2]],
                       projectedPositions[vertexIndeces[3]],
                       projectedPositions[vertexIndeces[0]],
                       projectedPositions[vertexIndeces[1]] };

        float energy = 0;
        for (int k = 0; k < 4; k++) {
            for (int j = 0; j < 4; j++) {
                energy += Q[j, k] * (Vector3.Dot(x[k], x[j]));
            }
        }
        energy *= 0.5f;

        Vector3[] gradC = { Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero };
        for (int k = 0; k < 4; k++)
            for (int j = 0; j < 4; j++)
                gradC[j] += Q[j, k] * x[k];


        float sum_normGradC = 0;
        for (int j = 0; j < 4; j++) {
            // compute sum of squared gradient norms
            sum_normGradC += mass * gradC[j].sqrMagnitude;
        }

        // exit early if required
        if (Mathf.Abs(sum_normGradC) > 1e-7) {
            // compute impulse-based scaling factor
            float s = energy / sum_normGradC;

            projectedPositions[vertexIndeces[0]] += -stiffness * (s * mass) * gradC[2];
            projectedPositions[vertexIndeces[1]] += -stiffness * (s * mass) * gradC[3];
            projectedPositions[vertexIndeces[2]] += -stiffness * (s * mass) * gradC[0];
            projectedPositions[vertexIndeces[3]] += -stiffness * (s * mass) * gradC[1];
        }
    }
}

public class CollisionConstraint : Constraint {
    private int vertexIndex;
    private Vector3 collisionPos;
    private Vector3 collisionNormal;
    private Collider collider;
    private Vector3 closestPos;

    public CollisionConstraint(int i, RaycastHit info, Vector3 originalPos) {
        vertexIndex = i;
        collider = info.collider;
        collisionPos = info.point;
        collisionNormal = info.normal;
        //closestPos = collider.ClosestPoint(originalPos) + collisionNormal * 0.001f;
        closestPos = collisionPos + collisionNormal * 0.001f;

    }

    public override void Satisfy(Vector3[] projectedPositions, float mass) {
        Vector3 p = projectedPositions[vertexIndex];
        float cp = Vector3.Dot(p - collisionPos, collisionNormal);

        if (cp < 0) { // if constraint violated, project the constraint
            projectedPositions[vertexIndex] = closestPos;
        }
    }
}

public class PointConstraint {
    private int index;

    public PointConstraint(int i) {
        index = i;
    }

    public void Satisfy(Vector3[] projectedPositions, Vector3[] positions) {
        projectedPositions[index] = positions[index];
        // TODO: handle the case of a moving point constraint e.g. curtain
    }
}

public class GroundConstraint {
    private float groundPlane;

    public GroundConstraint(float g) {
        groundPlane = g;
    }

    public void Satisfy(Vector3[] projectedPositions, Vector3[] velocities) {
        for (int i = 0; i < projectedPositions.Length; i++) {
            if (projectedPositions[i][1] < groundPlane) {
                projectedPositions[i][1] = groundPlane;
                velocities[i] = Vector3.zero;
            }
        }
    }
}
