using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Constraint {

    public abstract void Satisfy(Vector3[] projectedPositions, Vector3[] positions, float mass);
}

public class DistanceConstraint : Constraint {
    private Edge edge;
    private float restLength;
    private float weight;

    public DistanceConstraint(Edge e, float l, float w) {
        edge = e;
        restLength = l;
        weight = w;
    }

    public override void Satisfy(Vector3[] projectedPositions, Vector3[] positions, float mass) {
        //get positions
        Vector3 pi = projectedPositions[edge.startIndex];
        Vector3 pj = projectedPositions[edge.endIndex];

        //make edge vector
        Vector3 n = pi - pj;

        //get current length
        float L = n.magnitude;

        //normalize edge vector
        n.Normalize();

        float wi = mass;
        float wj = mass;

        projectedPositions[edge.startIndex] = pi - weight * wi
                                              / (wi + wj) * (L - restLength) * n;
        projectedPositions[edge.endIndex] = pj + weight * wj
                                              / (wi + wj) * (L - restLength) * n;
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
    float weight;

    public BendingConstraint(int[] indices, float a, float w) {
        vertexIndeces = indices;
        restAngle = a;
        weight = w;
    }

    public override void Satisfy(Vector3[] projectedPositions, Vector3[] positions, float mass) {
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
                lamda = (currentAngle - restAngle) / lamda * weight;

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

public class PointConstraint : Constraint {
    private int index;

    public PointConstraint(int i) {
        index = i;
    }

    public override void Satisfy(Vector3[] projectedPositions, Vector3[] positions, float mass) {
        projectedPositions[index] = positions[index];
        // TODO: handle the case of a moving point constraint e.g. curtain
    }
}