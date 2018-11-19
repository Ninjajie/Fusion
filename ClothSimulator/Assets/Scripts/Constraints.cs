using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Constraint {

    public abstract void Satisfy(Vector3[] projectedPositions, float mass);
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

    public override void Satisfy(Vector3[] projectedPositions, float mass) {
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

public class CollisionConstraint : Constraint {
    private Vector3 collisionPos;
    private Vector3 normal;
    private int index;
    private Vector3 sphereCenter;
    private float sphereRadius;

    public CollisionConstraint(int i, Vector3 pos, Vector3 projectedPos, Vector3 center, float radius) {
        index = i;
        sphereCenter = center;
        sphereRadius = radius;

        // get collision pos and normals

        Vector3 direction = (projectedPos - pos).normalized;
        // L is the distance from original point to the center
        float L = (center - pos).magnitude;
        // tc is the distance from original point to the center's
        // projected point on the ray
        float tc = Vector3.Dot(center - pos, direction);
        // d is the closest distance from center to the ray
        float d = Mathf.Sqrt(L * L - tc * tc);
        // tc1 is the distance from the collision point to the center's
        // projected point on the ray
        float tc1 = Mathf.Sqrt(radius * radius - d * d);
        // t is the distance from original position to the collision point
        float t = tc - tc1;

        collisionPos = pos + direction * t;
        normal = (collisionPos - center).normalized;
    }

    public override void Satisfy(Vector3[] projectedPositions, float mass) {
        Vector3 p = projectedPositions[index];
        float cp = Vector3.Dot(p - collisionPos, normal);

        if (cp < 0) { // constraint violated. project the constraint
                      // n is the normal of the closest point on the sphere's surface to p
            Vector3 n = (p - sphereCenter).normalized;
            // q is the closest point on the sphere's surface to p
            Vector3 q = sphereCenter + n * sphereRadius;

            projectedPositions[index] = q;
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
