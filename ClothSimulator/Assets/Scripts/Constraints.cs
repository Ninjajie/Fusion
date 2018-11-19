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

        //Eigen::Matrix < T, 3, 4 > p;

        //p.col(0) = projectedPositions[vertexIndeces[0]];
        //p.col(1) = projectedPositions[vertexIndeces[1]];
        //p.col(2) = projectedPositions[vertexIndeces[2]];
        //p.col(3) = projectedPositions[vertexIndeces[3]];

        //vec3<T> wing = p.col(3) - p.col(2);
        //auto wingLength = wing.norm();

        //if (wingLength >= std::numeric_limits < T >::epsilon()) {
        //    //get normals
        //    auto n1 = (p.col(2) - p.col(0)).cross(p.col(3) - p.col(0));
        //    n1 /= n1.squaredNorm();

        //    auto n2 = (p.col(3) - p.col(1)).cross(p.col(2) - p.col(1));
        //    n2 /= n2.squaredNorm();
        //    //unlike in the original PBD paper,
        //    // both normals point in same direction

        //    //getting constraints along gradients (gradpC)
        //    auto invWingLength = 1.0 / wingLength;

        //    Eigen::Matrix < T, 3, 4 > q;
        //    q.col(0) = wingLength * n1;

        //    q.col(1) = wingLength * n2;

        //    q.col(2) = (p.col(0) - p.col(3)).dot(wing) * invWingLength * n1
        //                + (p.col(1) - p.col(3)).dot(wing) * invWingLength * n2;

        //    q.col(3) = (p.col(2) - p.col(0)).dot(wing) * invWingLength * n1
        //                + (p.col(2) - p.col(1)).dot(wing) * invWingLength * n2;

        //    //find current angle
        //    n1.normalize();
        //    n2.normalize();

        //    auto d = n1.dot(n2);
        //    if (d < -1.0) d = -1.0;
        //    if (d > 1.0) d = 1.0;
        //    auto currentAngle = acos(d);

        //    //find lamda ( where deltap = lamda*wi*gradConstraint )
        //    auto lamda = 0;
        //    for (int i = 0; i < 4; i++) {
        //        lamda += (masses[vertexIndeces[i]]) * q.col(i).squaredNorm();
        //    }

        //    if (lamda != 0.0) {
        //        lamda = (currentAngle - restAngle) / lamda * weight;

        //        if (n1.cross(n2).dot(wing) > 0.0) lamda = -lamda;

        //        for (int i = 0; i < 4; i++) {
        //            projectedPositions[vertexIndeces[i]]
        //                -= (masses[vertexIndeces[i]]) * lamda * q.col(i);
        //        }
        //    }
        //}
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