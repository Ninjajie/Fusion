/**
    This constraint acts on every single edge in the mesh and ensures that no
        two vertices are more than a certain distance apart.
 */

#ifndef CISPBA_DISTANCECONSTRAINT_H
#define CISPBA_DISTANCECONSTRAINT_H

#include <vector>
#include <iostream>
#include <set>
#include <algorithm>

#include "Constraint.h"
#include "TriangleFace.h"
#include "Edge.h"


template <class T>
class DistanceConstraint : public Constraint<T>{
public:
    explicit DistanceConstraint(const Edge& edge, T restLength, double weight) :
            edge(edge), restLength(restLength), weight(weight) {};
    ~DistanceConstraint() = default;

    void satisfy(std::vector<vec3<T> >& projectedPositions,
                 std::vector<vec3<T> >& velocities,
                 const std::vector<T>& masses);

private:
    Edge edge;
    T restLength;
    double weight;
};


#endif //CISPBA_DISTANCECONSTRAINT_H
