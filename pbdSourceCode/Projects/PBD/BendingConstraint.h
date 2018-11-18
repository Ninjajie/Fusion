/**
    This class acts on every winged edge in the mesh and ensures that adjacent
        triangles do not bend more than a certain angle.
 */

#ifndef CISPBA_BENDINGCONSTRAINT_H
#define CISPBA_BENDINGCONSTRAINT_H

#include <vector>
#include "Constraint.h"
#include "TriangleFace.h"
#include "Edge.h"
#include <unordered_map>
#include <set>
#include <map>
#include <math.h>
#include <iostream>
#include <string>


template <class T>
class BendingConstraint : public Constraint<T>{
public:
    explicit BendingConstraint(const std::vector<int>& vertexIndeces,
                               T restAngle, double weight) :
            vertexIndeces(vertexIndeces),
            restAngle(restAngle),
            weight(weight) {};
    ~BendingConstraint() = default;

    void satisfy(std::vector<vec3<T> >& projectedPositions,
                 std::vector<vec3<T> >& velocities,
                 const std::vector<T>& masses);

private:
    /* index of each point in projectedPositions and masses
     * this is indexed like the Bridson, Simulation of Clothing with Folds
     *     and Wrinkles paper
     *    3
     *    ^
     * 0  |  1
     *    2
     */
    std::vector<int> vertexIndeces;


    T restAngle;
    double weight;
};


#endif //CISPBA_BENDINGCONSTRAINT_H
