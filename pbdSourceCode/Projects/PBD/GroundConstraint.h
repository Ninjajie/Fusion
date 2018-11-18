/**
    This constraint acts on every position in the mesh and simply makes
        sure that they are above the groundPlane
 */

#ifndef CISPBA_GROUNDCONSTRAINT_H
#define CISPBA_GROUNDCONSTRAINT_H


#include "Constraint.h"

template <class T>
class GroundConstraint : public Constraint<T>{
public:
    explicit GroundConstraint(double groundPlane) : groundPlane(groundPlane) {};
    ~GroundConstraint() = default;

    void satisfy(std::vector<vec3<T> >& projectedPositions,
                 std::vector<vec3<T> >& velocities,
                 const std::vector<T>& masses);

private:
    double groundPlane;

};


#endif //CISPBA_GROUNDCONSTRAINT_H
