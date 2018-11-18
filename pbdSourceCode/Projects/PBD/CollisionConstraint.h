/**
    This constraint acts on only a single point and ensures that it is not
        inside of a given sphere.

    This constraint is generated at the beginning of every time step for every
        point that needs to be corrected.
 */

#ifndef CISPBA_COLLISIONCONSTRAINT_H
#define CISPBA_COLLISIONCONSTRAINT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include "Constraint.h"
#include "Sphere.h"
#include "iostream"


template <class T>
class CollisionConstraint : public Constraint<T>{
public:
    explicit CollisionConstraint(int index,
                                 vec3<T> position,
                                 vec3<T> projectedPosition,
                                 const Sphere<T>& sphere);
    ~CollisionConstraint() = default;

    void satisfy(std::vector<vec3<T> >& projectedPositions,
                 std::vector<vec3<T> >& velocities,
                 const std::vector<T>& masses);

private:
    int index;
    vec3<T> collisionPosition;
    vec3<T> normal;
    const Sphere<T>& sphere;
};



#endif //CISPBA_COLLISIONCONSTRAINT_H
