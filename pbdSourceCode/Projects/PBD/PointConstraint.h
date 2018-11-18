/**
    This constraint acts on only a single point and ensures that it either
        stays put or moves in the prescribed manner.

 */

#ifndef CISPBA_POINTCONSTRAINT_H
#define CISPBA_POINTCONSTRAINT_H

#include <Eigen/Core>
#include "Constraint.h"


template <class T>
class PointConstraint{
public:
    explicit PointConstraint(int index, vec3<T> position) :
            index(index), position(position), duration(0) {
        vec3<T> zero;
        zero << 0, 0, 0;
        velocity = zero;
        currentDuration = duration / 2.0;
    };
    explicit PointConstraint(int index, vec3<T> position,
                             vec3<T> velocity, double duration) :
            index(index), position(position),
            velocity(velocity), duration(duration), currentDuration(0) {
        currentDuration = duration / 2.0;
    };
    ~PointConstraint() = default;


    void satisfy(std::vector<vec3<T> >& projectedPositions,
                 std::vector<vec3<T> >& positions,
                 double dt);

private:
    int index;
    vec3<T> position;
    vec3<T> velocity;
    double duration;
    double currentDuration;
};



#endif //CISPBA_POINTCONSTRAINT_H
