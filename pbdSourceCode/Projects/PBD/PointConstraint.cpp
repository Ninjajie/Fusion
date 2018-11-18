#include "PointConstraint.h"

// takes in projected positions and vertex mass,
// satisfy the projected positions and velocities according to this constraint
template <class T>
void PointConstraint<T>::satisfy(std::vector<vec3<T> >& projectedPositions,
                                 std::vector<vec3<T> >& positions,
                                 double dt) {
    if (duration > 0) {
        currentDuration += dt;
        if (currentDuration > duration) {
            currentDuration = 0;
            velocity *= -1;
        }
    }

        // set the corresponding vertex back to the starting position
    projectedPositions[index] = positions[index] + velocity * dt;
}

template class PointConstraint<double>;
template class PointConstraint<float>;