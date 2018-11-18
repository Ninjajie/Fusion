#include "GroundConstraint.h"


// takes in projected positions and vertex mass,
// satisfy the projected positions and velocities according to this constraint
template <class T>
void GroundConstraint<T>::satisfy(std::vector<vec3<T> > &projectedPositions,
                                  std::vector<vec3<T> >& velocities,
                                  const std::vector<T> &masses) {
    vec3<T> zero;
    zero << 0, 0, 0;
    for (auto i = 0u; i < projectedPositions.size(); i++){
        if (projectedPositions[i][1] < groundPlane){
            projectedPositions[i][1] = groundPlane;
            velocities[i] = zero;
        }
    }
}


template class GroundConstraint<double>;
template class GroundConstraint<float>;