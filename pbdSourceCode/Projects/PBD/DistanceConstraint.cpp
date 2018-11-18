#include "DistanceConstraint.h"


// takes in projected positions and vertex mass,
// satisfy the projected positions according to this constraint
template <class T>
void DistanceConstraint<T>::satisfy(std::vector<vec3<T> >& projectedPositions,
                                    std::vector<vec3<T> >& velocities,
                                    const std::vector<T>& masses) {
    //get positions
    vec3<T> pi = projectedPositions[edge.start];
    vec3<T> pj = projectedPositions[edge.end];

    //make edge vector
    vec3<T> n = ( pi.col(0) - pj.col(0) );

    //get current length
    T L = n.col(0).norm();

    //normalize edge vector
    n /= L;

    T wi = masses[edge.start];
    T wj = masses[edge.end];

    projectedPositions[edge.start] = pi - weight * wi
                                          / (wi + wj) * (L - restLength) * n;
    projectedPositions[edge.end]   = pj + weight * wj
                                          / (wi + wj) * (L - restLength) * n;
}


template class DistanceConstraint<double>;
template class DistanceConstraint<float>;