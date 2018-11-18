/*
    Base class for all constraints.

    Defines Satisfy(), which must be implemented by child classes and will be
        called upon by TriangleMesh to project its points
 */

#ifndef CISPBA_CONSTRAINT_H
#define CISPBA_CONSTRAINT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

template <class T>
using vec3 = Eigen::Matrix<T, 3, 1>;

template <class T>
class Constraint{
public:
    Constraint() = default;
    virtual ~Constraint() = default;

    // satisfies the given positions based on the requirements of the constraint
    // will be implemented by every derived class
    virtual void satisfy(std::vector<vec3<T> >& projectedPositions,
                         std::vector<vec3<T> >& velocities,
                         const std::vector<T>& masses) = 0;

};

#endif //CISPBA_CONSTRAINT_H
