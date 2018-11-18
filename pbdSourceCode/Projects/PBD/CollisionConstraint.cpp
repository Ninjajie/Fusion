#include "CollisionConstraint.h"


template <class T>
CollisionConstraint<T>::CollisionConstraint(int index, vec3<T> position,
                                            vec3<T> projectedPosition,
                                            const Sphere<T>& sphere)
        : index(index), sphere(sphere){
    collisionPosition = sphere.getCollisionPosition(position, projectedPosition);
    normal = sphere.getNormal(collisionPosition);
};


// takes in projected positions and vertex mass,
// satisfy the projected positions and velocities according to this constraint
template <class T>
void CollisionConstraint<T>::satisfy(std::vector<vec3<T> >& projectedPositions,
                                     std::vector<vec3<T> >& velocities,
                                     const std::vector<T>& masses){
    // check to see if the projected point still collides with the object
    vec3<T> p = projectedPositions[index];
    T cp = (p - collisionPosition).dot(normal);

    if (cp < 0){ // constraint violated. project the constraint
        // n is the normal of the closest point on the sphere's surface to p
        vec3<T> n = (p - sphere.getCenter()).normalized();
        // q is the closest point on the sphere's surface to p
        vec3<T> q = sphere.getCenter() + n * sphere.getRadius();

        projectedPositions[index] = q;
    }
}


template class CollisionConstraint<double>;
template class CollisionConstraint<float>;