// Kinematic objects for deformable objects to collide into

/**
    Simple kinematic sphere to collide with simulated objects.
 */

#ifndef CISPBA_KINEMATICOBJECT_H
#define CISPBA_KINEMATICOBJECT_H

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include "../../cmake-build-debug/partio-src/src/lib/Partio.h"

template <class T>
using vec3 = Eigen::Matrix<T, 3, 1>;


template <class T>
class Sphere{
public:
    explicit Sphere(vec3<T> c, double r): center(c), radius(r), speed(0) {
        direction << 0, 0, 0;
    };
    explicit Sphere(vec3<T> c, double r, vec3<T> d, double s) :
        center(c), radius(r), direction(d), speed(s) {};


    bool isColliding(const vec3<T>& point) const;

    vec3<T> getCollisionPosition(vec3<T> originalPosition,
                                 vec3<T> projectedPosition) const;

    vec3<T> getNormal(vec3<T> point) const;

    void move(double dt);

    void writeToBGEOFile(const std::string &fileName) const;

    vec3<T> getCenter() const { return center; };
    double getRadius() const { return radius; };

private:
    vec3<T> center;
    double radius;
    vec3<T> direction;
    double speed;
};

#endif //CISPBA_KINEMATICOBJECT_H
