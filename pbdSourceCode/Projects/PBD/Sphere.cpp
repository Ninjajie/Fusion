#include "Sphere.h"


// check if the given point is inside this sphere
template <class T>
bool Sphere<T>::isColliding(const vec3<T>& point) const{
    return (point - center).norm() <= radius;
}


// perform ray sphere intersection test to get the point of intersection
template <class T>
vec3<T> Sphere<T>::getCollisionPosition(vec3<T> originalPosition,
                                        vec3<T> projectedPosition) const{
    vec3<T> direction = (projectedPosition - originalPosition).normalized();
    // L is the distance from original point to the center
    T L = (center - originalPosition).norm();
    // tc is the distance from original point to the center's
    // projected point on the ray
    T tc = (center - originalPosition).dot(direction);
    // d is the closest distance from center to the ray
    T d = std::sqrt(std::pow(L, 2) - std::pow(tc, 2));
    // tc1 is the distance from the collision point to the center's
    // projected point on the ray
    T tc1 = std::sqrt(std::pow(radius, 2) - std::pow(d, 2));
    // t is the distance from original position to the collision point
    T t = tc - tc1;

    vec3<T> result = originalPosition + direction * t;

    return result;
};


// given a point on the sphere's surface, get its surface normal
template <class T>
vec3<T> Sphere<T>::getNormal(vec3<T> point) const{
    return (point - center).normalized();
};


// moves the sphere forward
template <class T>
void Sphere<T>::move(double dt) {
    center += direction * speed * dt;
}


// writes particle data into a file (just the center)
template <class T>
void Sphere<T>::writeToBGEOFile(const std::string &fileName) const {
    Partio::ParticlesDataMutable* parts = Partio::create();
    Partio::ParticleAttribute positionHandle;

    positionHandle = parts->addAttribute("position", Partio::VECTOR, 3);
    int idx = parts->addParticle();
    float* p = parts->dataWrite<float>(positionHandle, idx);
    for (int k = 0; k < 3; k++){
        p[k] = center[k];
    }

    Partio::write(fileName.c_str(), *parts);
    parts->release();
}


template class Sphere<double>;
template class Sphere<float>;