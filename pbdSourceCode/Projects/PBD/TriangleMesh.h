/**
    This class holds all the information for a given simulated object such as
        the positions, velocities and masses of its vertices. It also keeps
        track of all the constraints acting on this object.

    The class contains various functions that each implement a step in the PBD
        main loop. PBDSimulator calls each of these functions in order.
 */

#ifndef CISPBA_TRIANGLEMESH_H
#define CISPBA_TRIANGLEMESH_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "../../cmake-build-debug/partio-src/src/lib/Partio.h"
#include "TriangleFace.h"
#include "DistanceConstraint.h"
#include "BendingConstraint.h"
#include "CollisionConstraint.h"
#include "GroundConstraint.h"
#include "PointConstraint.h"
#include "Sphere.h"
#include <random>


template <class T>
using vec3 = Eigen::Matrix<T, 3, 1>;

// simple enum to differentiate between adding different constraints
enum ConstraintType{
    distance,
    bending,
    ground
};

// simple enum to differentiate between adding different point constraints
enum PointConstraintType{
    topCorners,
    topRow,
    leftCorners,
    leftRow,
    curtainRight
};


template <class T>
class TriangleMesh{
public:
    explicit TriangleMesh(const std::string& fileName, vec3<T> startingPos,
                          T mass = (T)1) :
            groundConstraint(nullptr),
            numParticles(0)
            { readFromOBJFile(fileName, startingPos, mass); };

    ~TriangleMesh();

    void readFromOBJFile(const std::string& fileName, vec3<T> startingPos,
                         T mass = (T)1);
    void writeToBGEOFile(const std::string& fileName) const;

    void addConstraint(ConstraintType type, double input);
    void addPointConstraint(PointConstraintType type, int width, int height,
                            float distance = 0, float duration = 0);

    // the following methods are used in PBDSimulator's main loop in order
    void applyExternalForce(vec3<T> force, double dt);
    void dampVelocity(double stiffness);
    void applyExplicitEuler(double dt);
    void clearCollisionConstraints();
    void generateCollisionConstraints(const Sphere<T>& sphere);
    void satisfyConstraints();
    void satisfyPointConstraints(double dt);
    void updateVertices(double dt);
    void applyFriction();

    int size() const {return numParticles;};

private:
    std::vector<vec3<T> > positions;
    std::vector<vec3<T> > projectedPositions;
    std::vector<vec3<T> > velocities;
    std::vector<T> masses;    // note that mass is assume to be inverse mass w_i
    std::vector<TriangleFace> faces;
    std::vector<Constraint<T>* > constraints;
    std::vector<Constraint<T>* > collisionConstraints;
    std::vector<PointConstraint<T>* > pointConstraints;
    Constraint<T>* groundConstraint;
    int numParticles;
};



#endif //CISPBA_TRIANGLEMESH_H
