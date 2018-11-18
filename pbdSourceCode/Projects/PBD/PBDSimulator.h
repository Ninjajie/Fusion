/**
    This class handles actually running the actual PBD simulation.
    Implementation based on this 2006 paper:
    http://matthias-mueller-fischer.ch/publications/posBasedDyn.pdf

    It keeps tracks of a list of simulated objects, a list of kinematic
        objects, and the parameters of the simulation.
 */

#ifndef CISPBA_PBDSIMULATOR_H
#define CISPBA_PBDSIMULATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include "TriangleMesh.h"
#include "Sphere.h"


template <class T>
using vec3 = Eigen::Matrix<T, 3, 1>;


template <class T>
class PBDSimulator{

public:
    // the constructor takes in all of the needed variables for the simulation
    PBDSimulator(double timeStep = 0.2,
                 int iterationNum = 5,
                 int frameRate = 30,
                 T m = 1,
                 double k = 0.9) :
                    timeStep(timeStep),
                    iterationNum(iterationNum),
                    frameRate(frameRate),
                    particleMass(m),
                    dampingStiffness(k) {};

    ~PBDSimulator(){
        for(auto& o : objects){
            delete o;
        }
    };

    void addObject(const std::string& objFilePath, vec3<T> startingPos);

    // runs the actual simulations and print out BGEO files
    void runSimulation(T seconds, const std::string& outputFilePath);

    // the following functions add additional settings to the simulation
    void addConstraint(ConstraintType type, double input);
    void addPointConstraint(PointConstraintType type,
                            int width, int height,
                            float distance = 0, float duration = 0);
    void addPointConstraint(int index, PointConstraintType type,
                            int width, int height,
                            float distance = 0, float duration = 0);
    void addKinematicSphere(Eigen::Matrix<T, 3, 1> center, double radius);
    void addKinematicSphere(Eigen::Matrix<T, 3, 1> center, double radius,
                            Eigen::Matrix<T, 3, 1> direction, double speed);


private:
    std::vector<TriangleMesh<T>*> objects;
    std::vector<Sphere<T> > spheres;
    double timeStep;
    int iterationNum;
    int frameRate;
    T particleMass;
    double dampingStiffness;
};



#endif //CISPBA_PBDSIMULATOR_H
