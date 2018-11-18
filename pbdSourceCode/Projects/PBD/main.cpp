/**
    Main file that runs the simulation.

    Contains several functions that each simulates a scenario.

    Each function creates a custom cloth mesh using OBJGenerator and feeds data
        into PBDSimulator that does all the simulation and outputs BGEO files.

    The generated OBJ and BGEO files can be visualized using Houdini.
 */

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "PBDSimulator.h"
#include "OBJGenerator.h"

using std::cout; using std::endl;

using T = double;
constexpr int dim = 3;

template <class T>
using vec3 = Eigen::Matrix<T, 3, 1>;


// Runs a simulation where a sphere moves through a hanging cloth
void runSphereCollisionSim(){
    cout << "running sphere collision simulation" << endl;

    // make a custom cloth mesh

    std::string fileName = "ClothSphereCollision";
    int width = 55;
    int height = 45;
    double dist = 0.5;
    vec3<T> startingPos;
    startingPos << 0, height * dist + 5, width * dist / 2.0;
    //OBJGenerator::generateClothOBJ(fileName, dist, width, height,
    //                               OBJGeneratorMode::vertical);

    // set parameters for the simulation
    double timeStep = 0.02;
    int frameRate = 24;
    int iterationNum = 5;
    double mass = 2;
    double simulationLength = 30;
    std::string inputFileName = fileName + ".obj";
    std::string outputFileName = "./output/" + fileName;

    // set parameters for the kinematic sphere
    double sphereRadius = 8;
    vec3<T> spherePos, sphereDir;
    spherePos << -sphereRadius - 4 , height * dist / 3 + 8, 0;
    sphereDir << 1, 0, 0;
    double sphereSpeed = 6;

    // set parameters for constraints
    double distanceWeight = 0.8;
    double bendingWeight = 0.5;
    double dampingStiffness = 0.02;
    double groundPlane = 0;

    // set up the simulation
    PBDSimulator<T> simulator(timeStep, iterationNum,
                              frameRate, mass, dampingStiffness);
    simulator.addObject(inputFileName, startingPos);
    simulator.addPointConstraint(PointConstraintType::topRow,
                                 width, height);
    simulator.addKinematicSphere(spherePos, sphereRadius,
                                 sphereDir, sphereSpeed);
    simulator.addConstraint(ConstraintType::distance, distanceWeight);
    simulator.addConstraint(ConstraintType::bending, bendingWeight);
    simulator.addConstraint(ConstraintType::ground, groundPlane);

    simulator.runSimulation(simulationLength, outputFileName);
}


// Runs a simulation where two spheres move through a hanging cloth
void runDoubleSphereCollisionSim(){
    cout << "running double sphere collision simulation" << endl;

    // make a custom cloth mesh
    std::string fileName = "ClothDoubleSphereCollision";
    int width = 55;
    int height = 45;
    double dist = 0.5;
    vec3<T> startingPos;
    startingPos << 0, height * dist + 5, width * dist / 2.0;
    OBJGenerator::generateClothOBJ(fileName, dist, width, height,
                                   OBJGeneratorMode::vertical);

    // set parameters for the simulation
    double timeStep = 0.02;
    int frameRate = 24;
    int iterationNum = 5;
    double mass = 3;
    double simulationLength = 30;
    std::string inputFileName = fileName + ".obj";
    std::string outputFileName = "./output/" + fileName;

    // set parameters for the kinematic spheres
    double sphereRadius = 4;
    vec3<T> spherePos1, sphereDir, spherePos2, sphereDir2;
    spherePos1 << -sphereRadius - 2 , height * dist / 4 + 5, 0;
    spherePos2 << -sphereRadius + 14 , - 10, 0;
    sphereDir << 1, 0, 0;
    sphereDir2 << 0, 1, 0;
    double sphereSpeed = 7;
    double sphereSpeed2 = 4;


    // set parameters for constraints
    double distanceWeight = 0.99;
    double bendingWeight = 0.5;
    double dampingStiffness = 0.02;
    double groundPlane = 0;

    // set up the simulation
    PBDSimulator<T> simulator(timeStep, iterationNum,
                              frameRate, mass, dampingStiffness);
    simulator.addObject(inputFileName, startingPos);
    simulator.addPointConstraint(PointConstraintType::topRow,
                                 width, height);
    simulator.addKinematicSphere(spherePos1, sphereRadius,
                                 sphereDir, sphereSpeed);
    simulator.addKinematicSphere(spherePos2, sphereRadius,
                                 sphereDir2, sphereSpeed2);
    simulator.addConstraint(ConstraintType::distance, distanceWeight);
    simulator.addConstraint(ConstraintType::bending, bendingWeight);
    simulator.addConstraint(ConstraintType::ground, groundPlane);

    simulator.runSimulation(simulationLength, outputFileName);
}


// Runs a simulation where a sphere moves through 3 pieces of hanging cloth
void runTripleClothSphereCollisionSim(){
    cout << "running triple cloth sphere collision simulation" << endl;

    // make a custom cloth mesh
    std::string fileName = "TripleClothSphereCollision";
    int width = 20;
    int height = 20;
    double dist = 0.8;
    vec3<T> startingPos1, startingPos2, startingPos3;
    startingPos1 << 0, height * dist + 5, width * dist / 2.0;
    startingPos2 << 20, height * dist + 7, width * dist / 2.0;
    startingPos3 << 40, height * dist + 9, width * dist / 2.0;
    OBJGenerator::generateClothOBJ(fileName, dist, width, height,
                                   OBJGeneratorMode::vertical);

    // set parameters for the simulation
    double timeStep = 0.02;
    int frameRate = 24;
    int iterationNum = 5;
    double mass = 3;
    double simulationLength = 30;
    std::string inputFileName = fileName + ".obj";
    std::string outputFileName = "./output/" + fileName;

    // set parameters for the kinematic sphere
    double sphereRadius = 4;
    vec3<T> spherePos, sphereDir;
    spherePos << -sphereRadius - 4 , height * dist / 3 + 5, 0;
    sphereDir << 1, 0, 0;
    double sphereSpeed = 3;

    // set parameters for constraints
    double distanceWeight = 0.4;
    double bendingWeight = 0.5;
    double dampingStiffness = 0.02;
    double groundPlane = 0;

    // set up the simulation
    PBDSimulator<T> simulator(timeStep, iterationNum,
                              frameRate, mass, dampingStiffness);
    simulator.addObject(inputFileName, startingPos1);
    simulator.addObject(inputFileName, startingPos2);
    simulator.addObject(inputFileName, startingPos3);
    simulator.addPointConstraint(PointConstraintType::topCorners,
                                 width, height);
    simulator.addKinematicSphere(spherePos, sphereRadius,
                                 sphereDir, sphereSpeed);
    simulator.addConstraint(ConstraintType::distance, distanceWeight);
    simulator.addConstraint(ConstraintType::bending, bendingWeight);
    simulator.addConstraint(ConstraintType::ground, groundPlane);

    simulator.runSimulation(simulationLength, outputFileName);
}

// runs a simulation where a hanging cloth is moved like a curtain
void runCurtainSim(){
    cout << "running curtain sim" << endl;

    // makes a custom cloth mesh
    std::string fileName = "ClothCurtain";
    int width = 40;
    int height = 40;
    double dist = 0.25;
    Eigen::Vector3d startingPos;
    startingPos << 0, height * dist + 5, 0;
    OBJGenerator::generateClothOBJ(fileName, dist, width, height,
                                   OBJGeneratorMode::vertical);

    // set parameters for the simulation
    double timeStep = 0.02;
    int frameRate = 24;
    int iterationNum = 5;
    double mass = 3.0*4;
    double simulationLength = 30;
    std::string inputFileName = fileName + ".obj";
    std::string outputFileName = "./output/" + fileName;

    // set parameters for the constraints
    double distanceWeight = 0.9;
    double bendingWeight = 0.8;
    double dampingStiffness = 0.02;
    double groundPlane = 0;
    double curtainMoveDist = width * dist;
    double curtainMoveDuration = 9;

    // setup the simulation
    PBDSimulator<T> simulator(timeStep, iterationNum,
                              frameRate, mass, dampingStiffness);
    simulator.addObject(inputFileName, startingPos);
    simulator.addPointConstraint(PointConstraintType::curtainRight,
                                 width, height,
                                 curtainMoveDist, curtainMoveDuration);
    simulator.addConstraint(ConstraintType::distance, distanceWeight);
    simulator.addConstraint(ConstraintType::bending, bendingWeight);
    simulator.addConstraint(ConstraintType::ground, groundPlane);

    simulator.runSimulation(simulationLength, outputFileName);
}


// runs a simulation where a cloth runs into a sphere
void runSphereRunIntoClothSim(){
    cout << "running sphere running to cloth sim" << endl;

    // makes a custom cloth mesh
    std::string fileName = "SphereIntoCloth";
    int width = 55;
    int height = 45;
    double dist = 0.5;
    Eigen::Vector3d startingPos;
    startingPos << 0, 40, width * dist / 2.0;
    OBJGenerator::generateClothOBJ(fileName, dist, width, height,
                                   OBJGeneratorMode::vertical);

    // set parameters for the simulation
    double timeStep = 0.01;
    int frameRate = 30;
    int iterationNum = 4;
    double mass = 1;
    double simulationLength = 30;
    std::string inputFileName = fileName + ".obj";
    std::string outputFileName = "./output/" + fileName;


    // set parameters for the static sphere
    vec3<T> spherePos, sphereDir;
    spherePos << -10, 15, 0;
    double sphereRadius = 6;
    sphereDir << 1, 0, 0;
    double sphereSpeed = 8;

    // set parameters for constraints
    double distanceWeight = 0.6;
    double bendingWeight = 0.5;
    double dampingStiffness = 0.02;
    double groundPlane = 0;

    // setup the simulation
    PBDSimulator<T> simulator(timeStep, iterationNum,
                              frameRate, mass, dampingStiffness);
    simulator.addObject(inputFileName, startingPos);
    simulator.addKinematicSphere(spherePos, sphereRadius,
                                 sphereDir, sphereSpeed);
    simulator.addConstraint(ConstraintType::distance, distanceWeight);
    simulator.addConstraint(ConstraintType::bending, bendingWeight);
    simulator.addConstraint(ConstraintType::ground, groundPlane);

    simulator.runSimulation(simulationLength, outputFileName);
}


// runs a simulation where a cloth drops onto a sphere
void runClothDropOntoSphereSim(){
    cout << "running cloth drop onto sphere sim" << endl;

    // makes a custom cloth mesh
    std::string fileName = "ClothDropOntoSphere";
    int width = 55;
    int height = 45;
    double dist = 0.5;
    Eigen::Vector3d startingPos;
    startingPos << 0, 40, width * dist / 2.0;
    OBJGenerator::generateClothOBJ(fileName, dist, width, height,
                                   OBJGeneratorMode::horizontal);

    // set parameters for the simulation
    double timeStep = 0.01;
    int frameRate = 30;
    int iterationNum = 4;
    double mass = 1;
    double simulationLength = 12;
    std::string inputFileName = fileName + ".obj";
    std::string outputFileName = "./output/" + fileName;


    // set parameters for the static sphere
    vec3<T> spherePos, sphereDir;
    spherePos << -10, 12, 0;
    double sphereRadius = 6;

    // set parameters for constraints
    double distanceWeight = 0.6;
    double bendingWeight = 0.5;
    double dampingStiffness = 0.02;
    double groundPlane = 0;

    // setup the simulation
    PBDSimulator<T> simulator(timeStep, iterationNum,
                              frameRate, mass, dampingStiffness);
    simulator.addObject(inputFileName, startingPos);
    simulator.addKinematicSphere(spherePos, sphereRadius);
    simulator.addConstraint(ConstraintType::distance, distanceWeight);
    simulator.addConstraint(ConstraintType::bending, bendingWeight);
    simulator.addConstraint(ConstraintType::ground, groundPlane);

    simulator.runSimulation(simulationLength, outputFileName);
}

// runs a simulation where a cloth drops onto a sphere
void runClothDropOnto2SpheresSim(){
    cout << "running cloth drop onto 2 spheres sim" << endl;

    // makes a custom cloth mesh
    std::string fileName = "ClothDropOnto2Spheres";
    int width = 55;
    int height = 45;
    double dist = 0.5;
    Eigen::Vector3d startingPos;
    startingPos << 0, 40, width * dist / 2.0;
    OBJGenerator::generateClothOBJ(fileName, dist, width, height,
                                   OBJGeneratorMode::horizontal);

    // set parameters for the simulation
    double timeStep = 0.01;
    int frameRate = 30;
    int iterationNum = 4;
    double mass = 1;
    double simulationLength = 24;
    std::string inputFileName = fileName + ".obj";
    std::string outputFileName = "./output/" + fileName;


    // set parameters for the static sphere
    vec3<T> spherePos, spherePos2;
    spherePos << -10, 24, 0;
    spherePos2 << -16, 12, 0;
    double sphereRadius = 6;
    double sphereRadius2 = 8;

    // set parameters for constraints
    double distanceWeight = 0.6;
    double bendingWeight = 0.5;
    double dampingStiffness = 0.02;
    double groundPlane = 0;

    // setup the simulation
    PBDSimulator<T> simulator(timeStep, iterationNum,
                              frameRate, mass, dampingStiffness);
    simulator.addObject(inputFileName, startingPos);
    simulator.addKinematicSphere(spherePos, sphereRadius);
    simulator.addKinematicSphere(spherePos2, sphereRadius2);
    simulator.addConstraint(ConstraintType::distance, distanceWeight);
    simulator.addConstraint(ConstraintType::bending, bendingWeight);
    simulator.addConstraint(ConstraintType::ground, groundPlane);

    simulator.runSimulation(simulationLength, outputFileName);
}

// runs a simulation where a bunny of varying squishiness drops onto the ground
void runBunnySim(double dampingStiffness, std::string version, T starting ){
    cout << "running bunny sim" << endl;

    
    // set parameters for the simulation
    double timeStep = 0.01;
    int frameRate = 30;
    int iterationNum = 4;
    double mass = 1;
    double simulationLength = 8;
    std::string fileName = "bunny";
    std::string inputFileName = fileName + ".obj";
    std::string outputFileName = "./output/" + fileName + version;

    vec3<T> startingPos;
    startingPos << starting, 0.1, 0;

    // set parameters for constraints
    double distanceWeight = 0.6;
    double bendingWeight = 0.9;
    double groundPlane = 0;

    // setup the simulation
    PBDSimulator<T> simulator(timeStep, iterationNum,
                              frameRate, mass, dampingStiffness);
    simulator.addObject(inputFileName, startingPos);
    simulator.addConstraint(ConstraintType::distance, distanceWeight);
    simulator.addConstraint(ConstraintType::bending, bendingWeight);
    simulator.addConstraint(ConstraintType::ground, groundPlane);

    simulator.runSimulation(simulationLength, outputFileName);
}

int main(int argc, char* argv[])
{
    runSphereCollisionSim();
    runDoubleSphereCollisionSim();
    runTripleClothSphereCollisionSim();
    runCurtainSim();
    runSphereRunIntoClothSim();
    runClothDropOntoSphereSim();
    runClothDropOnto2SpheresSim();
    //runBunnySim( (double)0.8, "0.8", (T)0);
    //runBunnySim( (double)0.02, "0.02",(T)0.1);
    //runBunnySim( (double)1, "1",(T)0.1);
    //runBunnySim( (double)10, "10",(T)0.1);

    return 0;
}
