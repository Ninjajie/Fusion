#include "PBDSimulator.h"


// instantiate an object from a filePath as a cloth Sim
template <class T>
void PBDSimulator<T>::addObject(const std::string& objFilePath,
                                vec3<T> startingPos) {
    objects.emplace_back(new TriangleMesh<T> (objFilePath, startingPos,
                                              1.0 / particleMass));
}


// run the simulation for given amount of seconds and output
// every frame as a BGEO file.
template <class T>
void PBDSimulator<T>::runSimulation(T seconds, const std::string& filePath) {
    // set up some initial data
    vec3<T> gravity;
    gravity << (T)0, (T)-9.81, (T)0;

    // Main loop begins
    // step 1-3: initialize position, velocity and weight
    // already initialized during object creation


    // set up variables for calculating the correct dt
    double time = 0.0;
    double frameTime = 1.0 / frameRate;
    int totalFrames = seconds * frameRate;
    double extra = 0.0;

    for (int frame = 0; frame <= totalFrames; frame++){
        double currentFrameEndTime = frame * frameTime;

        // run the simulation until we're at the desired frame time
        while (time < currentFrameEndTime){
            double dt = std::min(timeStep, currentFrameEndTime - time);
            // add in any extra dt from previous frames
            if (extra > 0.0){
                dt += extra;
                extra = 0.0;
            }

            // if dt is too small, and it to the next dt as extra
            if (dt < 0.005){
                extra = dt;
                break;
            }

            time += dt;

            for (auto i = 0u; i < objects.size(); i++){
                TriangleMesh<T>* object = objects[i];

                // step 5: add external forces like gravity to velocity
                object->applyExternalForce(gravity, dt);

                // step 6: damp velocity
                object->dampVelocity(dampingStiffness);

                // step 7: apply explicit Euler to positions based on velocity
                object->applyExplicitEuler(dt);

                // step 8: clear current collisions and generate new collisions
                object->clearCollisionConstraints();
                for (auto& s : spheres) {
                    s.move(dt);
                    object->generateCollisionConstraints(s);
                }

                // step 9-11: project constraints iterationNum times
                for (auto i = 0; i < iterationNum; i++){
                    // satisfy all constraints
                    object->satisfyConstraints();
                }

                // satisfy pointConstraints
                object->satisfyPointConstraints(dt);

                // step 13 & 14: apply projected positions to actual vertices
                object->updateVertices(dt);

                // step 16: update all velocities using friction
                object->applyFriction();
            }

        }

        // print out the frame for all objects and spheres
        for (auto i = 0u; i < objects.size(); i++){
            objects[i]->writeToBGEOFile(filePath + std::to_string(i) + "_"
                                        + std::to_string(frame) + ".bgeo");
        }
        for (auto i = 0u; i < spheres.size(); i++) {
            spheres[i].writeToBGEOFile(filePath + "Sphere" + std::to_string(i)
                                       + "_" + std::to_string(frame) + ".bgeo");
        }
    }
}


template <class T>
void PBDSimulator<T>::addConstraint(ConstraintType type, double input) {
    for (auto& o : objects) {
        o->addConstraint(type, input);
    }
}


// add point constraints to all objects
template <class T>
void PBDSimulator<T>::addPointConstraint(PointConstraintType type,
                                         int width, int height,
                                         float distance, float duration) {
    for (auto& o : objects){
        o->addPointConstraint(type, width, height, distance, duration);
    }
}


// add point constraint to a single object
template <class T>
void PBDSimulator<T>::addPointConstraint(int index, PointConstraintType type,
                                         int width, int height,
                                         float distance, float duration) {
    if (index <= (int)objects.size()){
        objects[index]->
                addPointConstraint(type, width, height, distance, duration);
    }else{
        std::cerr << "Error: adding point constraint to nonexistant object"
                  << std::endl;
    }
}


template <class T>
void PBDSimulator<T>::addKinematicSphere(vec3<T> center, double radius) {
    spheres.emplace_back(Sphere<T>(center, radius));
}


template <class T>
void PBDSimulator<T>::addKinematicSphere(vec3<T> center, double radius,
                                              vec3<T> direction, double speed) {
    spheres.emplace_back(Sphere<T>(center, radius, direction, speed));
}


template class PBDSimulator<double>;
template class PBDSimulator<float>;