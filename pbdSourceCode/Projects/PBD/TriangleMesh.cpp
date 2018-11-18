#include "TriangleMesh.h"

// destructor
template <class T>
TriangleMesh<T>::~TriangleMesh(){
    for (Constraint<T>* c : constraints){
        delete c;
    }
    for (Constraint<T>* c : collisionConstraints){
        delete c;
    }
    for (PointConstraint<T>* c : pointConstraints){
        delete c;
    }

    delete groundConstraint;
};



// reads particles and face data from a .obj file. replaces current data
template <class T>
void TriangleMesh<T>::readFromOBJFile(const std::string &fileName,
                                      vec3<T> startingPos, T mass) {
    std::ifstream in(fileName);

    std::string s;
    while (in >> s){
        if (s == "v"){
            T d1, d2, d3;
            d1 = d2 = d3 = (T)0.0;
            in >> d1;
            in >> d2;
            in >> d3;
            Eigen::Matrix<T, 3, 1> vertex;
            vertex << d1 + startingPos[0],
                      d2 + startingPos[1],
                      d3 + startingPos[2];

            // velocity is assumed to be 0.
            Eigen::Matrix<T, 3, 1> velocity;
            velocity << T(0), T(0), T(0);

            positions.push_back(vertex);
            projectedPositions.push_back(vertex);
            velocities.push_back(velocity);
            masses.push_back(mass);
            numParticles++;
        }
        else if (s == "f"){
            std::vector<int> v(3);
            v[1] = v[2] = v[3] = 0;
            in >> v[1];
            in >> v[2];
            in >> v[3];

            std::sort(v.begin(), v.end());

            // do -1 here because OBJ faces start at index 1,
            // but c++ index start at 0
            faces.emplace_back(TriangleFace(v[1] - 1, v[2] - 1, v[3] - 1));
        }
    }
}


// writes particle data into a file
template <class T>
void TriangleMesh<T>::writeToBGEOFile(const std::string &fileName) const {
    Partio::ParticlesDataMutable* parts = Partio::create();
    Partio::ParticleAttribute positionHandle, velocityHandle, massHandle;
    massHandle = parts->addAttribute("mass", Partio::VECTOR, 1);
    positionHandle = parts->addAttribute("position", Partio::VECTOR, 3);
    velocityHandle = parts->addAttribute("velocity", Partio::VECTOR, 3);
    for (int i = 0; i < numParticles; i++){
        int idx = parts->addParticle();
        float* m = parts->dataWrite<float>(massHandle, idx);
        float* p = parts->dataWrite<float>(positionHandle, idx);
        float* v = parts->dataWrite<float>(velocityHandle, idx);
        m[0] = masses[i];
        for (int k = 0; k < 3; k++)
            p[k] = positions[i][k];
        for (int k = 0; k < 3; k++)
            v[k] = velocities[i][k];
    }

    Partio::write(fileName.c_str(), *parts);
    parts->release();
}


// adds the specified constraint to the list of constraints acting on this mesh
template <class T>
void TriangleMesh<T>::addConstraint(ConstraintType type, double input) {
    if (type == ConstraintType::distance){
        // use a set to parse edges
        std::set<Edge> edgeSet;
        for(size_t i = 0; i < faces.size(); i++) {
            Edge e1 = Edge(faces[i][0], faces[i][1]);
            Edge e2 = Edge(faces[i][0], faces[i][2]);
            Edge e3 = Edge(faces[i][1], faces[i][2]);

            edgeSet.insert(e1);
            edgeSet.insert(e2);
            edgeSet.insert(e3);
        }

        for(auto it = edgeSet.begin(); it != edgeSet.end(); it++) {
            //get positions
            Eigen::Matrix<T, 3, 1> pi = positions[(*it).start];
            Eigen::Matrix<T, 3, 1> pj = positions[(*it).end];

            //make edge vector
            Eigen::Matrix<T, 3, 1> n = ( pi.col(0) - pj.col(0) );

            //get initial length
            T restLength = n.col(0).norm();

            constraints.emplace_back(
                    new DistanceConstraint<T>(*it, restLength, input));
        }
    } else if (type == ConstraintType::bending){
        std::map< Edge, std::vector<TriangleFace> > wingEdges;

        // map edges to all of the faces to which they are connected
        for(auto& face: faces){
            std::vector<int> pts(3);
            for(size_t i=0;i<3;i++) pts[i]=face[i];

            Edge currentEdge = Edge(face[0], face[1]);
            wingEdges[ currentEdge ].push_back(face);

            currentEdge = Edge(face[0], face[2]);
            wingEdges[ currentEdge ].push_back(face);

            currentEdge = Edge(face[1], face[2]);
            wingEdges[ currentEdge ].push_back(face);
        }

        // wingEdges are edges with 2 occurences,
        // so we need to remove the lower frequency ones
        auto it = wingEdges.begin();
        while(it !=wingEdges.end() ){
            if (it->second.size()<2) { it= wingEdges.erase(it); }
            else { it++; }
        }

        for(const auto& it: wingEdges ){
            /* wingEdges are indexed like in the Bridson,
             * Simulation of Clothing with Folds and Wrinkles paper
             *    3
             *    ^
             * 0  |  1
             *    2
             */

            //get the wing edge
            Edge wingEdge = it.first;
            std::vector<TriangleFace> wingTris = it.second;

            std::vector<int> indeces(4);

            Eigen::Matrix<T, 3, 4> p;

            p.col(2) = positions[wingEdge.start];
            p.col(3) = positions[wingEdge.end];

            indeces[2]=wingEdge.start;
            indeces[3]=wingEdge.end;

            int b=0;
            for(const auto& tri : it.second) {
                for(int i=0; i<3; i++){
                    auto point = tri[i];
                    if (point!=indeces[3] && point!=indeces[2]){
                        //tri #1
                        if(b==0){
                            indeces[0]=point;
                            p.col(0)=positions[point];
                            break;
                        }
                        //tri #2
                        else if(b==1){
                            indeces[1]=point;
                            p.col(1)=positions[point];
                            break;
                        }
                    }
                }
                b++;
            }

            auto n1 = (p.col(2) - p.col(0)).cross(p.col(3) - p.col(0));
            auto n2 = (p.col(3) - p.col(1)).cross(p.col(2) - p.col(1));
            n1.normalize();
            n2.normalize();

            auto d = n1.dot(n2);
            if(d<-1.0) d = -1.0;
            if(d>1.0)  d = 1.0;
            auto restAngle = acos(d);

            constraints.emplace_back(
                    new BendingConstraint<T>(indeces, restAngle, input));
        }
    }
    else if(type == ConstraintType::ground){
        delete groundConstraint;
        groundConstraint = new GroundConstraint<T>(input);
    }
}


// adds new point constraints
template <class T>
void TriangleMesh<T>::addPointConstraint(PointConstraintType type, int width,
                                         int height, float distance,
                                         float duration) {
    if (type == PointConstraintType::topCorners){
        pointConstraints.emplace_back(
                new PointConstraint<T>(0, positions[0]));
        pointConstraints.emplace_back(
                new PointConstraint<T>(width, positions[width]));
    }
    else if (type == PointConstraintType::leftCorners){
        pointConstraints.emplace_back(
                new PointConstraint<T>(0, positions[0]));
        pointConstraints.emplace_back(
                new PointConstraint<T>(height * (width + 1),
                                       positions[height * (width + 1)]));
    }
    else if (type == PointConstraintType::topRow){
        for(int i = 0; i <= width; i++){
            pointConstraints.emplace_back(
                    new PointConstraint<T>(i, positions[i]));
        }
    }
    else if (type == PointConstraintType::leftRow){
        for(int i = 0; i <= width; i++){
            pointConstraints.emplace_back(
                    new PointConstraint<T>(i * (width + 1),
                                           positions[i * (width + 1)]));
        }
    }
    else if (type == PointConstraintType::curtainRight) {
        pointConstraints.emplace_back(
                new PointConstraint<T>(0, positions[0]));

        vec3<T> v;
        v << 0, 0, distance / duration;
        pointConstraints.emplace_back(
                new PointConstraint<T>(width, positions[width], v, duration));
    }
}


// step 5: add external forces to all velocities
template <class T>
void TriangleMesh<T>::applyExternalForce(vec3<T> force, double dt) {
    // velocity += dt * weight * externalForce(currentPos)
    for (int i = 0; i < numParticles; i++){
        velocities[i] += force * masses[i] * dt;
    }
}


// step 6: apply damping to velocity
template <class T>
void TriangleMesh<T>::dampVelocity(double stiffness) {
    // dumb way
    /*
    for (auto& v : velocities){
        v *= 0.98;
    }
    */

    // smart way
    // first compute the center of mass's position and velocity
    vec3<T> centerMassPosition, centerMassVelocity;
    centerMassPosition << 0, 0, 0;
    centerMassVelocity << 0, 0, 0;
    T massSum = (T)0;
    T mass = masses[0];

    for(auto i = 0; i < numParticles; i++){
        massSum += mass;
        centerMassPosition += positions[i] * mass;
        centerMassVelocity += velocities[i] * mass;
    }
    centerMassPosition /= massSum;
    centerMassVelocity /= massSum;

    // now compute L = sum of all r cross mass * velocity
    // also compute I = sum of rs * rs_transpose * mass
    vec3<T> L;
    Eigen::Matrix<T, 3, 3> I;
    L << 0, 0, 0;
    I << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    for(auto i = 0; i < numParticles; i++){
        //  r is position - center of mass
        vec3<T> r = positions[i] - centerMassPosition;
        L += r.cross(mass * velocities[i]);
        // rs = [ 0      -r.z     r.y  ]
        //        r.z     0      -r.x
        //       -r.y     r.x     0
        Eigen::Matrix<T, 3, 3> rs;
        rs <<       0, -r[2],  r[1],
                r[2],     0, -r[0],
                -r[1],  r[0],     0;
        I += rs * rs.transpose() * mass;
    }

    // w = I_inverse * L
    Eigen::Matrix<T, 3, 3> I_inv = I.inverse();
    vec3<T> w = I_inv * L;

    // apply w back into velocities
    for(auto i = 0; i < numParticles; i++){
        vec3<T> r = positions[i] - centerMassPosition;
        vec3<T> dv = centerMassVelocity + w.cross(r) - velocities[i];
        velocities[i] += stiffness * dv;
    }
}


// step 7: compute projected positions using explicit euler
template <class T>
void TriangleMesh<T>::applyExplicitEuler(double dt) {
    // explicit euler: projectedPos = currentPos + velocity * dt
    for (auto i = 0; i < numParticles; i++){
        projectedPositions[i] = positions[i] + velocities[i] * dt;
    }
}

// step 8 pt1: clear all current collision constraints
template <class T>
void TriangleMesh<T>::clearCollisionConstraints() {
    for (auto i = 0u; i < collisionConstraints.size(); i++){
        delete collisionConstraints[i];
    }
    collisionConstraints.clear();
}


// step 8 pt2: generate new collision constraints with the given object
template <class T>
void TriangleMesh<T>::generateCollisionConstraints(const Sphere<T> &sphere) {
    // check all points in projectedPoints,
    // add collision if any of them intersects the sphere
    for(auto i = 0; i < numParticles; i++){
        if (sphere.isColliding(projectedPositions[i])){
            collisionConstraints.emplace_back(
                    new CollisionConstraint<T>(i, positions[i],
                                               projectedPositions[i], sphere));
        }
    }
}


// step 9-11: satisfy all constraints in a random order
template <class T>
void TriangleMesh<T>::satisfyConstraints(){
    // randomly shuffle constraints
    std::random_shuffle(constraints.begin(), constraints.end());
    std::random_shuffle(collisionConstraints.begin(),
                        collisionConstraints.end());

    // satisfy normal constraints first
    for(auto i = 0u; i < constraints.size(); i++){
        constraints[i]->satisfy(projectedPositions, velocities, masses);
    }

    // then satisfy collision constraints
    for(auto i = 0u; i < collisionConstraints.size(); i++){
        collisionConstraints[i]->satisfy(projectedPositions, velocities, masses);
    }

    // finall, satisfy ground constraints
    if (groundConstraint != nullptr){
        groundConstraint->satisfy(projectedPositions, velocities, masses);
    }
}


// satisfies all point constraints
template <class T>
void TriangleMesh<T>::satisfyPointConstraints(double dt) {
    for(auto& c : pointConstraints){
        c->satisfy(projectedPositions, positions, dt);
    }
}


// step 13 and 14: update positions and velocities with projectedPositions
template <class T>
void TriangleMesh<T>::updateVertices(double dt) {
    for (auto i = 0; i < numParticles; i++){
        // step 13: velocity = (projectedPos - currentPos) / dt
        velocities[i] = (projectedPositions[i] - positions[i]) / dt;
        // step 14: currentPos = projectedPos
        positions[i] = projectedPositions[i];
    }
}


// step 16: applies friction to all velocities
template <class T>
void TriangleMesh<T>::applyFriction() {
    // do simple air resistance by reducing speed of every vertex
    for (auto i = 0; i < numParticles; i++){
        velocities[i] *= 0.998;
    }
    // do simple ground friction by setting speed of objects on ground to zero
    for (auto i = 0; i < numParticles; i++){
        if (positions[i][1] == 0){
            velocities[i][0]*= .9;
            velocities[i][2]*= .9;
            if (std::abs(velocities[i][0] < 0.2)) velocities[i][0] = 0;
            if (std::abs(velocities[i][2] < 0.2)) velocities[i][2] = 0;
        }
    }
}


template class TriangleMesh<double>;
template class TriangleMesh<float>;