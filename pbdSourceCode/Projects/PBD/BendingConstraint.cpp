#include "BendingConstraint.h"

using std::cout; using std::endl;

// takes in projected positions and vertex mass,
// satisfy the projected positions and velocities according to this constraint
template <class T>
void BendingConstraint<T>::satisfy(std::vector<vec3<T> >& projectedPositions,
                                   std::vector<vec3<T> >& velocities,
                                   const std::vector<T>& masses) {
    /* this is indexed like the Bridson, Simulation of Clothing with Folds
     *     and Wrinkles paper
     *    3
     *    ^
     * 0  |  1
     *    2 
     */

    if (!(masses[ vertexIndeces[0] ]==0.0 && masses[ vertexIndeces[1] ]==0.0)){
        Eigen::Matrix<T, 3, 4> p;

        p.col(0) = projectedPositions[ vertexIndeces[0] ];
        p.col(1) = projectedPositions[ vertexIndeces[1] ];
        p.col(2) = projectedPositions[ vertexIndeces[2] ];
        p.col(3) = projectedPositions[ vertexIndeces[3] ];

        vec3<T> wing = p.col(3) - p.col(2);
        auto wingLength = wing.norm();

        if( wingLength >= std::numeric_limits<T>::epsilon() ){
            //get normals
            auto n1 = (p.col(2) - p.col(0)).cross(p.col(3) - p.col(0));
            n1 /= n1.squaredNorm();

            auto n2 = (p.col(3) - p.col(1)).cross(p.col(2) - p.col(1));
            n2 /= n2.squaredNorm();
            //unlike in the original PBD paper,
            // both normals point in same direction

            //getting constraints along gradients (gradpC)
            auto invWingLength = 1.0/wingLength;
        
            Eigen::Matrix<T, 3, 4> q;
            q.col(0) = wingLength*n1;

            q.col(1) = wingLength*n2;

            q.col(2) = (p.col(0)-p.col(3)).dot(wing) * invWingLength * n1
                        + (p.col(1)-p.col(3)).dot(wing) * invWingLength * n2;

            q.col(3) = (p.col(2)-p.col(0)).dot(wing) * invWingLength * n1
                        + (p.col(2)-p.col(1)).dot(wing) * invWingLength * n2;

            //find current angle
            n1.normalize();
            n2.normalize();

            auto d = n1.dot(n2);
            if(d<-1.0) d = -1.0;
            if(d>1.0)  d = 1.0;
            auto currentAngle = acos(d);

            //find lamda ( where deltap = lamda*wi*gradConstraint )
            auto lamda = 0;
            for(int i=0; i<4; i++ ){
                lamda += (masses[ vertexIndeces[i] ] ) * q.col(i).squaredNorm();
            }

            if (lamda!=0.0){
                lamda = ( currentAngle - restAngle ) / lamda * weight;

                if ( n1.cross(n2).dot(wing) > 0.0 ) lamda = -lamda;

                for(int i=0; i<4; i++) {
                    projectedPositions[ vertexIndeces[i] ]
                        -= (masses[ vertexIndeces[i] ] ) * lamda * q.col(i);
                }
            }
        }
    }
}


template class BendingConstraint<double>;
template class BendingConstraint<float>;