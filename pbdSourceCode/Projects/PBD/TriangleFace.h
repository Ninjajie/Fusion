/**
    Simple class that contains the 3 vertices of a triangle. Vertices are stored
        in terms of indices of the corresponding TriangleMesh's 'positions'
        vector.
 */

#ifndef CISPBA_TRIANGLEFACE_H
#define CISPBA_TRIANGLEFACE_H

#include <algorithm>


class TriangleFace{
public:
    TriangleFace() 
    {
        vertex[0] = -1;
        vertex[1] = -1;
        vertex[2] = -1;
    };

    TriangleFace(int v1, int v2, int v3)
    {
        vertex[0] = v1;
        vertex[1] = v2;
        vertex[2] = v3;

        std::sort(std::begin(vertex), std::end(vertex));
    };

    ~TriangleFace() {};
    int operator[](int i) const
    {
        if (i > 2){
            throw "INDEX OUT OF BOUNDS";
        }
        else{
            return vertex[i];
        }
    };

    bool operator<(const TriangleFace& right) const{
        return (this->vertex[0]<right[0]
                || (this->vertex[0]==right[0] && this->vertex[1]<right[1])
                ||  (this->vertex[0]==right[0] && this->vertex[1]==right[1]
                     && this->vertex[2]<right[2])
               );
    };

    //assuming properly sorted face vertex order
    bool operator==(const TriangleFace& right) const {
        return (this->vertex[0]==right[0] && this->vertex[1]==right[1]
                && this->vertex[2]==right[2]);
    };

    friend std::ostream& operator<<(std::ostream &strm,
                                    const TriangleFace& face) {
        return strm << "Face(" << face[0] << " "
                    << face[1] << " " << face[2] << ")";
    }

private:
    int vertex[3];
};




#endif //CISPBA_TRIANGLEFACE_H
