/**
    This class represents a single edge in the mesh.

    The indices it stores are the index of its start and end points in
        its corresponding TriangleMesh's 'positions' vector

    The Edge class overrides the == and < operators, which allows it to be
        used in a stl set.
 */

#ifndef CISPBA_EDGE_H
#define CISPBA_EDGE_H

class Edge{
public:
    int start;
    int end;

    Edge(){};
    Edge(int s, int e) : start(s), end(e) {};
    Edge(const Edge& e) : start(e.start), end(e.end) {};

    Edge& operator=(const Edge& e){
        start = e.start;
        end = e.end;
        return *this;
    }


    bool operator== (Edge other) const{
        return (this->start == other.start &&
               this->end == other.end);
    }

    bool operator< (Edge other) const{
        return (this->start < other.start || (this->start == other.start && this->end < other.end) );
    }

};


#endif //CISPBA_EDGE_H
