// simple class used to generate useful OBJ files

#ifndef CISPBA_OBJGENERATOR_H
#define CISPBA_OBJGENERATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

enum OBJGeneratorMode{
    horizontal,
    vertical
};

class OBJGenerator {
public:
    static void generateClothOBJ(const std::string& fileName,
                                 double vertexDist,
                                 int width,
                                 int height,
                                 OBJGeneratorMode mode);
};


#endif //CISPBA_OBJGENERATOR_H
