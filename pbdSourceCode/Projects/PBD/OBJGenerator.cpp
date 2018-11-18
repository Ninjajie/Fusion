#include "OBJGenerator.h"

using namespace std;
using namespace Eigen;

// generates a cloth mesh where each point is vertexDist apart and in the shape of
// 1 -- 2 -- 3
// | \  | \  |
// |  \ |  \ |
// 4 -- 5 -- 6
// width is the number of blocks across. height is the number of blocks down.
// starting position dictates the starting position of vertex 1
void OBJGenerator::generateClothOBJ(const std::string &fileName,
                                    double vertexDist,
                                    int width, int height,
                                    OBJGeneratorMode mode){
    std::string fullName = fileName + ".obj";
    std::ofstream out(fullName);

    vector<Vector3d> positions;

    if (mode == OBJGeneratorMode::vertical){
        for (int i = 0; i <= height; i++){
            for (int j = 0; j <= width; j++){
                Vector3d pos;
                pos <<  0,
                        0 - i * vertexDist,
                        0 - j * vertexDist;
                positions.push_back(pos);
            }
        }
    }
    else if (mode == OBJGeneratorMode::horizontal){
        for (int i = 0; i <= height; i++){
            for (int j = 0; j <= width; j++){
                Vector3d pos;
                pos <<  0 - i * vertexDist,
                        0,
                        0 - j * vertexDist;
                positions.push_back(pos);
            }
        }
    }


    vector<Vector3i> faces;
    for (int i = 0; i < height; i++){
        for (int j = 0; j < width; j++){
            Vector3i face1, face2;
            face1 << j + i * (width + 1) + 1,
                    (j + 1) + i * (width + 1) + 1,
                    (j + 1) + (i + 1) * (width + 1) + 1;
            face2 << j + i * (width + 1) + 1,
                    (j + 1) + (i + 1) * (width + 1) + 1,
                    j + (i + 1) * (width + 1) + 1;
            faces.push_back(face1);
            faces.push_back(face2);
        }
    }

    out << "# cloth obj file with vertexDist of " << vertexDist << ". mesh is "
            << width << " across and " << height << " down." << std::endl << std::endl;

    // first write all the vector data
    for (auto& p : positions){
        out << "v  " << std::to_string(p[0]);
        out << "  " << std::to_string(p[1]);
        out << "  " << std::to_string(p[2]) << std::endl;
    }
    out << std::endl;

    // then write all the face data
    for (auto& f : faces){
        out << "f  " << std::to_string(f[0]);
        out << "  " << std::to_string(f[1]);
        out << "  " << std::to_string(f[2]) << std::endl;
    }
    out.close();
}