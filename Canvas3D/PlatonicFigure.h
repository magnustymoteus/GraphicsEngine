//
// Created by magnustymoteus on 28/03/23.
//

#ifndef ENGINE_PLATONICFIGURE_H
#define ENGINE_PLATONICFIGURE_H

#include "Figure.h"
#define PI 3.14159265358979323846


class PlatonicFigure : public Figure {
private:
    void addPoints(std::vector<std::vector<double>> &new_points);
    void addFaces(std::vector<std::vector<int>> &pi);
public:
    void createCube();
    void createTetrahedron();
    void createOctahedron();
    void createIcosahedron();
    void createDodecahedron();
    void make(const std::string &type);
};


#endif //ENGINE_PLATONICFIGURE_H
