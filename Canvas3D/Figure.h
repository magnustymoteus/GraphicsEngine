//
// Created by magnustymoteus on 21/03/23.
//

#ifndef ENGINE_FIGURE_H
#define ENGINE_FIGURE_H

#include "tools/vector3d.h"
#include "Canvas2D/Color.h"
#include "Face.h"

#include <vector>
#include <list>

class Figure {
public:
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Color color = Color(0,0,0);

    Figure(std::vector<Vector3D> &points, std::vector<Face> &faces, Color &color) :
    points(points), faces(faces), color(color) {}

    Figure() = default;

    void clear() {points.clear();faces.clear();}
};

typedef std::list<Figure> Figures3D;

#endif //ENGINE_FIGURE_H
