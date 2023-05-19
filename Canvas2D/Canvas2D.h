//
// Created by gruzi on 24/02/2023.
//

#ifndef ENGINE_CANVAS2D_H
#define ENGINE_CANVAS2D_H

#include "tools/ini_configuration.h"
#include "tools/easy_image.h"
#include "tools/l_parser.h"
#include "Canvas2D/Line2D.h"
#include "Canvas3D/ZBuffer.h"
#include <list>
#include <stack>

class Color;
class Point2D;
class Line2D;

class Canvas2D {
protected:
    std::list<std::shared_ptr<Point2D>> points;
    std::list<std::shared_ptr<Line2D>> lines;
    std::list<std::shared_ptr<Color>> colors;

    void multiply_with_points(const double &factor);
    void add_to_points(const double &x, const double &y);
public:

    img::EasyImage generate_blank_image_centered(const int &size);
    img::EasyImage generate_2dlsystem(const ini::Configuration &configuration);
    void generate_lines_from_lsystem(const LParser::LSystem2D &l_system, const Color &color);
    void add_lines(const Lines2D &new_lines);
    void add_line(const Line2D &new_line);

    void draw_lines(img::EasyImage &img);

    void draw_lines(img::EasyImage &img, ZBuffer &zbuffer);

    void clear();
};


#endif //ENGINE_CANVAS2D_H
