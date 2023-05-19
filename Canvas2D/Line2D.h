//
// Created by gruzi on 24/02/2023.
//

#ifndef ENGINE_LINE2D_H
#define ENGINE_LINE2D_H

#include "Point2D.h"
#include "Color.h"
#include <memory>
#include <list>

class Line2D {
public:
    std::shared_ptr<Point2D> p1, p2;
    std::shared_ptr<Color> color;
    double z1=0.0, z2=0.0;

    Line2D(Point2D p1, Point2D p2, Color color):
    p1(std::make_shared<Point2D>(p1)), p2(std::make_shared<Point2D>(p2)), color(std::make_shared<Color>(color)) {}

    Line2D(Point2D p1, Point2D p2, Color color, double z1, double z2) :
    p1(std::make_shared<Point2D>(p1)), p2(std::make_shared<Point2D>(p2)), color(std::make_shared<Color>(color)),
    z1(z1), z2(z2) {}

    Line2D(std::shared_ptr<Point2D> &p1_ptr, std::shared_ptr<Point2D> &p2_ptr, std::shared_ptr<Color> &color_ptr) :
    p1(p1_ptr), p2(p2_ptr), color(color_ptr) {}

    Line2D(std::shared_ptr<Point2D> &p1_ptr, std::shared_ptr<Point2D> &p2_ptr, std::shared_ptr<Color> &color_ptr,
           const double &z1, const double &z2) :
            p1(p1_ptr), p2(p2_ptr), color(color_ptr),
            z1(z1), z2(z2) {}

    Point2D operator[] (const int &i) {return (!i)?*p1:*p2;}

};

typedef std::list<Line2D> Lines2D;

#endif //ENGINE_LINE2D_H