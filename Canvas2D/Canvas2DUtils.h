//
// Created by magnustymoteus on 22/03/23.
//

#ifndef ENGINE_CANVAS2DUTILS_H
#define ENGINE_CANVAS2DUTILS_H

#include "tools/ini_configuration.h"
#include "tools/easy_image.h"
#include "Line2D.h"
#include "Canvas3D/ZBuffer.h"

namespace Canvas2DUtils {
    #define PI 3.14159265358979323846
    img::EasyImage generate_blank_image(const int &width, const int &height);
    void draw_background(img::EasyImage &img, const Color &color);
    double degrees_to_radians(const double &degrees);
    std::pair<Point2D, Point2D> find_minimal_and_maximal_coordinates(const std::list<std::shared_ptr<Point2D>> &points);
};


#endif //ENGINE_CANVAS2DUTILS_H
