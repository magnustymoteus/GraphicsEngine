//
// Created by magnustymoteus on 22/03/23.
//

#include "Canvas2DUtils.h"
#include "Line2D.h"

double Canvas2DUtils::degrees_to_radians(const double &degrees) {
    return degrees * (PI / 180.0f);
}
std::pair<Point2D, Point2D> Canvas2DUtils::find_minimal_and_maximal_coordinates(
        const std::list<std::shared_ptr<Point2D>> &points) {
    Point2D currentMin {0,0};
    Point2D currentMax {0,0};
    for(auto& currentPoint : points) {
        currentMin.x = std::min(currentMin.x, currentPoint->x);
        currentMin.y = std::min(currentMin.y, currentPoint->y);

        currentMax.x = std::max(currentMax.x, currentPoint->x);
        currentMax.y = std::max(currentMax.y, currentPoint->y);
    }
    return {currentMin, currentMax};
}
img::EasyImage Canvas2DUtils::generate_blank_image(const int &width, const int &height) {
    img::EasyImage img = img::EasyImage(width, height);
    return img;
}
void Canvas2DUtils::draw_background(img::EasyImage &img, const Color &color) {
    for(int i=0;i<img.get_height();i++) {
        for(int j=0;j<img.get_width();j++) {
            const Point2D currentPoint = Point2D(j, i);
            const int x = static_cast<int>(lround(currentPoint.x)), y = static_cast<int>(lround(currentPoint.y));
            img(x, y) = img::Color(color.red_scaled, color.green_scaled, color.blue_scaled);
        }
    }
}