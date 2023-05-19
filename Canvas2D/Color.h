//
// Created by gruzi on 24/02/2023.
//

#ifndef ENGINE_COLOR_H
#define ENGINE_COLOR_H


#include <cmath>
#include "tools/ini_configuration.h"

class Color {
public:
    double red, green, blue;
    int red_scaled, green_scaled, blue_scaled;
    Color(const double &red, const double &green, const double &blue) : red(red), green(green), blue(blue),
    red_scaled(lround(red*255)), green_scaled(lround(green*255)), blue_scaled(lround(blue*255)) {};

    Color(const ini::DoubleTuple &colorTuple) : red(colorTuple[0]), green(colorTuple[1]), blue(colorTuple[2]),
    red_scaled(lround(colorTuple[0]*255)), green_scaled(lround(colorTuple[1]*255)),
    blue_scaled(lround(colorTuple[2]*255)){}

};

#endif //ENGINE_COLOR_H