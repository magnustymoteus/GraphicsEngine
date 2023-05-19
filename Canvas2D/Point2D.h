//
// Created by gruzi on 23/04/2023.
//

#ifndef ENGINE_POINT2D_H
#define ENGINE_POINT2D_H



class Point2D {
public:
    double x, y;
    double operator[] (const int &i) const {return (!i)?x:y;}
    Point2D(const double &x, const double &y): x(x), y(y) {}
};


#endif //ENGINE_POINT2D_H
