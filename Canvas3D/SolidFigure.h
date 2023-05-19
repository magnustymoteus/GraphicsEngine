//
// Created by magnustymoteus on 31/03/23.
//

#ifndef ENGINE_SOLIDFIGURE_H
#define ENGINE_SOLIDFIGURE_H

#include "PlatonicFigure.h"

class SolidFigure : public PlatonicFigure {
public:
    void createSphere(const int &n);
    void createCone(const int &n, const double &h);
    void createCylinder(const int &n, const double &h);
    void createTorus(const int &r, const double &R, const int &n, const int &m);
};


#endif //ENGINE_SOLIDFIGURE_H
