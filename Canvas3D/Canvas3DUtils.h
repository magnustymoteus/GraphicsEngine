//
// Created by magnustymoteus on 22/03/23.
//

#ifndef ENGINE_CANVAS3DUTILS_H
#define ENGINE_CANVAS3DUTILS_H

#include "tools/vector3d.h"
#include "tools/ini_configuration.h"
#include "Figure.h"
#include "Canvas2D/Line2D.h"
#include "ZBuffer.h"
#include "tools/easy_image.h"

namespace Canvas3DUtils {
    Matrix scaleFigure(const double &scale);
    Matrix rotateX(const double &angle);
    Matrix rotateY(const double &angle);
    Matrix rotateZ(const double &angle);
    Matrix translate(const Vector3D &vector);
    void applyTransformation(Figure &fig, const Matrix &m);
    Matrix eyePointTrans(const Vector3D &eyepoint);
    void toPolar(const Vector3D &point, double &theta, double &phi, double &r);
    void applyTransformation(Figures3D &figs, const Matrix &m);
    Lines2D doProjection(const Figures3D &figs);
    Point2D doProjection(const Vector3D &point, const double &d=1);
    Vector3D tupleToPoint(const ini::DoubleTuple &pointTuple);
    void divideAllTriangles(Figure &fig);
    std::vector<Face> triangulate(const Face &face);
    void draw_zbuf_triag(ZBuffer &zbuf, img::EasyImage &img,
                         Vector3D const & A,
                         Vector3D const & B,
                         Vector3D const & C,
                         const double &d, const double &dx, const double &dy, Color color
    );
};


#endif //ENGINE_CANVAS3DUTILS_H
