//
// Created by magnustymoteus on 22/03/23.
//

#include "Canvas3DUtils.h"
#include "Canvas2D/Canvas2DUtils.h"
#include <cmath>
#include <limits>

bool operator== (const Point2D &p1, const Point2D &p2) {
    return p1.x == p2.x && p1.y == p2.y;
}

std::vector<Face> Canvas3DUtils::triangulate(const Face &face) {
    std::vector<Face> new_faces;
    for(int i=1;i<=face.point_indexes.size()-2;i++) {
        Face new_face;
        new_face.point_indexes = {face.point_indexes[0], face.point_indexes[i], face.point_indexes[i+1]};
        new_faces.push_back(new_face);
    }
    return new_faces;
}
void Canvas3DUtils::draw_zbuf_triag(ZBuffer &zbuf, img::EasyImage &img, const Vector3D &A, const Vector3D &B,
                               const Vector3D &C, const double &d, const double &dx, const double &dy, Color color) {
    Point2D A_projected = Point2D(((d*A.x)/-A.z)+dx, ((d*A.y)/-A.z)+dy);
    Point2D B_projected = Point2D(((d*B.x)/-B.z)+dx, ((d*B.y)/-B.z)+dy);
    Point2D C_projected = Point2D(((d*C.x)/-C.z)+dx, ((d*C.y)/-C.z)+dy);

    const double x_G = (A_projected.x+B_projected.x+C_projected.x)/3;
    const double y_G = (A_projected.y+B_projected.y+C_projected.y)/3;
    const double z_G_inv = (1/(3*A.z))+(1/(3*B.z))+(1/(3*C.z));
    const Vector3D u = B-A;
    const Vector3D v = C-A;
    const Vector3D w = Vector3D::cross(u,v);
    const double k = (w.x*A.x)+(w.y*A.y)+(w.z*A.z);
    const double dzdx = w.x/(-d*k), dzdy = w.y/(-d*k);

    double y_min_d=A_projected.y, y_max_d=A_projected.y;
    for(const auto& curr_projected : {A_projected, B_projected, C_projected}) {
        if(curr_projected.y < y_min_d) y_min_d = curr_projected.y;
        if(curr_projected.y > y_max_d ) y_max_d = curr_projected.y;
    }
    double x_L_AB, x_L_AC, x_L_BC;
    double x_R_AB, x_R_AC, x_R_BC;
    const int y_min=(int)lround(y_min_d+0.5), y_max = (int)lround(y_max_d-0.5);
    for(int y_i=y_min;y_i<=y_max;y_i++) {
        x_L_AB = x_L_AC = x_L_BC = std::numeric_limits<double>::infinity();

        x_R_AB = x_R_AC = x_R_BC = -std::numeric_limits<double>::infinity();

        std::list<std::pair<Point2D, Point2D>> lijnstukken = {{A_projected,B_projected},
                                                              {A_projected,C_projected},
                                                              {B_projected,C_projected}};

        for(const auto& lijnstuk : lijnstukken) {
            const bool test_passed = (((y_i-lijnstuk.first.y)*(y_i-lijnstuk.second.y)) <= 0.0)
                                     && (lijnstuk.first.y != lijnstuk.second.y);
            if(test_passed) {
                const double x_i = lijnstuk.second.x + (lijnstuk.first.x - lijnstuk.second.x) *
                                                       ((y_i-lijnstuk.second.y) / (lijnstuk.first.y-lijnstuk.second.y));
                if(lijnstuk.first == A_projected && lijnstuk.second == B_projected) {
                    x_L_AB = x_R_AB = x_i;
                }
                else if(lijnstuk.first == A_projected && lijnstuk.second == C_projected) {
                    x_L_AC = x_R_AC = x_i;
                }
                else {
                    x_L_BC = x_R_BC = x_i;
                }
                double x_min_d = std::numeric_limits<double>::infinity(), x_max_d = -std::numeric_limits<double>::infinity();
                for(const auto& curr_projected : {x_L_AB, x_L_AC, x_L_BC}) {
                    if(x_min_d > curr_projected) x_min_d = curr_projected;
                }
                for(const auto& curr_projected : {x_R_AB, x_R_AC, x_R_BC}) {
                    if(x_max_d < curr_projected) x_max_d = curr_projected;
                }
                const int x_L = lround(x_min_d+0.5);
                const int x_R = lround(x_max_d-0.5);
                for(int i=x_L;i<=x_R;i++) {
                    const double z_inv = 1.0001f*z_G_inv+(i-x_G)*dzdx+(y_i-y_G)*dzdy;
                    if(z_inv < zbuf[y_i][i]) {
                        img(i, y_i) =
                                img::Color(color.red_scaled, color.green_scaled, color.blue_scaled);
                        zbuf[y_i][i] = z_inv;
                    }
                }
            }
        }
    }
}
void Canvas3DUtils::divideAllTriangles(Figure &fig) {
    std::vector<Face> new_faces;
    for (Face &currentFace : fig.faces) {
            if(currentFace.point_indexes.size() != 3) continue;
            Vector3D A = fig.points[currentFace.point_indexes[0]];
            Vector3D B = fig.points[currentFace.point_indexes[1]];
            Vector3D C = fig.points[currentFace.point_indexes[2]];

            Vector3D D = (A+B)/2;
            Vector3D E = (A+C)/2;
            Vector3D F = (B+C)/2;

            fig.points.push_back(D), fig.points.push_back(E), fig.points.push_back(F);

            const int Ai = currentFace.point_indexes[0], Bi = currentFace.point_indexes[1],
                    Ci = currentFace.point_indexes[2], Di = fig.points.size()-3, Ei = fig.points.size()-2,
                    Fi = fig.points.size()-1;

            currentFace.point_indexes[0] = Ai, currentFace.point_indexes[1]=Di, currentFace.point_indexes[2]=Ei;
            Face triangle2;
            triangle2.point_indexes = {Bi, Fi, Di};
            Face triangle3;
            triangle3.point_indexes = {Ci, Ei, Fi};
            Face triangle4;
            triangle4.point_indexes = {Di, Fi, Ei};

            new_faces.push_back(triangle2), new_faces.push_back(triangle3), new_faces.push_back(triangle4);
    }
    fig.faces.insert(fig.faces.end(), new_faces.begin(), new_faces.end());
}

Vector3D Canvas3DUtils::tupleToPoint(const ini::DoubleTuple &pointTuple) {
    return Vector3D::point(pointTuple[0], pointTuple[1], pointTuple[2]);
}

Matrix Canvas3DUtils::scaleFigure(const double &scale) {
    Matrix result;
    for(int i=0;i<3;i++) {
        result(i+1,i+1) = scale;
    }
    result(4,4) = 1;
    return result;
}

Matrix Canvas3DUtils::rotateX(const double &angle) {
    Matrix result;
    double cosine=cos(angle), sine=sin(angle);
    result(1,1)=1,result(4,4)=1;

    result(2,2)=cosine,result(3,3)=cosine;

    result(2,3)=sine,result(3,2)=-sine;


    return result;
}

Matrix Canvas3DUtils::rotateY(const double &angle) {
    Matrix result;
    double cosine=cos(angle), sine=sin(angle);

    result(1,1)=cosine, result(3,3)=cosine;

    result(4,4)=1, result(2,2)=1;

    result(1,3)=-sine,result(3,1)=sine;


    return result;
}

Matrix Canvas3DUtils::rotateZ(const double &angle) {
    Matrix result;
    double cosine=cos(angle), sine=sin(angle);

    result(1,1)=cosine, result(2,2)=cosine;
    result(1,2)=sine, result(2,1)=-sine;

    result(3,3)=1,result(4,4)=1;


    return result;
}

Matrix Canvas3DUtils::translate(const Vector3D &vector) {
    Matrix result;

    for(int i=0;i<4;i++) result(i+1,i+1)=1;

    result(4,1)=vector.x;
    result(4,2)=vector.y;
    result(4,3)=vector.z;

    return result;
}

void Canvas3DUtils::applyTransformation(Figure &fig, const Matrix &m) {
    for(Vector3D &currentVector3D : fig.points) {
        currentVector3D *= m;
    }
}
void Canvas3DUtils::toPolar(const Vector3D &point, double &theta, double &phi, double &r) {
    r = sqrt(pow(point.x,2)+pow(point.y,2)+pow(point.z,2));
    theta = atan2(point.y, point.x);
    phi = (r==0.0)? 0 : acos(point.z/r);
}
Matrix Canvas3DUtils::eyePointTrans(const Vector3D &eyepoint) {
    Matrix V;
    double theta, phi, r;
    toPolar(eyepoint, theta, phi, r);

    double cosTheta = cos(theta), cosPhi = cos(phi), sinTheta=sin(theta), sinPhi=sin(phi);

    V(1,1) = -sinTheta, V(2,1)=cosTheta;
    V(3,2)=sinPhi, V(3,3)=cosPhi;
    V(4,3)=-r,V(4,4)=1;
    V(1,2)=-cosTheta*cosPhi, V(1,3)=cosTheta*sinPhi;
    V(2,2)=-sinTheta*cosPhi, V(2,3)=sinTheta*sinPhi;

    return V;
}
void Canvas3DUtils::applyTransformation(Figures3D &figs, const Matrix &m) {
    for(Figure &currentFig: figs) {
        applyTransformation(currentFig, m);
    }
}
Lines2D Canvas3DUtils::doProjection(const Figures3D &figs) {
    Lines2D lines;
    for(const Figure &currentFig : figs) {
        for(const Face &currentFace : currentFig.faces) {
            Point2D lastNewPoint = doProjection(currentFig.points[currentFace.point_indexes[0]]);
            double z1=currentFig.points[currentFace.point_indexes[0]].z, z2;
            for(int i=1;i<=currentFace.point_indexes.size();i++) {
                Vector3D point = currentFig.points[currentFace.point_indexes[i % currentFace.point_indexes.size()]];
                Point2D newPoint = doProjection(point);
                z2 = point.z;

                Line2D line = Line2D(lastNewPoint, newPoint, currentFig.color, z1, z2);
                lastNewPoint = newPoint;
                z1 = z2;
                lines.push_back(line);
            }
        }
    }
    return lines;
}
Point2D Canvas3DUtils::doProjection(const Vector3D &point, const double &d) {
    const double new_x = (point.z == 0.0)?0.0:(d*point.x)/(-point.z);
    const double new_y = (point.z == 0.0)?0.0:(d*point.y)/(-point.z);
    Point2D new_point = Point2D(new_x, new_y);
    return new_point;
}

