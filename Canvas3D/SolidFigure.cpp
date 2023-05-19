//
// Created by magnustymoteus on 31/03/23.
//

#include "SolidFigure.h"
#include "Canvas3DUtils.h"

void SolidFigure::createSphere(const int &n) {
    createIcosahedron();
    std::vector<Face> new_faces;
    for(int i=0;i<n;i++) Canvas3DUtils::divideAllTriangles(*this);
    for(Vector3D &currentPoint : points) {
        currentPoint = Vector3D::normalise(currentPoint);
    }
}
void SolidFigure::createCone(const int &n, const double &h) {
    clear();
    for(int i=0;i<n;i++) {
        points.push_back(Vector3D::point(cos((2*i*PI)/n), sin((2*i*PI)/n), 0));
    }
    points.push_back(Vector3D::point(0,0,h));
    for(int i=0;i<n;i++) {
        Face face;
        face.point_indexes =  {i, (i+1)%n, n};
        faces.push_back(face);
    }
    Face lastFace;
    for(int i=n-1;i>=0;i--) {
        lastFace.point_indexes.push_back(i);
    }
    faces.push_back(lastFace);
}
void SolidFigure::createTorus(const int &r, const double &R, const int &n, const int &m) {
    for(int i=0;i<n;i++) {
        for(int j=0;j<m;j++) {
            const double u = (2*i*PI)/n;
            const double v = (2*j*PI)/m;
            const double x = (R+r*cos(v))*cos(u);
            const double y = (R+r*cos(v))*sin(u);
            const double z = r*sin(v);
            points.push_back(Vector3D::point(x,y,z));

            Face face;
            face.point_indexes = {i*n+j, ((i+1)%n)*n+j, ((i+1)%n)*n+((j+1)%m), i*n+((j+1)%m)};
            faces.push_back(face);
        }
    }
}

void SolidFigure::createCylinder(const int &n, const double &h) {
    for(int i=0;i<n;i++) {
        points.push_back(Vector3D::point(cos((2*i*PI)/n), sin((2*i*PI)/n), 0));
    }
    for(int i=0;i<n;i++) {
        points.push_back(Vector3D::point(cos((2*i*PI)/n), sin((2*i*PI)/n), h));
    }
    for(int i=0;i<n;i++) {
        Face face;
        face.point_indexes =  {i, (i+1)%n, n+(i+1)%n, n+i};
        faces.push_back(face);
    }
    Face face;
    for(int i=0;i<n;i++) {
        face.point_indexes.push_back(i);
    }
    Face face2;
    for(int i=0;i<n;i++) {
        face2.point_indexes.push_back(n+i);
    }
    faces.push_back(face);
    faces.push_back(face2);
}