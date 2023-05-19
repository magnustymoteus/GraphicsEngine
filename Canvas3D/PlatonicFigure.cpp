//
// Created by magnustymoteus on 28/03/23.
//

#include "PlatonicFigure.h"

void PlatonicFigure::addPoints(std::vector<std::vector<double>> &new_points) {
    for(std::vector<double> &currentVector : new_points) {
        Vector3D currentPoint = Vector3D::point(currentVector[0], currentVector[1], currentVector[2]);
        points.push_back(currentPoint);
    }
}
void PlatonicFigure::addFaces(std::vector<std::vector<int>> &pi) {
    for(const std::vector<int> &currentVector : pi) {
        Face face;
        face.point_indexes = currentVector;
        faces.push_back(face);
    }
}

void PlatonicFigure::createOctahedron() {
    clear();
    std::vector<std::vector<double>> new_points = {
            {1,0,0},
            {0,1,0},
            {-1,0,0},
            {0,-1,0},
            {0,0,-1},
            {0,0,1}
    };
    addPoints(new_points);
    std::vector<std::vector<int>> pi = {
            {0,1,5},
            {1,2,5},
            {2,3,5},
            {3,0,5},
            {1,0,4},
            {2,1,4},
            {3,2,4},
            {0,3,4}
    };
    addFaces(pi);
}

void PlatonicFigure::createTetrahedron() {
    clear();
    std::vector<std::vector<double>> new_points = {
            {1,-1,-1},
            {-1,1,-1},
            {1,1,1},
            {-1,-1,1},
    };
    addPoints(new_points);
    std::vector<std::vector<int>> pi = {
            { 0,1,2},
            { 1,3,2 },
            { 0,3,1},
            { 0,2,3 },
    };
    addFaces(pi);
}

void PlatonicFigure::createCube() {
    clear();
    std::vector<std::vector<double>> new_points = {
            {1,-1,-1},
            {-1,1,-1},
            {1,1,1},
            {-1,-1,1},
            {1,1,-1},
            {-1,-1,-1},
            {1,-1,1},
            {-1,1,1}
    };
    addPoints(new_points);
    std::vector<std::vector<int>> pi = {
            { 0, 4, 2, 6 },
             { 4, 1, 7, 2 },
             { 1, 5, 3, 7 },
            { 5, 0, 6, 3 },
             { 6, 2, 7, 3 },
            { 0, 5, 1, 4 }
    };
    addFaces(pi);
}
void PlatonicFigure::createIcosahedron() {
    clear();
    std::vector<std::vector<double>> new_points;
    for(int i=1;i<=12;i++) {
        if(i==1) new_points.push_back({0,0,sqrt(5)/2});
        else if(i>=2 && i<=6) new_points.push_back({cos((i-2)*2*PI/5), sin((i-2)*2*PI/5), 0.5});
        else if(i>=7 && i<=11) new_points.push_back({cos(PI/5+(i-7)*(2*PI)/5), sin(PI/5+(i-7)*(2*PI)/5),-0.5});
        else if(i==12) new_points.push_back({0,0,-sqrt(5)/2});
    }
    addPoints(new_points);
    std::vector<std::vector<int>> pi = {
            {0,1,2},
            {0,2,3},
            {0,3,4},
            {0,4,5},
            {0,5,1},
            {1,6,2},
            {2,6,7},
            {2,7,3},
            {3,7,8},
            {3,8,4},
            {4,8,9},
            {4,9,5},
            {5,9,10},
            {5,10,1},
            {1,10,6},
            {11,7,6},
            {11,8,7},
            {11,9,8},
            {11,10,9},
            {11,6,10}
    };
    addFaces(pi);
}
void PlatonicFigure::createDodecahedron() {
    createIcosahedron();
    std::vector<Vector3D> new_points;
    for(Face &current_face : faces) {
            const double x =
                    (points[current_face.point_indexes[0]].x+points[current_face.point_indexes[1]].x+points[current_face.point_indexes[2]].x)/3;

            const double y =
                    (points[current_face.point_indexes[0]].y+points[current_face.point_indexes[1]].y+points[current_face.point_indexes[2]].y)/3;

            const double z =
                    (points[current_face.point_indexes[0]].z+points[current_face.point_indexes[1]].z+points[current_face.point_indexes[2]].z)/3;
            Vector3D point = Vector3D::point(x,y,z);
            new_points.push_back(point);
    }
    clear();
    points = new_points;
    std::vector<std::vector<int>> pi = {
            {0,1,2,3,4},
            {0,5,6,7,1},
            {1,7,8,9,2},
            {2,9,10,11,3},
            {3,11,12,13,4},
            {4,13,14,5,0},
            {19,18,17,16,15},
            {19,14,13,12,18},
            {18,12,11,10,17},
            {17,10,9,8,16},
            {16,8,7,6,15},
            {15,6,5,14,19}
    };
    addFaces(pi);
}
void PlatonicFigure::make(const std::string &type) {
    if(type == "Cube") {
        createCube();
    }
    else if(type == "Tetrahedron") {
        createTetrahedron();
    }
    else if(type == "Octahedron") {
        createOctahedron();
    }
    else if(type == "Icosahedron") {
        createIcosahedron();
    }
    else if(type == "Dodecahedron") {
        createDodecahedron();
    }
}