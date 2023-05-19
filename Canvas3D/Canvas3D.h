//
// Created by magnustymoteus on 21/03/23.
//

#ifndef ENGINE_CANVAS3D_H
#define ENGINE_CANVAS3D_H

#include "tools/vector3d.h"
#include "Figure.h"
#include "Canvas2D/Canvas2D.h"

class Canvas3D : public Canvas2D {
protected:
    Vector3D eye;
    Figures3D figures;
public:
    void add_projected_figures();
    void generate_figures(const ini::Configuration &configuration);
    void generate_zbuffered_figures(const ini::Configuration &configuration);
    Figure generate_zbuffered_figure(const ini::Configuration &configuration, const std::string &curr_fig_str);
    Figure generate_figure(const ini::Configuration &configuration,const std::string &curr_fig);
    Figure generate_linedrawing(const ini::Configuration &configuration,const std::string &curr_fig);
    Figure generate_platonic(const ini::Configuration &configuration,const std::string &curr_fig, const std::string &solid);
    Figure generate_sphere(const ini::Configuration &configuration,const std::string &curr_fig);
    Figure generate_cone(const ini::Configuration &configuration, const std::string &curr_fig);
    Figure generate_cylinder(const ini::Configuration &configuration, const std::string &curr_fig);
    Figure generate_torus(const ini::Configuration &configuration, const std::string &curr_fig);
    void transformFigure(const ini::Configuration &configuration, const std::string &curr_fig, Figure &fig);
    img::EasyImage generate_wireframe(const ini::Configuration &configuration);
    img::EasyImage generate_zbuffering(const ini::Configuration &configuration);
    img::EasyImage generate_zbuffered_wireframe(const ini::Configuration &configuration);
    Figure generate_3dlsystem(const ini::Configuration &configuration, const std::string &curr_fig);
    void generate_lines_from_3dlsystem(const LParser::LSystem3D &l_system, Figure &curr_fig);

    void generate_fractal(Figure &fig, Figures3D &fractal, const int &nr_iterations, const double &scale);
};


#endif //ENGINE_CANVAS3D_H
