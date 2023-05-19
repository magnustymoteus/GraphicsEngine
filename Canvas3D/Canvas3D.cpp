#include "Canvas3D.h"
#include "Canvas3DUtils.h"
#include "Canvas2D/Canvas2DUtils.h"
#include "Canvas3D/ZBuffer.h"

#include "PlatonicFigure.h"
#include "SolidFigure.h"
#include <fstream>
#include <array>

void Canvas3D::add_projected_figures() {
    Lines2D lines = Canvas3DUtils::doProjection(figures);
    add_lines(lines);
}
img::EasyImage Canvas3D::generate_zbuffering(const ini::Configuration &configuration) {
    img::EasyImage img;
    const std::string general = "General";
    const int size = configuration[general]["size"].as_int_or_die();
    const Color backgroundColor = Color(configuration["General"]["backgroundcolor"].as_double_tuple_or_die());
    const ini::DoubleTuple eyeTuple = configuration["General"]["eye"].as_double_tuple_or_die();

    eye = Canvas3DUtils::tupleToPoint(eyeTuple);
    generate_zbuffered_figures(configuration);

    std::pair<Point2D, Point2D> min_max = Canvas2DUtils::find_minimal_and_maximal_coordinates(points);
    const double x_range = min_max.second.x-min_max.first.x;
    const double y_range = min_max.second.y-min_max.first.y;
    const double image_x = ((x_range*size)/(std::max(x_range, y_range)));
    const double image_y = ((y_range*size)/(std::max(x_range, y_range)));
    const double d = ((image_x*0.95)/(x_range));
    const double DC_x = (d*(min_max.first.x+min_max.second.x))/(2);
    const double DC_y = (d*(min_max.first.y+min_max.second.y))/(2);
    const double dx = (image_x/2)-DC_x;
    const double dy = (image_y/2)-DC_y;
    img = Canvas2D::generate_blank_image_centered(size);

    ZBuffer zbuffer(image_x, image_y);
    Canvas2DUtils::draw_background(img, backgroundColor);
    draw_lines(img);
    for(const auto& figure : figures) {
        for(const auto& face : figure.faces) {
            Vector3D A = figure.points[face.point_indexes[0]];
            Vector3D B = figure.points[face.point_indexes[1]];
            Vector3D C = figure.points[face.point_indexes[2]];
            Canvas3DUtils::draw_zbuf_triag(zbuffer, img, A,B,C, d, dx, dy, figure.color);
        }
    }
    return img;
}

img::EasyImage Canvas3D::generate_wireframe(const ini::Configuration &configuration) {
    img::EasyImage img;
    const std::string general = "General";
    const int size = configuration[general]["size"].as_int_or_die();
    const Color backgroundColor = Color(configuration["General"]["backgroundcolor"].as_double_tuple_or_die());
    const ini::DoubleTuple eyeTuple = configuration["General"]["eye"].as_double_tuple_or_die();

    eye = Canvas3DUtils::tupleToPoint(eyeTuple);
    generate_figures(configuration);
    img = generate_blank_image_centered(size);
    Canvas2DUtils::draw_background(img, backgroundColor);
    draw_lines(img);
    return img;
}
img::EasyImage Canvas3D::generate_zbuffered_wireframe(const ini::Configuration &configuration) {
    img::EasyImage img;
    const std::string general = "General";
    const int size = configuration[general]["size"].as_int_or_die();
    const Color backgroundColor = Color(configuration["General"]["backgroundcolor"].as_double_tuple_or_die());
    const ini::DoubleTuple eyeTuple = configuration["General"]["eye"].as_double_tuple_or_die();

    eye = Canvas3DUtils::tupleToPoint(eyeTuple);
    generate_figures(configuration);
    img = generate_blank_image_centered(size);
    Canvas2DUtils::draw_background(img, backgroundColor);
    ZBuffer zbuffer(img.get_width(),img.get_height());
    draw_lines(img, zbuffer);
    return img;
}
void Canvas3D::transformFigure(const ini::Configuration &configuration, const std::string &curr_fig_str, Figure &curr_fig) {
    Matrix eyeMat = Canvas3DUtils::eyePointTrans(eye);

    const double scale = configuration[curr_fig_str]["scale"].as_double_or_die();
    Matrix scaleMat = Canvas3DUtils::scaleFigure(scale);

    const double rotateX = configuration[curr_fig_str]["rotateX"].as_double_or_die();
    Matrix xMat = Canvas3DUtils::rotateX(Canvas2DUtils::degrees_to_radians(rotateX));

    const double rotateY = configuration[curr_fig_str]["rotateY"].as_double_or_die();
    Matrix yMat = Canvas3DUtils::rotateY(Canvas2DUtils::degrees_to_radians(rotateY));

    const double rotateZ = configuration[curr_fig_str]["rotateZ"].as_double_or_die();
    Matrix zMat = Canvas3DUtils::rotateZ(Canvas2DUtils::degrees_to_radians(rotateZ));

    const ini::DoubleTuple centerTuple = configuration[curr_fig_str]["center"].as_double_tuple_or_die();
    Vector3D center;
    center.x = centerTuple[0], center.y = centerTuple[1], center.z = centerTuple[2];

    Matrix centerMat = Canvas3DUtils::translate(center);

    Matrix m = scaleMat * xMat * yMat * zMat * centerMat * eyeMat;

    Canvas3DUtils::applyTransformation(curr_fig, m);
}
Figure Canvas3D::generate_linedrawing(const ini::Configuration &configuration, const std::string &curr_fig_str) {
    std::vector<Vector3D> points;
    std::vector<Face> faces;

    Color color = Color(configuration[curr_fig_str]["color"].as_double_tuple_or_die());

    const int nr_points = configuration[curr_fig_str]["nrPoints"].as_int_or_die();
    for (int curr_point = 0; curr_point < nr_points; curr_point++) {
        std::string curr_point_str = "point" + std::to_string(curr_point);
        const ini::DoubleTuple pointTuple = configuration[curr_fig_str][curr_point_str].as_double_tuple_or_die();
        points.push_back(Canvas3DUtils::tupleToPoint(pointTuple));
    }
    const int nr_lines = configuration[curr_fig_str]["nrLines"].as_int_or_die();
    for (int curr_line = 0; curr_line < nr_lines; curr_line++) {
        Face face;
        std::string curr_line_str = "line" + std::to_string(curr_line);
        const ini::IntTuple lineTuple = configuration[curr_fig_str][curr_line_str].as_int_tuple_or_die();
        face.point_indexes.push_back(lineTuple[0]), face.point_indexes.push_back(lineTuple[1]);
        faces.push_back(face);
    }

    Figure curr_fig = Figure(points, faces, color);

    transformFigure(configuration, curr_fig_str, curr_fig);

    return curr_fig;
}
Figure Canvas3D::generate_platonic(const ini::Configuration &configuration, const std::string &curr_fig_str,
                                   const std::string &solid) {
    PlatonicFigure curr_fig = PlatonicFigure();
    curr_fig.make(solid);
    Color color = Color(configuration[curr_fig_str]["color"].as_double_tuple_or_die());
    curr_fig.color = color;
    transformFigure(configuration, curr_fig_str, curr_fig);
    return curr_fig;
}
Figure Canvas3D::generate_sphere(const ini::Configuration &configuration,const std::string &curr_fig_str) {
    const int n = configuration[curr_fig_str]["n"].as_int_or_die();
    SolidFigure curr_fig = SolidFigure();
    curr_fig.createSphere(n);
    Color color = Color(configuration[curr_fig_str]["color"].as_double_tuple_or_die());
    curr_fig.color = color;
    transformFigure(configuration, curr_fig_str, curr_fig);
    return curr_fig;
}
Figure Canvas3D::generate_cone(const ini::Configuration &configuration, const std::string &curr_fig_str) {
    const int n = configuration[curr_fig_str]["n"].as_int_or_die();
    const double h = configuration[curr_fig_str]["height"].as_double_or_die();
    SolidFigure curr_fig = SolidFigure();
    curr_fig.createCone(n,h);
    Color color = Color(configuration[curr_fig_str]["color"].as_double_tuple_or_die());
    curr_fig.color = color;
    transformFigure(configuration, curr_fig_str, curr_fig);
    return curr_fig;
}
Figure Canvas3D::generate_cylinder(const ini::Configuration &configuration, const std::string &curr_fig_str) {
    const int n = configuration[curr_fig_str]["n"].as_int_or_die();
    const double h = configuration[curr_fig_str]["height"].as_double_or_die();
    SolidFigure curr_fig = SolidFigure();
    curr_fig.createCylinder(n,h);
    Color color = Color(configuration[curr_fig_str]["color"].as_double_tuple_or_die());
    curr_fig.color = color;
    transformFigure(configuration, curr_fig_str, curr_fig);
    return curr_fig;
}
Figure Canvas3D::generate_torus(const ini::Configuration &configuration, const std::string &curr_fig_str) {
    const double r = configuration[curr_fig_str]["r"].as_double_or_die();
    const double R = configuration[curr_fig_str]["R"].as_double_or_die();
    const int m = configuration[curr_fig_str]["m"].as_int_or_die();
    const int n = configuration[curr_fig_str]["n"].as_int_or_die();

    SolidFigure curr_fig = SolidFigure();
    curr_fig.createTorus(r,R,n,m);
    Color color = Color(configuration[curr_fig_str]["color"].as_double_tuple_or_die());
    curr_fig.color = color;
    transformFigure(configuration, curr_fig_str, curr_fig);
    return curr_fig;
}
Figure Canvas3D::generate_zbuffered_figure(const ini::Configuration &configuration, const std::string &curr_fig_str) {
    const std::string type = configuration[curr_fig_str]["type"].as_string_or_die();
    if(type == "Sphere") {
        return generate_sphere(configuration, curr_fig_str);
    }
    else if(type == "Cone") {
        return generate_cone(configuration, curr_fig_str);
    }
    else if(type == "Cylinder") {
        return generate_cylinder(configuration, curr_fig_str);
    }
    else if(type == "Torus") {
        return generate_torus(configuration, curr_fig_str);
    }
    else {
        return generate_platonic(configuration, curr_fig_str, type);
    }
}
Figure Canvas3D::generate_figure(const ini::Configuration &configuration, const std::string &curr_fig_str) {
    const std::string type = configuration[curr_fig_str]["type"].as_string_or_die();
    if(type == "LineDrawing") {
        return generate_linedrawing(configuration, curr_fig_str);
    }
    else if(type == "3DLSystem") {
        return generate_3dlsystem(configuration, curr_fig_str);
    }
    else if(type == "Sphere") {
        return generate_sphere(configuration, curr_fig_str);
    }
    else if(type == "Cone") {
        return generate_cone(configuration, curr_fig_str);
    }
    else if(type == "Cylinder") {
        return generate_cylinder(configuration, curr_fig_str);
    }
    else if(type == "Torus") {
        return generate_torus(configuration, curr_fig_str);
    }
    else {
        return generate_platonic(configuration, curr_fig_str, type);
    }
}
void Canvas3D::generate_figures(const ini::Configuration &configuration) {
    for(int i=0;i<configuration["General"]["nrFigures"].as_int_or_die();i++) {
        std::string curr_fig_str = "Figure"+std::to_string(i);
        Figure curr_fig = generate_figure(configuration, curr_fig_str);
        figures.push_back(curr_fig);
    }

    add_projected_figures();
}

void Canvas3D::generate_zbuffered_figures(const ini::Configuration &configuration) {
    for(int i=0;i<configuration["General"]["nrFigures"].as_int_or_die();i++) {
        std::vector<Face> new_faces;
        std::string curr_fig_str = "Figure"+std::to_string(i);
        Figure curr_fig = generate_zbuffered_figure(configuration, curr_fig_str);
        for(const auto& current_face : curr_fig.faces) {
            std::vector<Face> triangulated_faces = Canvas3DUtils::triangulate(current_face);
            new_faces.insert(new_faces.end(), triangulated_faces.begin(), triangulated_faces.end());
        }
        curr_fig.faces = new_faces;
        figures.push_back(curr_fig);
    }
    add_projected_figures();
}

Figure Canvas3D::generate_3dlsystem(const ini::Configuration &configuration, const std::string &curr_fig_str) {
    LParser::LSystem3D l_system;

    const std::string input_file = configuration[curr_fig_str]["inputfile"].as_string_or_die();

    const Color color = Color( configuration[curr_fig_str]["color"].as_double_tuple_or_die());

    std::ifstream input_stream(input_file);
    input_stream >> l_system;
    input_stream.close();

    Figure curr_fig;
    curr_fig.color = color;
    generate_lines_from_3dlsystem(l_system, curr_fig);
    transformFigure(configuration, curr_fig_str, curr_fig);
    return curr_fig;
}
void Canvas3D::generate_lines_from_3dlsystem(const LParser::LSystem3D &l_system, Figure &curr_fig) {
    std::stack<std::pair<Vector3D, std::array<Vector3D, 3>>> points_angles;
    std::string current_string = l_system.get_initiator();
    const double angle = Canvas2DUtils::degrees_to_radians(l_system.get_angle());

    Vector3D current_point = Vector3D::point(0,0,0);

    Vector3D H = Vector3D::vector(1,0,0);
    Vector3D L = Vector3D::vector(0,1,0);
    Vector3D U = Vector3D::vector(0,0,1);

    Vector3D Ht = H, Lt = L, Ut=U;

    int point_index = 0;
    for(int i=0;i<l_system.get_nr_iterations();i++) {
        std::string new_string;
        for(const char &curr_char : current_string) {
            if(l_system.get_alphabet().find(curr_char) != l_system.get_alphabet().end()) {
                new_string += l_system.get_replacement(curr_char);
            }
            else new_string += curr_char;
        }
        current_string = new_string;
    }
    for(const char &curr_char : current_string) {
        Ht=H, Lt=L, Ut=U;
        switch(curr_char) {
            case '+': {
                H = H*cos(angle)+L*sin(angle);
                L = -Ht*sin(angle)+L*cos(angle);
                break;
            }
            case '-': {
                H = H*cos(-angle)+L*sin(-angle);
                L = -Ht*sin(-angle)+L*cos(-angle);
                break;
            }
            case '^': {
                H = H*cos(angle)+U*sin(angle);
                U = -Ht*sin(angle)+U*cos(angle);
                break;
            }
            case '&': {
                H = H*cos(-angle)+U*sin(-angle);
                U = -Ht*sin(-angle)+U*cos(-angle);
                break;
            }
            case '\\': {
                L = L*cos(angle)-U*sin(angle);
                U = Lt*sin(angle)+U*cos(angle);
                break;
            }
            case '/': {
                L = L*cos(-angle)-U*sin(-angle);
                U = Lt*sin(-angle)+Ut*cos(-angle);
                break;
            }
            case '|': {
                H = -H;
                L = -L;
                break;
            }
            case '(': {
                points_angles.push({current_point, {H, L, U}});
                break;
            }
            case ')': {
                std::pair<Vector3D, std::array<Vector3D, 3>> point_angle = points_angles.top();
                current_point = Vector3D::point(point_angle.first);
                H = point_angle.second[0];
                L = point_angle.second[1];
                U = point_angle.second[2];
                points_angles.pop();
                break;
            }
            default: {
                const double x = current_point.x + H.x;
                const double y = current_point.y + H.y;
                const double z = current_point.z + H.z;
                if (l_system.draw(curr_char)) {
                    Face face;
                    Vector3D next_point = Vector3D::point(x, y, z);
                    curr_fig.points.push_back(current_point);
                    curr_fig.points.push_back(next_point);
                    face.point_indexes.push_back(point_index);
                    face.point_indexes.push_back(point_index+1);
                    point_index+=2;
                    curr_fig.faces.push_back(face);
                }
                current_point = Vector3D::point(x,y,z);
                break;
            }
        }
    }
}
