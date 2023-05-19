//
// Created by gruzi on 24/02/2023.
//
#include "Canvas2D.h"
#include "Canvas2DUtils.h"
#include <fstream>

void Canvas2D::clear() {
    points.clear();
    lines.clear();
    colors.clear();
}
void Canvas2D::multiply_with_points(const double &factor) {
    for(auto& currentPoint : points) {
        currentPoint->x *= factor, currentPoint->y *= factor;
    }
}
void Canvas2D::add_to_points(const double &x, const double &y) {
    for(auto& currentPoint : points) {
        currentPoint->x += x, currentPoint->y += y;
    }
}
img::EasyImage Canvas2D::generate_blank_image_centered(const int &size) {
    const std::pair<Point2D, Point2D> min_max = Canvas2DUtils::find_minimal_and_maximal_coordinates(points);
    const double x_range = min_max.second.x-min_max.first.x;
    const double y_range = min_max.second.y-min_max.first.y;
    if(std::max(x_range, y_range) == 0) {
        std::cerr << "cannot create a centered image without lines!\n";
        img::EasyImage image(1,1);
        return image;
    }
    const double image_x = ((x_range*size)/(std::max(x_range, y_range)));
    const double image_y = ((y_range*size)/(std::max(x_range, y_range)));

    const double d = ((image_x*0.95f)/(x_range));

    multiply_with_points(d);

    const double DC_x = (d*(min_max.first.x+min_max.second.x))/(2.0f);
    const double DC_y = (d*(min_max.first.y+min_max.second.y))/(2.0f);
    const double dx = (image_x/2.0f)-DC_x;
    const double dy = (image_y/2.0f)-DC_y;

    add_to_points(dx, dy);

    img::EasyImage image(image_x, image_y);
    return image;
}
void Canvas2D::add_lines(const Lines2D &new_lines) {
    for(const Line2D &currentLine : new_lines) {
        add_line(currentLine);
    }
}
void Canvas2D::add_line(const Line2D &new_line) {
    auto p1Ptr = std::make_shared<Point2D>(Point2D(new_line.p1->x, new_line.p1->y));
    auto p2Ptr = std::make_shared<Point2D>(Point2D(new_line.p2->x, new_line.p2->y));
    auto colorPtr =
            std::make_shared<Color>(Color(new_line.color->red, new_line.color->green, new_line.color->blue));
    auto linePtr =
            std::make_shared<Line2D>(Line2D(p1Ptr, p2Ptr, colorPtr, new_line.z1, new_line.z2));

    points.push_back(p1Ptr);
    points.push_back(p2Ptr);
    lines.push_back(linePtr);
    colors.push_back(colorPtr);
}

void Canvas2D::draw_lines(img::EasyImage &img, ZBuffer &zbuffer) {
    for(auto& currentLine : lines) {
        Color color = Color(currentLine->color->red, currentLine->color->green, currentLine->color->blue);
        const int p1_x = static_cast<int>(lround(currentLine->p1->x));
        const int p1_y = static_cast<int>(lround(currentLine->p1->y));
        const int p2_x = static_cast<int>(lround(currentLine->p2->x));
        const int p2_y = static_cast<int>(lround(currentLine->p2->y));
        img.draw_zbuf_line(p1_x, p1_y, currentLine->z1, p2_x, p2_y, currentLine->z2,
                           img::Color(color.red_scaled,color.green_scaled, color.blue_scaled), zbuffer);
    }
}

void Canvas2D::draw_lines(img::EasyImage &img) {
    for(auto& currentLine : lines) {
        Color color = Color(currentLine->color->red, currentLine->color->green, currentLine->color->blue);
        const int p1_x = static_cast<int>(lround(currentLine->p1->x));
        const int p1_y = static_cast<int>(lround(currentLine->p1->y));
        const int p2_x = static_cast<int>(lround(currentLine->p2->x));
        const int p2_y = static_cast<int>(lround(currentLine->p2->y));
        img.draw_line(p1_x, p1_y,p2_x, p2_y,
                      img::Color(color.red_scaled, color.green_scaled, color.blue_scaled));
    }
}

img::EasyImage Canvas2D::generate_2dlsystem(const ini::Configuration &configuration) {
    img::EasyImage img;
    LParser::LSystem2D l_system;

    const std::string input_file = configuration["2DLSystem"]["inputfile"].as_string_or_die();
    const int size = configuration["General"]["size"].as_int_or_die();

    const Color backgroundColor =
            Color(configuration["General"]["backgroundcolor"].as_double_tuple_or_die());
    const Color color = Color( configuration["2DLSystem"]["color"].as_double_tuple_or_die());

    std::ifstream input_stream(input_file);
    input_stream >> l_system;
    input_stream.close();

    generate_lines_from_lsystem(l_system, color);
    img = generate_blank_image_centered(size);
    Canvas2DUtils::draw_background(img, backgroundColor);
    draw_lines(img);
    return img;
}
void Canvas2D::generate_lines_from_lsystem(const LParser::LSystem2D &l_system, const Color &color) {
    std::stack<std::pair<Point2D, double>> points_angles;
    std::string current_string = l_system.get_initiator();
    Point2D current_point = Point2D(0,0);

    double current_angle = Canvas2DUtils::degrees_to_radians(l_system.get_starting_angle());
    double angle_factor = Canvas2DUtils::degrees_to_radians(l_system.get_angle());

   for(int i=0;i<l_system.get_nr_iterations();i++) {
        std::string new_string;
        for(const char &curr_char : current_string) {
            if(curr_char == '+' || curr_char == '-' || curr_char == '(' || curr_char == ')') new_string += curr_char;
            else new_string += l_system.get_stochastic_replacement(curr_char);
        }
        current_string = new_string;
    }
    for(const char &curr_char : current_string) {
        if(curr_char == '+') std::fmod(current_angle += angle_factor, 2*PI);
        else if(curr_char == '-') std::fmod(current_angle -= angle_factor, 2*PI);
        else if(curr_char == '(') points_angles.emplace(current_point, current_angle);
        else if(curr_char == ')') {
            std::pair<Point2D, double> point_angle = points_angles.top();
            current_point.x = point_angle.first.x, current_point.y = point_angle.first.y;
            current_angle = point_angle.second;
            points_angles.pop();
        }
        else {
            Point2D next_point =
                    Point2D(current_point.x+cos(current_angle), current_point.y+sin(current_angle));
            if(l_system.draw(curr_char)) {
                Line2D current_line = Line2D(current_point, next_point, color);
                add_line(current_line);
            }
            current_point = next_point;
        }
    }
}
