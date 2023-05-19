//
// Created by gruzi on 09/04/2023.
//

#ifndef ENGINE_ZBUFFER_H
#define ENGINE_ZBUFFER_H

#include <vector>

class ZBuffer: public std::vector<std::vector<double>> {
public:
    ZBuffer(const unsigned int &width, const unsigned int &height);
};


#endif //ENGINE_ZBUFFER_H
