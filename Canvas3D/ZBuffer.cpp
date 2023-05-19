//
// Created by gruzi on 09/04/2023.
//

#include "ZBuffer.h"
#include <limits>

ZBuffer::ZBuffer(const unsigned int &width, const unsigned int &height) : std::vector<std::vector<double>>
(height, std::vector<double>(width, std::numeric_limits<double>::infinity())) {}