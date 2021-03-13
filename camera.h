#pragma once

#include "vec.h"
#include "mat.h"

struct Camera {
    spt::vec3f pos;
    spt::mat3f orient = spt::mat3f::identity();
};
