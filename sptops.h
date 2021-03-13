// Copyright (C) 2018-2021 Artyom Tokarev. All rights reserved.
// Licensed under the MIT License.

#pragma once
#include <cmath>
#include <algorithm>
#include "mat.h"

namespace spt {

template <std::size_t Dim, typename ValueType>
mat<Dim, ValueType> dot(const mat<Dim, ValueType>& m0, const mat<Dim, ValueType>& m1) {
    mat<Dim, ValueType> res;
    for (std::size_t i = 0; i < Dim; ++i) {
        for (std::size_t j = 0; j < Dim; ++j) {
            ValueType buf = 0.0;
            if constexpr (Dim == 2) {
                buf += m0[i][0] * m1[0][j] + m0[i][1] * m1[1][j];
            } else if constexpr (Dim == 3) {
                buf += m0[i][0] * m1[0][j] + m0[i][1] * m1[1][j] + m0[i][2] * m1[2][j];
            } else {
                for (std::size_t k = 0; k < Dim; ++k)
                    buf += m0[i][k] * m1[k][j];
            }
            res[i][j] = buf;
        }
    }
    return res;
}

template <std::size_t Dim, typename ValueType>
vec<Dim, ValueType> dot(const mat<Dim, ValueType>& m, const vec<Dim, ValueType>& v) {
    vec<Dim, ValueType> res;
    if constexpr (Dim == 2) {
        res[0] = dot(m[0], v);
        res[1] = dot(m[1], v);
    } else if constexpr (Dim == 3) {
        res[0] = dot(m[0], v);
        res[1] = dot(m[1], v);
        res[2] = dot(m[2], v);
    } else {
        for (std::size_t i = 0; i < Dim; ++i)
            res[i] = dot(m[i], v);
    }
    return res;
}

template <std::size_t Dim, typename ValueType>
ValueType dot(const vec<Dim, ValueType>& v0, const vec<Dim, ValueType>& v1) {
    ValueType res = 0;
    if constexpr (Dim == 2) {
        res += v0[0] * v1[0] + v0[1] * v1[1];
    } else if constexpr (Dim == 3) {
        res += v0[0] * v1[0] + v0[1] * v1[1] + v0[2] * v1[2];
    } else {
        for (std::size_t i = 0; i < Dim; ++i)
            res += v0[i] * v1[i];
    }
    return res;
}

template <typename ValueType>
ValueType det(const vec<2, ValueType>& v0, const vec<2, ValueType>& v1) {
    return v0[0] * v1[1] - v0[1] * v1[0];
}

template <typename ValueType>
vec<3, ValueType> cross(const vec<3, ValueType>& vect0, const vec<3, ValueType>& vect1) {
    return std::array{
        vect0.x[1] * vect1.x[2] - vect0.x[2] * vect1.x[1],
        vect0.x[2] * vect1.x[0] - vect0.x[0] * vect1.x[2],
        vect0.x[0] * vect1.x[1] - vect0.x[1] * vect1.x[0] };
}

template <typename ValueType>
ValueType cross(const vec<2, ValueType>& vect0, const vec<2, ValueType>& vect1) {
    return det(vect0, vect1);
}

template <typename ValueType>
ValueType mixed(const vec<3, ValueType>& vect0,
                const vec<3, ValueType>& vect1,
                const vec<3, ValueType>& vect2) {
    return dot(cross(vect0, vect1), vect2);
}

template <std::size_t Dim, typename Real>
Real cos(const vec<Dim, Real>& vect0, const vec<Dim, Real>& vect1) {
    return dot(vect0, vect1) / std::sqrt(vect0.magnitude2() * vect1.magnitude2());
}

template <std::size_t Dim, typename ValueType>
void sort_elementwise(vec<Dim, ValueType>& vect0, vec<Dim, ValueType>& vect1) {
    for (std::size_t i = 0; i < Dim; ++i)
        if (vect0[i] > vect1[i])
            std::swap(vect0[i], vect1[i]);
}

} // namespace spt
