// Copyright (C) 2018-2021 Artyom Tokarev. All rights reserved.
// Licensed under the MIT License.

#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace spt {

using default_value_type = double;

template <std::size_t Dim, typename ValueType = default_value_type>
struct vec;

template <std::size_t Dim> using veci = vec<Dim, std::int64_t>;
template <std::size_t Dim> using vecu = vec<Dim, std::uint64_t>;
template <std::size_t Dim> using vecf = vec<Dim, float>;
template <std::size_t Dim> using vecd = vec<Dim, double>;
template <typename ValueType> 
using vec2 = vec<2, ValueType>;
using vec2i = vec2<std::int64_t>;
using vec2u = vec2<std::uint64_t>;
using vec2f = vec2<float>;
using vec2d = vec2<double>;
template <typename ValueType> 
using vec3 = vec<3, ValueType>;
using vec3i = vec3<std::int64_t>;
using vec3u = vec3<std::uint64_t>;
using vec3f = vec3<float>;
using vec3d = vec3<double>;
template <typename ValueType> 
using vec4 = vec<4, ValueType>;
using vec4i = vec4<std::int64_t>;
using vec4u = vec4<std::uint64_t>;
using vec4f = vec4<float>;
using vec4d = vec4<double>;


template <std::size_t Dim, typename ValueType>
struct vec {
    static constexpr std::size_t dim = Dim;
    using value_type = ValueType;

    std::array<ValueType, Dim> x{};

    static vec zeros() {
        return {};
    }
    static vec ones() {
        return filled_with(static_cast<ValueType>(1));
    }
    static vec filled_with(ValueType val) {
        vec res;
        std::fill(res.x.begin(), res.x.end(), val);
        return res;
    }

    auto magnitude() const {
        return std::sqrt(magnitude2());
    }
    ValueType magnitude2() const {
        return dot(*this, *this);
    }

    vec& normalize() {
        ValueType inv_magn = static_cast<ValueType>(1) / magnitude();
        for (std::size_t i = 0; i < Dim; ++i)
            x[i] *= inv_magn;
        return *this;
    }

    vec& operator=(const vec& right) {
        x = right.x;
        return *this;
    }
    vec operator-() const {
        vec res = *this;
        if constexpr (Dim == 2) {
            res.x[0] = -res.x[0];
            res.x[1] = -res.x[1];
        } else if constexpr (Dim == 3) {
            res.x[0] = -res.x[0];
            res.x[1] = -res.x[1];
            res.x[2] = -res.x[2];
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                res.x[i] = -res.x[i];
        }
        return res;
    }
    vec operator+(const vec& right) const {
        vec res = *this;
        if constexpr (Dim == 2) {
            res.x[0] += right.x[0];
            res.x[1] += right.x[1];
        } else if constexpr (Dim == 3) {
            res.x[0] += right.x[0];
            res.x[1] += right.x[1];
            res.x[2] += right.x[2];
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                res.x[i] += right.x[i];
        }
        return res;
    }
    vec operator-(const vec& right) const {
        vec res = *this;
        if constexpr (Dim == 2) {
            res.x[0] -= right.x[0];
            res.x[1] -= right.x[1];
        } else if constexpr (Dim == 3) {
            res.x[0] -= right.x[0];
            res.x[1] -= right.x[1];
            res.x[2] -= right.x[2];
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                res.x[i] -= right.x[i];
        }
        return res;
    }
    vec operator*(ValueType scalar) const {
        vec res = *this;
        if constexpr (Dim == 2) {
            res.x[0] *= scalar;
            res.x[1] *= scalar;
        } else if constexpr (Dim == 3) {
            res.x[0] *= scalar;
            res.x[1] *= scalar;
            res.x[2] *= scalar;
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                res.x[i] *= scalar;
        }
        return res;
    }
    vec operator/(ValueType scalar) const {
        vec res = *this;
        if constexpr (Dim == 2) {
            res.x[0] /= scalar;
            res.x[1] /= scalar;
        } else if constexpr (Dim == 3) {
            res.x[0] /= scalar;
            res.x[1] /= scalar;
            res.x[2] /= scalar;
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                res.x[i] /= scalar;
        }
        return res;
    }
    vec& operator+=(const vec& right) {
        if constexpr (Dim == 2) {
            x[0] += right.x[0];
            x[1] += right.x[1];
        } else if constexpr (Dim == 3) {
            x[0] += right.x[0];
            x[1] += right.x[1];
            x[2] += right.x[2];
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                x[i] += right.x[i];
        }
        return *this;
    }
    vec& operator-=(const vec& right) {
        if constexpr (Dim == 2) {
            x[0] -= right.x[0];
            x[1] -= right.x[1];
        } else if constexpr (Dim == 3) {
            x[0] -= right.x[0];
            x[1] -= right.x[1];
            x[2] -= right.x[2];
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                x[i] -= right.x[i];
        }
        return *this;
    }
    vec& operator*=(ValueType scalar) {
        if constexpr (Dim == 2) {
            x[0] *= scalar;
            x[1] *= scalar;
        } else if constexpr (Dim == 3) {
            x[0] *= scalar;
            x[1] *= scalar;
            x[2] *= scalar;
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                x[i] *= scalar;
        }
        return *this;
    }
    vec& operator/=(ValueType scalar) {
        if constexpr (Dim == 2) {
            x[0] /= scalar;
            x[1] /= scalar;
        } else if constexpr (Dim == 3) {
            x[0] /= scalar;
            x[1] /= scalar;
            x[2] /= scalar;
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                x[i] /= scalar;
        }
        return *this;
    }
    bool operator==(const vec& right) const {
        for (std::size_t i = 0; i < Dim; ++i) {
            if (std::abs(right.x[i] - x[i]) > std::numeric_limits<ValueType>::epsilon()
                    * std::abs(right.x[i] + x[i])) {
                return false;
            }
        }
        return true;
    }
    bool operator!=(const vec& right) const {
        return !(*this == right);
    }
    bool operator<(const vec& right) const {
        return x < right.x;
    }
    ValueType& operator[](std::size_t i) {
        return x[i];
    }
    const ValueType& operator[](std::size_t i) const {
        return x[i];
    }

    vec() {
        std::fill(x.begin(), x.end(), static_cast<ValueType>(0));
    }
    template <std::size_t FromDim, typename FromValueType>
    vec(const vec<FromDim, FromValueType>& other) {
        if constexpr (Dim == 2) {
            x[0] = static_cast<ValueType>(other.x[0]);
            x[1] = static_cast<ValueType>(other.x[1]);
        } else if constexpr (Dim == 3) {
            x[0] = static_cast<ValueType>(other.x[0]);
            x[1] = static_cast<ValueType>(other.x[1]);
            x[2] = static_cast<ValueType>(other.x[2]);
        } else {
            for (std::size_t i = 0; i < Dim; ++i)
                x[i] = static_cast<ValueType>(other.x[i]);
        }
    }
    vec(const vec& other) {
        x = other.x;
    }
    vec(const std::array<ValueType, Dim>& x) : x{x} {}
};


template<std::size_t Dim, typename ValueType>
vec(const std::array<ValueType, Dim>& x) -> vec<Dim, ValueType>;

} // namespace spt


template <std::size_t Dim, typename ValueType>
spt::vec<Dim, ValueType> operator*(ValueType scalar, const spt::vec<Dim, ValueType>& v) {
    return v * scalar;
}

// taken from boost
template<std::size_t Dim, typename ValueType>
struct std::hash<spt::vec<Dim, ValueType>> {
    std::size_t operator()(const spt::vec<Dim, ValueType>& key) const {
        std::hash<ValueType> hasher;
        std::size_t h = 0;
        for (auto e : key.x)
            h ^= hasher(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};
