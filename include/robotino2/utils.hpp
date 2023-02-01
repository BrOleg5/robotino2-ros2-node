// Copyright 2022 BrOleg5

#ifndef ROBOTINO2__UTILS_HPP_
#define ROBOTINO2__UTILS_HPP_

#ifndef PI
#   define PI 3.14159265359f
#endif

#include <array>

template<class T>
inline T rads2rpm(T rads) {
    return rads / (2 * PI) * 60;
}

template<class T>
inline T rpm2rads(T rpm) {
    return rpm * 2 * PI / 60;
}

template<class T, std::size_t N>
inline std::array<T, N> rpm2rads(std::array<T, N> rpm) {
    std::array<T, N> rads;
    for (size_t i = 0; i < N; i++) {
        rads[i] = rpm[i] * 2 * PI / 60
    }
    return rads;
}

template<class T, std::size_t N>
inline std::array<T, N> rads2rpm(std::array<T, N> rads) {
    std::array<T, N> rpm;
    for (size_t i = 0; i < N; i++) {
        rpm[i] = rads[i] / (2 * PI) * 60;
    }
    return rpm;
}

#endif  // ROBOTINO2__UTILS_HPP_
