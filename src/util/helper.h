#pragma once

#include "Eigen/Dense"

namespace util {
template <typename T>
constexpr T PI() {
    return static_cast<T>(3.141592653589793238);
}
constexpr float radians(float angle) { return static_cast<float>(angle * PI<float>() / 180.0); }
Eigen::Matrix4f perspective(float fovy, float aspect, float zNear, float zFar);
Eigen::Matrix4f scale(float x, float y, float z);
Eigen::Matrix4f translate(Eigen::Vector3f offset);
Eigen::Matrix4f translate(float x, float y, float z);
Eigen::Matrix4f rotateRadian(float x, float y, float z);
Eigen::Matrix4f rotateDegree(float x, float y, float z);
Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up);
Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar);
}  // namespace util
