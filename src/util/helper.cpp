#include "helper.h"

#include <cmath>

namespace util {
// https://github.com/OpenGP/htrack/blob/master/util/eigen_opengl_helpers.h
Eigen::Matrix4f perspective(float fovy, float aspect, float zNear, float zFar) {
    Eigen::Transform<float, 3, Eigen::Projective> tr;
    tr.matrix().setZero();
    float tan_half_fovy = tanf(radians(fovy) / 2.0f);
    tr(0, 0) = 1.0f / (aspect * tan_half_fovy);
    tr(1, 1) = 1.0f / tan_half_fovy;
    tr(2, 2) = -(zFar + zNear) / (zFar - zNear);
    tr(3, 2) = -1.0f;
    tr(2, 3) = -(2.0f * zFar * zNear) / (zFar - zNear);
    return tr.matrix();
}

Eigen::Matrix4f scale(float x, float y, float z) {
    Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
    tr(0, 0) = x;
    tr(1, 1) = y;
    tr(2, 2) = z;
    return tr;
}

Eigen::Matrix4f translate(Eigen::Vector3f offset) {
    Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
    tr.block<3, 1>(0, 3) = offset;
    return tr;
}

Eigen::Matrix4f translate(float x, float y, float z) {
    Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
    tr(0, 3) = x;
    tr(1, 3) = y;
    tr(2, 3) = z;
    return tr;
}

Eigen::Matrix4f rotateRadian(float x, float y, float z) {
    auto rotation = Eigen::AngleAxisf(x, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(y, Eigen::Vector3f::UnitY()) *
                    Eigen::AngleAxisf(z, Eigen::Vector3f::UnitZ());
    rotation.normalize();
    Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
    tr.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    return tr;
}

Eigen::Matrix4f rotateDegree(float x, float y, float z) {
    auto rotation = Eigen::AngleAxisf(radians(x), Eigen::Vector3f::UnitX()) *
                    Eigen::AngleAxisf(radians(y), Eigen::Vector3f::UnitY()) *
                    Eigen::AngleAxisf(radians(z), Eigen::Vector3f::UnitZ());
    rotation.normalize();
    Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
    tr.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    return tr;
}

Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up) {
    Eigen::Vector3f f = center - eye;
    f.normalize();
    Eigen::Vector3f u = up;
    u.normalize();
    Eigen::Vector3f s = f.cross(u);
    s.normalize();
    u = s.cross(f);
    Eigen::Matrix4f mat = Eigen::Matrix4f::Zero();
    mat(0, 0) = s.x();
    mat(0, 1) = s.y();
    mat(0, 2) = s.z();
    mat(0, 3) = -s.dot(eye);
    mat(1, 0) = u.x();
    mat(1, 1) = u.y();
    mat(1, 2) = u.z();
    mat(1, 3) = -u.dot(eye);
    mat(2, 0) = -f.x();
    mat(2, 1) = -f.y();
    mat(2, 2) = -f.z();
    mat(2, 3) = f.dot(eye);
    mat.row(3) << 0, 0, 0, 1;
    return mat;
}

Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 0) = 2.0f / (right - left);
    mat(1, 1) = 2.0f / (top - bottom);
    mat(2, 2) = -2.0f / (zFar - zNear);
    mat(3, 0) = -(right + left) / (right - left);
    mat(3, 1) = -(top + bottom) / (top - bottom);
    mat(3, 2) = -(zFar + zNear) / (zFar - zNear);
    return mat;
}
}  // namespace util
