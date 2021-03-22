#include "camera.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "../util/helper.h"

namespace gfx {
float CameraBase::getAspectRatio() {
    if (aspectRatio < 0.0f) {
        std::cerr << "Warning: You should set aspect ratio first!" << std::endl;
        std::cerr << "Aspect ratio will be set automatically..." << std::endl;
        int width, height;
        glfwGetFramebufferSize(glfwGetCurrentContext(), &width, &height);
        this->aspectRatio = static_cast<float>(width) / height;
        setAspectRatio(width, height);
    }
    return aspectRatio;
}

Eigen::Vector3f CameraBase::getPosition() const { return position; }

Eigen::Vector3f CameraBase::getCenter() const { return center; }

void CameraBase::setAspectRatio(int width, int height) { aspectRatio = static_cast<float>(width) / height; }

void CameraBase::setPosition(Eigen::Vector3f _position) { position = _position; }

void CameraBase::setCenter(Eigen::Vector3f _center) { center = _center; }

Eigen::Matrix4f CameraBase::viewWithProjection() { return projection() * view(); }

float* Camera::getCameraRotationAnglePointer() { return &cameraRotationAngle; }

float* Camera::getCameraYOffsetPointer() { return &cameraYOffset; }

float* Camera::getCameraRotationRadiusPointer() { return &cameraRotationRadius; }

Eigen::Matrix4f Camera::projection() {
    return util::perspective(fieldOfView, getAspectRatio(), nearClipPlane, farClipPlane);
}

Eigen::Matrix4f Camera::view() {
    position[0] = center[0] + cameraRotationRadius * cosf(util::radians(cameraRotationAngle));
    position[1] = (center[1] + cameraYOffset) * cameraRotationRadius;
    position[2] = center[2] + cameraRotationRadius * sinf(util::radians(cameraRotationAngle));
    return util::lookAt(position, center, up);
}

bool DebugCamera::first = true;
float DebugCamera::moveSpeed = 5.0f, DebugCamera::mouseSensitivity = 0.1f;
float DebugCamera::yaw = 135, DebugCamera::pitch = -45;
float DebugCamera::lastx = 0, DebugCamera::lasty = 0;

DebugCamera::DebugCamera() { position = Eigen::Vector3f(8.0f, 21.0f, -9.0f); }

void DebugCamera::moveCamera(GLFWwindow* window) {
    float currentFrameTime = static_cast<float>(glfwGetTime());
    float deltaTime = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;
    float speed = moveSpeed * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        position += center * speed;
    else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        position -= center * speed;
    else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        position -= right * speed;
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        position += right * speed;
    else if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        position += up * speed;
    else if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
             glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS)
        position -= up * speed;
}

void DebugCamera::cameraLook(GLFWwindow* window, double _x, double _y) {
    float x = static_cast<float>(_x);
    float y = static_cast<float>(_y);
    if (first) {
        lastx = x;
        lasty = y;
        first = false;
    } else {
        float dx = x - lastx;
        float dy = lasty - y;
        lastx = x;
        lasty = y;
        dx *= mouseSensitivity;
        dy *= mouseSensitivity;
        yaw += dx;
        pitch += dy;
        pitch = std::clamp(pitch, -89.9f, 89.9f);
    }
}

Eigen::Matrix4f DebugCamera::projection() {
    return util::perspective(fieldOfView, getAspectRatio(), nearClipPlane, farClipPlane);
}

void DebugCamera::setFirst(bool isfirst) { first = isfirst; }

Eigen::Matrix4f DebugCamera::view() {
    using util::radians;
    center[0] = cosf(radians(yaw)) * cosf(radians(pitch));
    center[1] = sinf(radians(pitch));
    center[2] = sinf(radians(yaw)) * cosf(radians(pitch));
    center.normalize();

    right = center.cross(Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    right.normalize();
    up = right.cross(center);
    up.normalize();
    return util::lookAt(position, position + center, up);
}
}  // namespace gfx
