#pragma once

#include "Eigen/Dense"
#include "GLFW/glfw3.h"

namespace gfx {

class CameraBase {
 public:
    float getAspectRatio();
    Eigen::Vector3f getCenter() const;
    Eigen::Vector3f getPosition() const;

    void setAspectRatio(int width, int height);
    void setPosition(Eigen::Vector3f _position);
    void setCenter(Eigen::Vector3f _center);
    virtual Eigen::Matrix4f view() = 0;
    virtual Eigen::Matrix4f projection() = 0;
    virtual Eigen::Matrix4f viewWithProjection();

 protected:
    virtual ~CameraBase() = default;
    float aspectRatio = -1.0f;
    float fieldOfView = 60.0f;
    float nearClipPlane = 0.01f;
    float farClipPlane = 100.0f;
    Eigen::Vector3f position = Eigen::Vector3f(0.0f, 8.0f, 0.0f);
    Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
};
class Camera final : public CameraBase {
 public:
    float* getCameraRotationAnglePointer();
    float* getCameraRotationRadiusPointer();
    float* getCameraYOffsetPointer();
    Eigen::Matrix4f view() override;
    Eigen::Matrix4f projection() override;

 private:
    float cameraRotationAngle = 60.0f;
    float cameraYOffset = 0.2f;
    float cameraRotationRadius = 17.0f;
};

class DebugCamera final : public CameraBase {
 public:
    static float moveSpeed, mouseSensitivity;
    DebugCamera();
    Eigen::Matrix4f view() override;
    Eigen::Matrix4f projection() override;
    void moveCamera(GLFWwindow* window);
    static void setFirst(bool isfirst);
    static void cameraLook(GLFWwindow* window, double x, double y);

 private:
    static bool first;
    static float yaw, pitch;
    static float lastx, lasty;
    float lastFrameTime = 0.0f;
    Eigen::Vector3f right = Eigen::Vector3f::Zero();
};
}  // namespace gfx
