/*
Computer animation assignment 1: particle system for softbody (jelly) simulation

First step: Search TODO comments to find the methods that you need to implement.
    - main.cpp
    - src/simulation/cube.cpp
    - src/simulation/integrator.cpp
    - src/simulation/terrain.cpp

*/
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "Eigen/Core"
#include "GLFW/glfw3.h"
#include "glad/glad.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "src/gfx.h"
#include "src/simulation.h"
#include "src/util.h"

// Global variables are evil!
namespace {
// TODO
// Please change this c string to your student ID
constexpr const char* studentID = "987654321";
// Shadow texture size, default is 8192 * 8192
// If shadow render time in debug panel with vsync OFF is too high, you can
// reduce it to 1024 * 1024 or lower but it will result in low shadow quality
int shadowTextureSize = 8192;
// Scene size
int g_ScreenWidth = 1024, g_ScreenHeight = 768;
// Current terrain
gfx::Rigidbody* currentTerrainGraphics = nullptr;
// Current terrain type
auto currentTerrainType = simulation::TerrainType::Plane;
// Plane and tiltedplane share same vertices
// Their difference is model matrix
std::unique_ptr<gfx::Plane> g_Plane = nullptr;
// Sphere and wiresphere share same vertices
// But they have different render behavior based on
// gfx::Sphere::RenderMode::WIREFRAME or gfx::Sphere::RenderMode::FILLED
std::unique_ptr<gfx::Sphere> g_Sphere = nullptr;
// Soft body cube's graphic
std::unique_ptr<gfx::SoftCube> g_cube = nullptr;
// Camera controlled by setting in camera panel
gfx::Camera basicCamera;
// Camera controlled by keyboard and mouse
gfx::DebugCamera debugCamera;
// Current camera
gfx::CameraBase* currentCamera = &basicCamera;
// Switch for camera
bool isUsingDebugCamera = false;
// Switch for recording
bool isRecording = false;
// Switch for render camera control panel
bool isUsingCameraPanel = false;
// Switch for render record control panel
bool isUsingRecordingPanel = false;
// Switch for render simulation control panel
bool isUsingSimulationPanel = false;
// Switch for render debug panel
bool isUsingDebugPanel = false;
// for simulation
simulation::MassSpringSystem particleSystem;
// The value is based on moniter refresh rate to uniform simulation speed
// It is calculated by ceil(480 / refresh rate)
int simulationPerFrame = 0;
// Test system stability
bool isSystemStable = true;
// For debugging
double simulationTime = 0, dataUpdateTime = 0, shadowRenderTime = 0, sceneRenderTime = 0, uiRenderTime = 0,
       eventPollTime = 0, recordTime = 0;
// For checking Vsync is ON or OFF
bool isFPSLimited = true;
// For screenshot
std::unique_ptr<util::Exporter> g_Exporter = nullptr;
// For simulationPerFrame before enter export mode
int simulationPerFrameOld = 0;
}  // namespace

/**
 * @brief When resizing window, we need to update viewport and camera's aspect
 * ratio.
 * @param window current GLFW context
 * @param screenWidth Screen width after resize.
 * @param screenHeight Screen height after resize.
 */
void reshape(GLFWwindow* window, int screenWidth, int screenHeight);

/**
 * @brief Initialize globals and OpenGL
 *
 * @return `GLFWwindow*` The current GLFW context window
 */
GLFWwindow* initialize();

/**
 * @brief Callback function for the debug camera.
 */
void debugCameraKeyboard(GLFWwindow* window, int key, int scancode, int action, int mode);
/**
 * @brief Reset pointer of std::unique_ptr to call destructor before calling
 * glfwTerminate() to avoid segfault since some destructor contains OpenGL
 * functions.
 *
 * @param window The context to be destroyed
 */
void shutdown();
/**
 * @brief Dear-ImGui main control panel
 *
 */
void mainPanel();
/**
 * @brief Dear-ImGui camera control panel
 *
 * @param window: Current context
 */
void cameraPanel(GLFWwindow* window);
/**
 * @brief Dear-ImGui recording control panel
 *
 */
void recordingPanel();
/**
 * @brief Dear-ImGui simulation control panel
 *
 */
void simulationPanel();
/**
 * @brief Dear-ImGui debug panel
 *
 */
void debugPanel();
/**
 * @brief Put panels between ImGui::NewFrame() and ImGui::Render() to render
 *
 * @param window: Current context
 */
void renderUI(GLFWwindow* window);

int main() {
    GLFWwindow* window = initialize();
    // No window created
    if (window == nullptr) return 1;
    // Shader programs
    gfx::Program renderProgram;
    gfx::Program simpleRenderProgram;
    gfx::Program skyboxRenderProgram;
    gfx::Program shadowProgram;
    // Texture for shadow mapping
    gfx::ShadowMapTexture shadow(shadowTextureSize);
    // Handle softbody graphics
    g_cube = std::make_unique<gfx::SoftCube>(particleSystem.getCubePointer(0));
    // The skybox
    gfx::SkyBox skybox;
    // Load data from assets
    {
        using pf = util::PathFinder;
        gfx::Shader shadowVertexShader(pf::find("Shader/shadow.vert"), GL_VERTEX_SHADER);
        gfx::Shader shadowFragmentShader(pf::find("Shader/shadow.frag"), GL_FRAGMENT_SHADER);
        gfx::Shader renderVertexShader(pf::find("Shader/render.vert"), GL_VERTEX_SHADER);
        gfx::Shader renderFragmentShader(pf::find("Shader/render.frag"), GL_FRAGMENT_SHADER);
        gfx::Shader particleVertexShader(pf::find("Shader/particle.vert"), GL_VERTEX_SHADER);
        gfx::Shader particleFragmentShader(pf::find("Shader/particle.frag"), GL_FRAGMENT_SHADER);
        gfx::Shader skyboxVertexShader(pf::find("Shader/skybox.vert"), GL_VERTEX_SHADER);
        gfx::Shader skyboxFragmentShader(pf::find("Shader/skybox.frag"), GL_FRAGMENT_SHADER);

        auto dice1 = std::make_shared<gfx::Texture>(pf::find("Texture/dice0.png"));
        auto dice2 = std::make_shared<gfx::Texture>(pf::find("Texture/dice1.png"));
        auto dice3 = std::make_shared<gfx::Texture>(pf::find("Texture/dice2.png"));
        auto dice4 = std::make_shared<gfx::Texture>(pf::find("Texture/dice3.png"));
        auto dice5 = std::make_shared<gfx::Texture>(pf::find("Texture/dice4.png"));
        auto dice6 = std::make_shared<gfx::Texture>(pf::find("Texture/dice5.png"));

        auto skyboxFileList = std::array<util::fs::path, 6>(
            {pf::find("Texture/skybox0.png"), pf::find("Texture/skybox1.png"), pf::find("Texture/skybox2.png"),
             pf::find("Texture/skybox3.png"), pf::find("Texture/skybox4.png"), pf::find("Texture/skybox5.png")});
        auto sky = std::make_shared<gfx::CubeTexture>(skyboxFileList);
        // Setup shaders, these objects can be destroyed after linkShader()
        renderProgram.linkShader(renderVertexShader, renderFragmentShader);
        shadowProgram.linkShader(shadowVertexShader, shadowFragmentShader);
        simpleRenderProgram.linkShader(particleVertexShader, particleFragmentShader);
        skyboxRenderProgram.linkShader(skyboxVertexShader, skyboxFragmentShader);
        // Ownership is passed to gfx::SoftCube, so these shared pointers
        // can be destroyed safely.
        g_cube->setTextures({dice1, dice2, dice3, dice4, dice5, dice6});
        skybox.setTexture(sky);
    }
    // Setup light, uniforms are persisted.
    {
        Eigen::Vector3f lightPosition(11.1f, 24.9f, -14.8f);
        Eigen::Matrix4f lightSpaceMatrix = util::ortho(-30.0f, 30.0f, -30.0f, 30.0f, -75.0f, 75.0f);
        lightSpaceMatrix *=
            util::lookAt(lightPosition, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        // Shader program should be use atleast once before setting up uniforms
        shadowProgram.use();
        shadowProgram.setUniform("lightSpaceMatrix", lightSpaceMatrix);

        renderProgram.use();
        renderProgram.setUniform("lightSpaceMatrix", lightSpaceMatrix);
        renderProgram.setUniform("shadowMap", shadow.getIndex());
        renderProgram.setUniform("lightPos", lightPosition);
    }
    util::Clock clock;
    clock.reset();
    while (!glfwWindowShouldClose(window)) {
        // Keyboard and mouse inputs.
        glfwPollEvents();
        // Moving camera only if debug camera is on.
        if (isUsingDebugCamera) debugCamera.moveCamera(window);
        // Update numbers smoothly.
        eventPollTime = 0.5 * eventPollTime + 0.5 * clock.timeElapsed();
        // Simulation
        for (int i = 0; i < simulationPerFrame; i++) particleSystem.simulationOneTimeStep();
        // Stability checking
        if (particleSystem.isSimulating) {
            if (particleSystem.checkStable()) {
                isSystemStable = true;
            } else {
                isSystemStable = false;
                particleSystem.isSimulating = false;
            }
        }
        simulationTime = 0.5 * simulationTime + 0.5 * clock.timeElapsed();
        // Update position and normal only when simulating
        if (particleSystem.isSimulating) g_cube->update();
        dataUpdateTime = 0.5 * dataUpdateTime + 0.5 * clock.timeElapsed();
        // 1. Render shadow to texture
        glViewport(0, 0, shadow.getShadowSize(), shadow.getShadowSize());
        glCullFace(GL_FRONT);
        shadowProgram.use();
        shadow.bindFrameBuffer();
        glClear(GL_DEPTH_BUFFER_BIT);
        // Rendor terrain's shadow
        currentTerrainGraphics->render(&shadowProgram);
        // Rendor cube's shadow
        if (particleSystem.isDrawingCube)
            g_cube->renderCube(&shadowProgram);
        else
            g_cube->renderPoints(&shadowProgram);

        if (particleSystem.isDrawingStruct) {
            g_cube->renderLines(&shadowProgram, simulation::Spring::SpringType::STRUCT);
        }
        if (particleSystem.isDrawingShear) {
            g_cube->renderLines(&shadowProgram, simulation::Spring::SpringType::SHEAR);
        }
        if (particleSystem.isDrawingBending) {
            g_cube->renderLines(&shadowProgram, simulation::Spring::SpringType::BENDING);
        }
        shadow.unbindFrameBuffer();
        glCullFace(GL_BACK);
        shadowRenderTime = 0.5 * shadowRenderTime + 0.5 * clock.timeElapsed();
        // 2a. Render scene (springs / particles)
        glViewport(0, 0, g_ScreenWidth, g_ScreenHeight);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        simpleRenderProgram.use();
        simpleRenderProgram.setUniform("VP", currentCamera->viewWithProjection());

        if (!particleSystem.isDrawingCube) {
            g_cube->renderPoints(&simpleRenderProgram);
        }
        if (particleSystem.isDrawingStruct) {
            g_cube->renderLines(&simpleRenderProgram, simulation::Spring::SpringType::STRUCT);
        }
        if (particleSystem.isDrawingShear) {
            g_cube->renderLines(&simpleRenderProgram, simulation::Spring::SpringType::SHEAR);
        }
        if (particleSystem.isDrawingBending) {
            g_cube->renderLines(&simpleRenderProgram, simulation::Spring::SpringType::BENDING);
        }
        // 2b. Render scene (cube / terrain)
        renderProgram.use();
        renderProgram.setUniform("viewPos", currentCamera->getPosition());
        renderProgram.setUniform("VP", currentCamera->viewWithProjection());
        currentTerrainGraphics->render(&renderProgram);
        if (particleSystem.isDrawingCube) {
            const bool cubeWithLine =
                particleSystem.isDrawingStruct || particleSystem.isDrawingShear || particleSystem.isDrawingBending;
            if (cubeWithLine) {
                glEnable(GL_POLYGON_OFFSET_FILL);
                glPolygonOffset(1.0, 1.0f);
            }
            g_cube->renderCube(&renderProgram);
            if (cubeWithLine) {
                glDisable(GL_POLYGON_OFFSET_FILL);
            }
        }
        // 3. Render the skybox when system is stable.
        if (isSystemStable) {
            skyboxRenderProgram.use();
            skyboxRenderProgram.setUniform("projection", currentCamera->projection());
            skyboxRenderProgram.setUniform("view", currentCamera->view());
            skybox.render(&skyboxRenderProgram);
        }
        sceneRenderTime = 0.5 * sceneRenderTime + 0.5 * clock.timeElapsed();
        // 4. Render ImGui UI
        renderUI(window);
        uiRenderTime = 0.5 * uiRenderTime + 0.5 * clock.timeElapsed();
        // 5. Output screenshots if needed
        if (isRecording) {
            g_Exporter->outputScreenShot();
            if (util::Writer::getPictureCounter() >= 999) {
                // Restore old simulation per frame setting.
                simulationPerFrame = simulationPerFrameOld;
                isRecording = false;
            }
        }
        recordTime = 0.5 * recordTime + 0.5 * clock.timeElapsed();
        glfwSwapBuffers(window);
    }
    shutdown();
    glfwDestroyWindow(window);
    return 0;
}

void reshape(GLFWwindow* window, int screenWidth, int screenHeight) {
    isRecording = false;
    g_ScreenWidth = screenWidth;
    g_ScreenHeight = screenHeight;
    glViewport(0, 0, g_ScreenWidth, g_ScreenHeight);
    basicCamera.setAspectRatio(g_ScreenWidth, g_ScreenHeight);
    debugCamera.setAspectRatio(g_ScreenWidth, g_ScreenHeight);
    g_Exporter->resize(g_ScreenWidth, g_ScreenHeight);
}

GLFWwindow* initialize() {
    // Initialize GLFW
    if (glfwInit() == GLFW_FALSE) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        return nullptr;
    }
    std::atexit(glfwTerminate);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Create GLFW context
    const auto screenTitle = std::string("Soft-body Simulation ") + studentID;
    GLFWwindow* window = glfwCreateWindow(g_ScreenWidth, g_ScreenHeight, screenTitle.c_str(), nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create OpenGL 4.1 window!" << std::endl;
        return nullptr;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // Initialize glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize OpenGL context" << std::endl;
        return nullptr;
    }
    // Find assets folder
    if (!util::PathFinder::initialize()) {
        std::cerr << "Cannot find assets!" << std::endl;
        return nullptr;
    }
    // OK, everything works fine
    // ----------------------------------------------------------
    // For high dpi monitors
    glfwGetFramebufferSize(window, &g_ScreenWidth, &g_ScreenHeight);
    GLFWmonitor* moniter = glfwGetPrimaryMonitor();
    const GLFWvidmode* vidMode = glfwGetVideoMode(moniter);
    int maxTextureSize = 1024;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
    shadowTextureSize = std::min(shadowTextureSize, maxTextureSize);
    // Print some system information
    std::cout << std::left << std::setw(26) << "Current OpenGL renderer"
              << ": " << glGetString(GL_RENDERER) << std::endl;
    std::cout << std::left << std::setw(26) << "Current OpenGL context"
              << ": " << glGetString(GL_VERSION) << std::endl;
    std::cout << std::left << std::setw(26) << "Moniter refresh rate"
              << ": " << vidMode->refreshRate << " Hz" << std::endl;
    std::cout << std::left << std::setw(26) << "Max texture size support"
              << ": " << maxTextureSize << " * " << maxTextureSize << std::endl;
    std::cout << std::left << std::setw(26) << "Shadow texture size"
              << ": " << shadowTextureSize << " * " << shadowTextureSize << std::endl;
    // Setup exporter
    g_Exporter = std::make_unique<util::Exporter>();
    // Setup Opengl
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glPointSize(3.0f);
    glClearColor(0.6f, 0.0f, 0.0f, 1.0f);
    // Setup GLFW
    reshape(window, g_ScreenWidth, g_ScreenHeight);
    glfwSetFramebufferSizeCallback(window, reshape);
    // Initialize dear-ImGui
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 410 core");
    // Setup simulation speed
    simulationPerFrame = 480 / vidMode->refreshRate + static_cast<bool>(480 % vidMode->refreshRate);
    // Load texture and create graphic of terrain
    auto wood = std::make_shared<gfx::Texture>(util::PathFinder::find("Texture/wood.png"));
    g_Plane = std::make_unique<gfx::Plane>();
    g_Plane->setTexture(wood);
    g_Sphere = std::make_unique<gfx::Sphere>();
    g_Sphere->setTexture(wood);
    // Physics engine
    auto terrain = simulation::TerrainFactory::CreateTerrain(simulation::TerrainType::Plane);
    auto integrator = simulation::IntegratorFactory::CreateIntegrator(simulation::IntegratorType::ExplicitEuler);
    particleSystem.setIntegrator(std::move(integrator));
    currentTerrainGraphics = g_Plane.get();
    currentTerrainGraphics->setModelMatrix(terrain->getModelMatrix());
    particleSystem.setTerrain(std::move(terrain));
    return window;
}

void debugCameraKeyboard(GLFWwindow* window, int key, int scancode, int action, int mode) {
    if (action == GLFW_PRESS && key == GLFW_KEY_F9) {
        if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED) {
            // Show the mouse cursor
            glfwSetCursorPosCallback(window, nullptr);
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        } else {
            // Reset delta x and delta y to avoid view teleporting
            gfx::DebugCamera::setFirst(true);
            // Hide the mouse cursor
            glfwSetCursorPosCallback(window, gfx::DebugCamera::cameraLook);
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }
    }
}

void shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    // Destructor should be called before glfwTerminate()
    g_Plane.reset();
    g_Sphere.reset();
    g_cube.reset();
    g_Exporter.reset();
}

void mainPanel() {
    // Main Panel
    ImGui::SetNextWindowSize(ImVec2(220.0f, 210.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(60.0f, 60.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);
    if (ImGui::Begin("Main Control Panel")) {
        if (!isSystemStable) {
            ImGui::Text("System Unstable!");
            ImGui::Text("Please reset the parameters!");
        }
        // Open / close other panels.
        // x ^= 1 === x = !x
        if (ImGui::Button("Camera Panel")) {
            isUsingCameraPanel ^= true;
        }
        if (ImGui::Button("Recording Panel")) {
            isUsingRecordingPanel ^= true;
        }
        if (ImGui::Button("Simulation Panel")) {
            isUsingSimulationPanel ^= true;
        }
        if (ImGui::Button("Debug Panel")) {
            isUsingDebugPanel ^= true;
        }
        // Terrain setup
        ImGui::Text("Select terrain type here :");
        using simulation::TerrainType;
        bool terrainChanged = false;
        if (ImGui::Button("Plane") && currentTerrainType != TerrainType::Plane) {
            currentTerrainType = TerrainType::Plane;
            terrainChanged = true;
            currentTerrainGraphics = g_Plane.get();
        }
        ImGui::SameLine();
        if (ImGui::Button("Sphere") && currentTerrainType != TerrainType::Sphere) {
            currentTerrainType = TerrainType::Sphere;
            terrainChanged = true;
            g_Sphere->setRenderMode(gfx::Sphere::RenderMode::FILLED);
            currentTerrainGraphics = g_Sphere.get();
        }
        if (ImGui::Button("Bowl") && currentTerrainType != TerrainType::Bowl) {
            currentTerrainType = TerrainType::Bowl;
            terrainChanged = true;
            g_Sphere->setRenderMode(gfx::Sphere::RenderMode::WIREFRAME);
            currentTerrainGraphics = g_Sphere.get();
        }
        ImGui::SameLine();
        if (ImGui::Button("TiltedPlane") && currentTerrainType != TerrainType::TiltedPlane) {
            currentTerrainType = TerrainType::TiltedPlane;
            terrainChanged = true;
            currentTerrainGraphics = g_Plane.get();
        }
        if (terrainChanged) {
            auto terrain = simulation::TerrainFactory::CreateTerrain(currentTerrainType);
            currentTerrainGraphics->setModelMatrix(terrain->getModelMatrix());
            particleSystem.setTerrain(std::move(terrain));
        }
    }
    ImGui::End();
}

void cameraPanel(GLFWwindow* window) {
    // Camera control
    // Create imgui window
    ImGui::SetNextWindowSize(ImVec2(500.0f, 210.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(300.0f, 60.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);
    if (ImGui::Begin("Camera Control Panel", &isUsingCameraPanel)) {
        if (!isUsingDebugCamera) ImGui::Text("Use this panel to control the camera");
        Eigen::Vector3f cameraPosition = currentCamera->getPosition();
        ImGui::Text("Camera position : (%f, %f, %f)", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
        Eigen::Vector3f cameraCenter = currentCamera->getCenter();
        ImGui::Text("Camera lookat : (%f, %f, %f)", cameraCenter[0], cameraCenter[1], cameraCenter[2]);
        if (!isUsingDebugCamera) {
            ImGui::SliderFloat("Camera rotation angle", basicCamera.getCameraRotationAnglePointer(), 0.0f, 360.0f);
            ImGui::SliderFloat("Camera rotation radius", basicCamera.getCameraRotationRadiusPointer(), 0.125f, 50.0f);
            ImGui::SliderFloat("Camera Y Offset", basicCamera.getCameraYOffsetPointer(), -10.0f, 10.0f);
            if (ImGui::Button("Debug Mode")) {
                isUsingDebugCamera = true;
                currentCamera = &debugCamera;
                glfwSetKeyCallback(window, debugCameraKeyboard);
            }
        } else {
            ImGui::InputFloat("Mouse sensitivity", &gfx::DebugCamera::mouseSensitivity, 0.01f, 0.05f);
            ImGui::InputFloat("Move speed", &gfx::DebugCamera::moveSpeed, 0.01f, 0.05f);
            ImGui::Text("Use W A S D CTRL SPACE to move");
            ImGui::Text("Press F9 to bind / unbind mouse");
            ImGui::Text("Bind mouse to control view");
            if (ImGui::Button("Leave debug Mode")) {
                isUsingDebugCamera = false;
                currentCamera = &basicCamera;
                glfwSetCursorPosCallback(window, nullptr);
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }
    }
    ImGui::End();
}

void recordingPanel() {
    // Recording Control
    ImGui::SetNextWindowSize(ImVec2(220.0f, 80.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(60.0f, 280.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);
    if (ImGui::Begin("Recording Control Panel", &isUsingRecordingPanel)) {
        ImGui::Text("Currently %srecording...", isRecording ? "" : "not ");
        const char* const recordingText = isRecording ? "Stop Recording" : "Start Recording";
        if (ImGui::Button(recordingText)) {
            if (isRecording) {
                simulationPerFrame = simulationPerFrameOld;
            } else {
                simulationPerFrameOld = simulationPerFrame;
                simulationPerFrame = 5;
            }
            // Reset filename counter to 0
            util::Writer::resetPictureCounter();
            isRecording ^= true;
        }
    }
    ImGui::End();
}

void simulationPanel() {
    // Simulation Control Panel
    ImGui::SetNextWindowSize(ImVec2(470.0f, 370.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(300.0f, 280.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);
    if (ImGui::Begin("Simulation Control Panel", &isUsingSimulationPanel)) {
        if (ImGui::Button(particleSystem.isSimulating ? "Stop" : "Start")) {
            particleSystem.reset();
            particleSystem.isSimulating ^= true;
        }
        // Render checkbox can be edited while simulating
        ImGui::Checkbox("Draw Cube", &particleSystem.isDrawingCube);
        ImGui::Checkbox("Draw Struct", &particleSystem.isDrawingStruct);
        ImGui::Checkbox("Draw Shear", &particleSystem.isDrawingShear);
        ImGui::Checkbox("Draw Bending", &particleSystem.isDrawingBending);
        // Only allow simulation parameter editing when system is not
        // simulating.
        if (particleSystem.isSimulating) {
            ImGui::Text("Stop simulation to edit parameters...");
        } else {
            ImGui::Text("Integration Method = %d", static_cast<int>(particleSystem.integratorType));
            using simulation::IntegratorFactory;
            using simulation::IntegratorType;
            if (ImGui::Button("EXPLICIT_EULER")) {
                particleSystem.setIntegrator(IntegratorFactory::CreateIntegrator(IntegratorType::ExplicitEuler));
            }
            ImGui::SameLine();
            if (ImGui::Button("IMPLICIT_EULER")) {
                particleSystem.setIntegrator(IntegratorFactory::CreateIntegrator(IntegratorType::ImplicitEuler));
            }
            if (ImGui::Button("MIDPOINT_EULER")) {
                particleSystem.setIntegrator(IntegratorFactory::CreateIntegrator(IntegratorType::MidpointEuler));
            }
            ImGui::SameLine();
            if (ImGui::Button("RUNGE_KUTTA")) {
                particleSystem.setIntegrator(IntegratorFactory::CreateIntegrator(IntegratorType::RungeKuttaFourth));
            }
            if (ImGui::InputInt("Simulation Per Frame", &simulationPerFrame)) {
                simulationPerFrame = std::max(simulationPerFrame, 1);
            }
            if (ImGui::InputFloat("Delta Time", &particleSystem.deltaTime, 0.0001f, 0.0005f, "%.4f")) {
                particleSystem.deltaTime = std::max(particleSystem.deltaTime, 0.0f);
            }
            if (ImGui::InputFloat("Spring Coef", &particleSystem.springCoefStruct, 10.0f, 1.0f)) {
                particleSystem.springCoefStruct = std::max(particleSystem.springCoefStruct, 0.0f);
                particleSystem.setSpringCoef(particleSystem.springCoefStruct, simulation::Spring::SpringType::STRUCT);
                particleSystem.setSpringCoef(particleSystem.springCoefStruct, simulation::Spring::SpringType::SHEAR);
                particleSystem.setSpringCoef(particleSystem.springCoefStruct, simulation::Spring::SpringType::BENDING);
            }
            if (ImGui::InputFloat("Damper Coef", &particleSystem.damperCoefStruct, 10.0, 1.0)) {
                particleSystem.damperCoefStruct = std::max(particleSystem.damperCoefStruct, 0.0f);
                particleSystem.setDamperCoef(particleSystem.damperCoefStruct, simulation::Spring::SpringType::STRUCT);
                particleSystem.setDamperCoef(particleSystem.damperCoefStruct, simulation::Spring::SpringType::SHEAR);
                particleSystem.setDamperCoef(particleSystem.damperCoefStruct, simulation::Spring::SpringType::BENDING);
            }
            if (ImGui::InputFloat("Cube Y Offset", &particleSystem.position[1], 0.3f, 0.05f)) {
                particleSystem.position[1] = std::max(particleSystem.position[1], 1.0f);
            }
            ImGui::InputFloat("Cube Rotation", &particleSystem.rotation, 0.3f, 0.05f);
        }
        if (ImGui::Button("Reset cube")) {
            particleSystem.reset();
            g_cube->update();
        }
    }
    ImGui::End();
}

void debugPanel() {
    ImGui::SetNextWindowSize(ImVec2(220.0f, 230.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(60.0f, 370.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);
    if (ImGui::Begin("Debug Panel", &isUsingDebugPanel)) {
        ImGui::Text("Poll events    : %.3lf ms", eventPollTime);
        ImGui::Text("Simulation     : %.3lf ms", simulationTime);
        ImGui::Text("Update data    : %.3lf ms", dataUpdateTime);
        ImGui::Text("Render shadows : %.3lf ms", shadowRenderTime);
        ImGui::Text("Render scene   : %.3lf ms", sceneRenderTime);
        ImGui::Text("Render UI      : %.3lf ms", uiRenderTime);
        ImGui::Text("Recording      : %.3lf ms", recordTime);
        ImGui::Text("-------------------------");
        double totalTime = eventPollTime + simulationTime + dataUpdateTime + shadowRenderTime + sceneRenderTime +
                           uiRenderTime + recordTime;
        ImGui::Text("Total          : %.3lf ms", totalTime);
        ImGui::Text("Current FPS    : %.1lf FPS", 1000.0 / totalTime);
        if (ImGui::Button("Vsync")) {
            isFPSLimited ^= true;
            glfwSwapInterval(isFPSLimited);
        }
        ImGui::SameLine();
        ImGui::Text(isFPSLimited ? "ON" : "OFF");
    }
    ImGui::End();
}

void renderUI(GLFWwindow* window) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    mainPanel();
    if (isUsingCameraPanel) cameraPanel(window);
    if (isUsingRecordingPanel) recordingPanel();
    if (isUsingSimulationPanel) simulationPanel();
    if (isUsingDebugPanel) debugPanel();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
