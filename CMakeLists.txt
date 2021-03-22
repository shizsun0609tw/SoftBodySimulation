cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(main LANGUAGES C CXX)

if (NOT (CMAKE_BUILD_TYPE OR CMAKE_CONFIGURATION_TYPES))
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "RelWithDebInfo")
endif ()

# GLFW provides cmake to compile itself
add_subdirectory(vendor/glfw)
# Softbody simulation part
add_executable(main
	${CMAKE_CURRENT_SOURCE_DIR}/src/gfx/camera.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/gfx/graphic.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/gfx/shader.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/gfx/texture.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/cube.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/integrator.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/massSpringSystem.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/particle.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/spring.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/terrain.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/util/clock.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/util/exporter.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/util/filesystem.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/util/helper.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/glad.c
	${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/imgui.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/imgui_demo.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/imgui_draw.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/imgui_impl_glfw.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/imgui_impl_opengl3.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/imgui_tables.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/imgui_widgets.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)
set_target_properties(main PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
set_target_properties(main PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)

target_compile_definitions(main PRIVATE GLFW_INCLUDE_NONE)
target_compile_definitions(main PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)
target_compile_definitions(main PRIVATE EIGEN_MPL2_ONLY)
target_compile_definitions(main PRIVATE EIGEN_NO_DEBUG)

if (MSVC)
	target_compile_options(main PRIVATE "/MP")
	target_compile_options(main PRIVATE "$<$<CONFIG:Release>:/GL>")
	target_compile_options(main PRIVATE "$<$<CONFIG:Release>:/Oi>")
	target_compile_options(glfw PRIVATE "/MP")
	target_compile_options(glfw PRIVATE "$<$<CONFIG:Release>:/GL>")
	target_compile_options(glfw PRIVATE "$<$<CONFIG:Release>:/Oi>")
	target_link_options(main PRIVATE "$<$<CONFIG:Release>:/LTCG:incremental>")
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-march=native" COMPILER_SUPPORT_MARCH_NATIVE)
check_cxx_compiler_flag("-xHost" COMPILER_SUPPORT_xHOST)
check_cxx_compiler_flag("/QxHost" COMPILER_SUPPORT_QxHOST)

if (COMPILER_SUPPORT_MARCH_NATIVE)
	message(STATUS "Compiler support -march=native, build with this flag.")
	target_compile_options(main PRIVATE "-march=native")
elseif(COMPILER_SUPPORT_xHOST)
	message(STATUS "Compiler support -xHost, build with this flag.")
	target_compile_options(main PRIVATE "-xHost")
elseif(COMPILER_SUPPORT_QxHOST)
	message(STATUS "Compiler support /QxHost, build with this flag.")
	target_compile_options(main PRIVATE "/QxHost")
elseif(MSVC)
	try_run(AVX2_RUN_RESULT AVX2_COMPILE_RESULT ${CMAKE_CURRENT_SOURCE_DIR}/bin ${CMAKE_CURRENT_SOURCE_DIR}/vendor/test/cpu_avx2.cpp)
	if(AVX2_RUN_RESULT EQUAL 0)
		message(STATUS "Your CPU supports AVX2")
		target_compile_options(main PRIVATE "/arch:AVX2")
		target_compile_options(glfw PRIVATE "/arch:AVX2")
	else()
		try_run(AVX_RUN_RESULT AVX_COMPILE_RESULT ${CMAKE_CURRENT_BINARY_DIR}/bin ${CMAKE_CURRENT_SOURCE_DIR}/vendor/test/cpu_avx.cpp)
		if(AVX_RUN_RESULT EQUAL 0)
			message(STATUS "Your CPU supports AVX")
			target_compile_options(main PRIVATE "/arch:AVX")
			target_compile_options(glfw PRIVATE "/arch:AVX")
		endif()
	endif()
endif()

target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vendor/include)

find_package(Threads REQUIRED)
target_link_libraries(main PRIVATE Threads::Threads)
target_link_libraries(main PRIVATE glfw)

install(TARGETS main RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)

