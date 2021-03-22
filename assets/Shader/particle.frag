#version 410 core
layout (location = 0) out vec4 FragColor;

uniform vec3 baseColor;

void main() {
    FragColor = vec4(baseColor, 1.0);
}
