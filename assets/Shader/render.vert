#version 410 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal_in;
layout(location = 2) in vec2 TexCoord_in;

out VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec4 FragPosLightSpace;
} vs_out;

uniform mat4 model;
uniform mat4 VP;
uniform mat4 lightSpaceMatrix;
uniform mat3 invtransmodel;

void main() {
    vs_out.FragPos = vec3(model * vec4(position, 1.0));
    vs_out.Normal = invtransmodel * normal_in;
    vs_out.TexCoords = TexCoord_in;
    vs_out.FragPosLightSpace = lightSpaceMatrix * vec4(vs_out.FragPos, 1.0);
    gl_Position = VP * model * vec4(position, 1.0);
}
