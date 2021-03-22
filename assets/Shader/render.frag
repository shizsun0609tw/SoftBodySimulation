#version 410 core
layout(location = 0) out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec4 FragPosLightSpace;
} fs_in;

uniform int useTexture;
uniform vec3 baseColor;
uniform vec3 lightPos;
uniform vec3 viewPos;
uniform sampler2D diffuseTexture;
uniform sampler2DShadow shadowMap;

float calculateShadow(vec3 projectionCoordinate, float normaldotlight) {
    // Domain transformation to [0, 1]
    projectionCoordinate = projectionCoordinate * 0.5 + 0.5;
    // PCF
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            vec3 textureCoord = vec3(projectionCoordinate.xy + vec2(x, y) * texelSize, projectionCoordinate.z);
            shadow += texture(shadowMap, textureCoord);
        }
    }
    return 0.25 + shadow / 12.0;
}
void main() {
    vec3 color = useTexture == 1 ? texture(diffuseTexture, fs_in.TexCoords).rgb : baseColor;
    vec3 normal = normalize(fs_in.Normal);
    vec3 lightColor = vec3(0.65);
    // Ambient
    vec3 ambient = 0.25 * color;
    // Diffuse
    vec3 lightDir = normalize(lightPos);
    float normaldotlight = dot(normal, lightDir);
    float diff = max(normaldotlight, 0.0);
    vec3 diffuse = diff * lightColor;
    // Specular
    vec3 viewDir = normalize(viewPos - fs_in.FragPos);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), 16.0);
    vec3 specular = spec * lightColor;
    // If using perspective projection, we need to do perspective division
    // fs_in.FragPosLightSpace.xyz / fs_in.FragPosLightSpace.w
    // No shadow outside farClitPlane.
    float shadow = fs_in.FragPosLightSpace.z > 1.0 ? 1.0 : calculateShadow(fs_in.FragPosLightSpace.xyz, normaldotlight);
    vec3 lighting = (ambient + shadow * (diffuse + specular)) * color;
    FragColor = vec4(lighting, 1.0);
}
