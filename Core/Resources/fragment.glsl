#version 330 core
in vec3 fragColor;
in vec3 fragNormal;
in vec3 fragPosition;
out vec4 finalColor;

uniform vec3 lightDir;
uniform vec3 cameraPos;
uniform float brightness;
uniform float lightStrength;

#include "lighting.glsl"

void main()
{
    // Prepare structs
    Light light;
    light.direction = normalize(lightDir);
    light.color = vec3(1.0);
    light.strength = lightStrength;

    Material mat;
    mat.diffuse = fragColor;
    mat.specular = vec3(1.0);
    mat.shininess = 32.0;

    vec3 viewDir = normalize(cameraPos - fragPosition);

    // Compute lighting
    vec3 result = CalculateBlinnPhong(fragNormal, viewDir, light, mat, fragColor);

    // Apply brightness and gamma correction
    result *= brightness;
    result = pow(result, vec3(1.0 / 2.2));

    finalColor = vec4(result, 1.0);
}
