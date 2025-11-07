struct Light {
    vec3 direction;
    vec3 color;
    float strength;
};

struct Material {
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

vec3 CalculateBlinnPhong(
    vec3 normal,
    vec3 viewDir,
    Light light,
    Material material,
    vec3 baseColor
) {
    vec3 N = normalize(normal);
    vec3 L = normalize(-light.direction);
    vec3 V = normalize(viewDir);

    vec3 H = normalize(L + V);

    vec3 ambient = 0.1 * light.color * baseColor;

    float diff = max(dot(N, L), 0.0);
    vec3 diffuse = diff * light.color * material.diffuse * light.strength;

    float spec = pow(max(dot(N, H), 0.0), material.shininess);
    vec3 specular = spec * light.color * material.specular * light.strength;

    return ambient + diffuse + specular;
}