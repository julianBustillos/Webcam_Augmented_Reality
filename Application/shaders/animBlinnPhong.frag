#version 330 core

out vec4 color;

in vec3 Position;
in vec3 Normal;
in vec4 Color;

uniform vec3 lightPos;
uniform vec3 viewPos;


void main()
{
    // Constants
    float ambientStrength = 0.5f;
    float diffuseStrength = 0.9f;
    float specularStrength = 1.0f;
    int shininess = 32;
    float eta = 10.0f;


    // Ambient
    float ambient = ambientStrength;

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - Position);
    float diffuse = diffuseStrength * max(dot(norm, lightDir), 0.0f);

    // Specular
    vec3 viewDir = normalize(viewPos - Position);
    vec3 halfDir = normalize(viewDir + lightDir);
    float specular = specularStrength * pow(max(dot(norm, halfDir), 0.0f), shininess);

    // Fresnel coefficient
    float F0 = pow((1.0f - eta) / (1.0f + eta), 2);
    float F = F0 + (1.0f - F0) * pow(1.0f - dot(halfDir, viewDir) , 5);

    // Compute final color
    vec3 result = (ambient + diffuse + F * specular) * Color.xyz;
    color = vec4(result, Color.w);
}
