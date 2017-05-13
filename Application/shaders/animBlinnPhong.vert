#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec4 color;

out vec3 Normal;
out vec3 Position;
out vec4 Color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform vec3 scale;
uniform vec3 rotation;
uniform vec3 translation;


mat3 rotationX(float angle)
{
    return mat3(1.0f, 0.0f,       0.0f,
                0.0f, cos(angle), -sin(angle),
                0.0f, sin(angle),  cos(angle));
}

mat3 rotationY(float angle)
{
    return mat3(cos(angle),  0.0f, sin(angle),
                0.0f,        1.0f, 0.0f,
                -sin(angle), 0.0f, cos(angle));
}

mat3 rotationZ(float angle)
{
    return mat3(cos(angle), -sin(angle), 0.0f,
                sin(angle), cos(angle),  0.0f,
                0.0f,       0.0f,        1.0f);
}

vec3 applyScaling(vec3 scale, vec3 position) {
    return vec3(scale.x * position.x, scale.y * position.y, scale.z * position.z);
}


void main()
{
    vec3 realPosition = rotationZ(rotation.z) * rotationY(rotation.y) * rotationX(rotation.x) * applyScaling(scale, position) + translation;
    vec3 realNormal = rotationZ(rotation.z) * rotationY(rotation.y) * rotationX(rotation.x) * normal;
    gl_Position = projection * view *  model * vec4(realPosition, 1.0f);
    Position = vec3(model * vec4(realPosition, 1.0f));
    Normal = mat3(transpose(inverse(model))) * realNormal;
    Color = color;
}
