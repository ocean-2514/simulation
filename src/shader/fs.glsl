#version 330 core
in vec3 vNormal;
in vec2 vTexCoord;

out vec4 FragColor;

uniform vec3 uColor;

void main()
{
    vec3 lightDir = normalize(vec3(0.5, 0.7, 1.0));
    float diff = max(dot(normalize(vNormal), lightDir), 0.0);
    vec3 color = uColor * (0.35 + diff * 0.95);
    FragColor = vec4(color, 1.0);
}
