#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;

uniform vec4 objectColor;
uniform vec3 lightDir;
uniform vec3 lightColor;
uniform vec3 viewPos;
uniform float ambientStrength;
uniform float specularStrength;

out vec4 FragColor;

void main() {
    vec3 norm = normalize(Normal);
    vec3 lightDirN = normalize(-lightDir);

    // Ambient
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse
    float diff = max(dot(norm, lightDirN), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular (Blinn-Phong)
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 halfDir = normalize(lightDirN + viewDir);
    float spec = pow(max(dot(norm, halfDir), 0.0), 64.0);
    vec3 specular = specularStrength * spec * lightColor;

    // Rim lighting for better shape perception
    float rim = 1.0 - max(dot(viewDir, norm), 0.0);
    rim = smoothstep(0.6, 1.0, rim);
    vec3 rimLight = rim * 0.15 * lightColor;

    vec3 result = (ambient + diffuse + specular + rimLight) * objectColor.rgb;

    // Gamma correction
    result = pow(result, vec3(1.0/2.2));

    FragColor = vec4(result, objectColor.a);
}
