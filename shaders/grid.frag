#version 330 core
in vec3 FragPos;

uniform vec4 gridColor;
uniform float fadeDistance;

out vec4 FragColor;

void main() {
    float dist = length(FragPos.xy);
    float alpha = gridColor.a * (1.0 - smoothstep(fadeDistance * 0.5, fadeDistance, dist));

    // Highlight main axes
    float lineWidth = 0.02;
    vec3 color = gridColor.rgb;
    if (abs(FragPos.x) < lineWidth) {
        color = vec3(0.2, 0.8, 0.2); // Y-axis green
        alpha = max(alpha, 0.6);
    }
    if (abs(FragPos.y) < lineWidth) {
        color = vec3(0.8, 0.2, 0.2); // X-axis red
        alpha = max(alpha, 0.6);
    }

    FragColor = vec4(color, alpha);
}
