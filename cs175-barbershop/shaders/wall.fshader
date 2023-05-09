#version 150

uniform sampler2D uTexColor;

// lights in eye space
uniform vec3 uLight;
uniform vec3 uLight2;

in vec2 vTexCoord;

out vec4 fragColor;

void main() {
  vec3 color = texture(uTexColor, vTexCoord).xyz;

  fragColor = vec4(color, 1);
}
