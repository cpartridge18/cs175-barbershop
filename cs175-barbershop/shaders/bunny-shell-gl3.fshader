#version 150

uniform sampler2D uTexShell;

uniform vec3 uLight;

uniform float uAlphaExponent;

in vec3 vNormal;
in vec3 vPosition;
in vec2 vTexCoord;

out vec4 fragColor;

float random (vec2 st) {
    return fract(sin(dot(st.xy,
                         vec2(12.9898,78.233)))*
        43758.5453123);
}

void main() {
  vec3 normal = normalize(vNormal);
  vec3 toLight = normalize(uLight - vPosition);

  vec3 toP = -normalize(vPosition);

  vec3 h = normalize(toP + toLight);

  float u = dot(normal, toLight);
  float v = dot(normal, toP);
  u = 1.0 - u*u;
  v = pow(1.0 - v*v, 16.0);

  float r = 0.009 + .29 * u + 0.29* v;
  float g = 0.009+ .21 * u + 0.21* v;
  float b = 0.009+ .4 * u + 0.4 * v;

  float alpha = pow(texture(uTexShell, vTexCoord).r, uAlphaExponent);

  fragColor = vec4(r, g, b, alpha);
}
