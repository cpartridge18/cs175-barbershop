uniform sampler2D uTexColor;

// lights in eye space
uniform vec3 uLight;
uniform vec3 uLight2;

varying vec2 vTexCoord;
varying mat3 vNTMat;
varying vec3 vEyePos;

void main() {
  vec3 color = texture2D(uTexColor, vTexCoord).xyz;
  gl_FragColor = vec4(color, 1);
}

