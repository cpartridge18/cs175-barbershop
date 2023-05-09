#version 150

uniform mat4 uProjMatrix;
uniform mat4 uModelViewMatrix;
uniform mat4 uNormalMatrix;

in vec3 aPosition;
in vec3 aNormal;
in vec3 aTangent;
in vec3 aBinormal;
in vec2 aTexCoord;

out vec2 vTexCoord;

void main() {
  vTexCoord = aTexCoord;
  vec4 posE = uModelViewMatrix * vec4(aPosition, 1.0);
  gl_Position = uProjMatrix * posE;
}
