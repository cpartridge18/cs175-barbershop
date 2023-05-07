#version 150

uniform mat4 uProjMatrix;
uniform mat4 uModelViewMatrix;
uniform mat4 uNormalMatrix;

in vec3 aPosition;
in vec2 aTexCoord;

out vec2 vTexCoord;

void main() {
  vTexCoord = aTexCoord;
  vNTMat = mat3(uNormalMatrix) * mat3(aTangent, aBinormal, aNormal);
  vec4 posE = uModelViewMatrix * vec4(aPosition, 1.0);
  gl_Position = uProjMatrix * posE;
}

