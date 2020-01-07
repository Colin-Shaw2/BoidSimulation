#version 330 core

uniform mat4 view;
uniform mat4 projection;


uniform mat4 model;

in vec4 vPosition;
in vec3 vNormal;
out vec3 normal;
out vec3 position;

void main() {
	//gl_Position = projection * modelView * vPosition;
	gl_Position = projection * view * model * vPosition;
	position = vPosition.xyz;
	normal = position;
	
}