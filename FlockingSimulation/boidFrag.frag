#version 330 core
#define M_PI 3.1415926535897932384626433832795

in vec3 normal;
in vec3 position;
uniform vec3 light;
uniform vec4 material;

//this int is a flag to tell what we are working on
uniform int isPred;

vec4 sample();

void main() {
	vec4 white = vec4(1.0, 1.0, 1.0, 1.0);
	float diffuse;
	vec3 L = normalize(light - position);
	//vec3 L = normalize(vec3(1.0, 1.0, -10.0));
	vec3 N;
	vec3 R = normalize(reflect(-L,normal));
	float specular;
	vec4 colour;
	N = normalize(normal);
	diffuse = dot(N,L);
	
	if(diffuse < 0.0) {
		diffuse = 0.0;
		specular = 0.0;
	} else {
		specular = pow(max(0.0, dot(N,R)),material.w);
	}

	colour = vec4(0.1, 0.0, 0.1, 1.0);
	
	if(isPred==2){
	colour = vec4(1.0, 0.0, 1.0, 1.0);
	}
	
	if(isPred==1){
		colour = vec4(1.0, 0.0, 0.0, 1.0);
		
	}else if(isPred==0){
		colour = vec4(0.0, 0.0, 1.0, 1.0);

	}

	gl_FragColor = min(material.x*colour + material.y*diffuse*colour + material.z*white*specular, vec4(1.0));

	gl_FragColor.a = colour.a;
}


