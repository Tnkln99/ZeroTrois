#version 450

layout (location = 0) in vec3 fragColor;
layout (location = 1) in vec3 fragPosWorld;
layout (location = 2) in vec3 fragNormalWorld;

layout(set = 0, binding = 0) uniform GlobalUbo{
	mat4 projectionMatrix;
	mat4 viewMatrix;
	vec4 ambientLightColor; // w is intesity
	vec3 lightPosition;
	vec4 lightColor;
} ubo;


layout(push_constant) uniform Push{
	mat4 modelMatrix; // projection * view * transform
	mat4 normalMatrix;
} push;

layout (location = 0) out vec4 outColor;

void main(){
	vec3 directionToLight = ubo.lightPosition - fragPosWorld;
	float attenuation = 1.0f / dot(directionToLight, directionToLight); // distance squared
	
	vec3 lightColor = ubo.lightColor.xyz * ubo.lightColor.w * attenuation;
	vec3 ambientLight = ubo.ambientLightColor.xyz * ubo.ambientLightColor.w;
	vec3 diffuseLight = lightColor * max(dot(normalize(fragNormalWorld), normalize(directionToLight)),0);

	outColor = vec4((diffuseLight + ambientLight) * fragColor, 1.0);
}