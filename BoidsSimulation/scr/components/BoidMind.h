#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>


struct BoidMind
{
	glm::vec3 cohesion{ 1.0f };
	glm::vec3 separation{ 1.0f };
	glm::vec3 alignment{ 1.0f };

	float speed = 10.0f;
	float turnSpeed = 20.0f;
	float detectionRad = 1.5f;
	float avoidRad = 1.0f;
};



