#include "BoidsManagerSystem.h"
#include "components/Transform.h"
#include "../components/BoidMind.h"
#include <math.h>


void BoidsManagerSystem::init()
{
}

void BoidsManagerSystem::update(zt::core::Coordinator& coordinator, float dt)
{
	// auto i = entities.begin();
	// auto transformLeader = coordinator.getComponent<zt::component::Transform>(*i);
	// auto leaderPos = transformLeader.translation;
	for(auto& boid1 : entities)
	{
 		auto& transform1 = coordinator.getComponent<zt::component::Transform>(boid1);
		const auto& boidMind = coordinator.getComponent<BoidMind>(boid1);
        constexpr float worldSize = 3.0f;
		if (transform1.translation.x < -worldSize) 
		{
			transform1.translation.x = worldSize;
		}
		else if (transform1.translation.x > worldSize) 
		{
			transform1.translation.x = -worldSize;
		}

		if (transform1.translation.y < -worldSize) 
		{
			transform1.translation.y = worldSize;
		}
		else if (transform1.translation.y > worldSize) 
		{
			transform1.translation.y = -worldSize;
		}

		if (transform1.translation.z < -worldSize) 
		{
			transform1.translation.z = worldSize;
		}
		else if (transform1.translation.z > worldSize) 
		{
			transform1.translation.z = -worldSize;
		}

		glm::vec3 steering{ 0.0f };

		glm::vec3 cohesion{ 0.0f };
		glm::vec3 separation{ 0.0f };
		glm::vec3 alignment{ 0.0f };

		std::uint32_t leaderBoid = *entities.begin();
		auto leaderAngle = 180.0f;

		int neighborCount = 0;
		int separationCount = 0;

		for (auto& boid2 : entities)
		{
			if (boid1 == boid2) { continue; }
			auto& transform2 = coordinator.getComponent<zt::component::Transform>(boid2);
			const float distance = glm::distance(transform1.translation, transform2.translation);
			if (distance < boidMind.detectionRad)
			{
				neighborCount++;
				alignment += transform2.getForwardVector();
				cohesion += transform2.translation;
				if (distance < boidMind.avoidRad)
				{
					separation += transform2.translation - transform1.translation;
					separationCount++;
				}
				auto angle = glm::angle(transform2.translation - transform1.translation, transform1.getForwardVector());
				if (angle < leaderAngle && angle < 90.0f)
				{
					leaderBoid = boid2;
					leaderAngle = angle;
				}
			}
		}

		//calculate average
		if (separationCount > 0)
		{
			separation /= separationCount;
		}
		if (neighborCount > 0)
		{
			alignment /= neighborCount;
			cohesion /= neighborCount;
		}
		if (leaderBoid != 0)
		{
			auto leaderTransform = coordinator.getComponent<zt::component::Transform>(leaderBoid);
			glm::vec3 leaderPos = leaderTransform.translation - transform1.translation;
			steering += leaderPos * 0.5f;
		}

		cohesion -= transform1.translation;
		separation = -separation;

		steering += cohesion * 0.6f + separation * 0.7f + alignment * 0.16f;// +obstacle;

		steering = -transform1.translation;

		

 		if(steering!= glm::vec3(0))
		{
			transform1.rotation = transform1.rotateTowards(transform1.rotation, steering, boidMind.turnSpeed * dt);
		}

		transform1.translation += transform1.transformDirection(glm::vec3{ 0.0,0.0, boidMind.speed}) * dt;
	}
}

bool BoidsManagerSystem::isPositionOutsideCube(glm::vec3 position, glm::vec3 cubeMin, glm::vec3 cubeMax)
{
	if (position.x < cubeMin.x || position.x > cubeMax.x ||
		position.y < cubeMin.y || position.y > cubeMax.y ||
		position.z < cubeMin.z || position.z > cubeMax.z)
	{
		return true; // position is outside the cube
	}
	return false; // position is inside the cube
}