#include "BoidsManagerSystem.h"
#include "components/Transform.h"
#include "../components/BoidMind.h"
#include "utilities/Timer.h"
#include <math.h>
#include <random>


void BoidsManagerSystem::init()
{
	srand(time(0)); // seed the random number generator with current time
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

        constexpr float worldSize = 5.0f;

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
		glm::vec3 leader{ 0 };

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
			separation = -separation;
			if(separation != glm::vec3{0})
			{
				separation = normalize(separation);
			}
		}
		if (neighborCount > 0)
		{
			alignment /= neighborCount;
			
			cohesion /= neighborCount;
			cohesion -= transform1.translation;

			if(cohesion!=glm::vec3{0})
			{
				cohesion = normalize(cohesion);
			}
			if (alignment != glm::vec3{ 0 })
			{
				alignment = normalize(cohesion);
			}
		}

		if (leaderBoid != 0)
		{
			auto leaderTransform = coordinator.getComponent<zt::component::Transform>(leaderBoid);
			leader = leaderTransform.translation - transform1.translation;
			leader = normalize(leader);
		}

		// avoidance for cube boundaries
		// float avoidDistance = 1.0f;
        // auto avoid = glm::vec3 {0};
		// if (std::abs(transform1.translation.x - (-worldSize)) < avoidDistance) { avoid.x = 1.0f; }
		// else if (std::abs(transform1.translation.x - (worldSize)) < avoidDistance) { avoid.x = -1.0f; }
		// 
		// if (std::abs(transform1.translation.y - (-worldSize)) < avoidDistance) { avoid.y = 1.0f; }
		// else if (std::abs(transform1.translation.y - (worldSize)) < avoidDistance) { avoid.y = -1.0f; }
		// 
		// if (std::abs(transform1.translation.z - (-worldSize)) < avoidDistance) { avoid.z = 1.0f; }
		// else if (std::abs(transform1.translation.z - (worldSize)) < avoidDistance) { avoid.z = -1.0f; }

		steering += cohesion * 0.6f + separation * 0.7f + alignment * 0.36f + leader * 0.5f;
		

		//glm::vec3 centerOfInterest = normalize(glm::vec3{ 0 } - transform1.translation);
		//steering += centerOfInterest;


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