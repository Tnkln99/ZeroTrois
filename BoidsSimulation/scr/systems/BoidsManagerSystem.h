#pragma once
#include <glm/vec3.hpp>

#include "core/System.h"

#include "core/Coordinator.h"

class BoidsManagerSystem : public zt::core::System
{
public:
	void init();

	void update(zt::core::Coordinator& coordinator, float dt);
private:
	bool isPositionOutsideCube(glm::vec3 position, glm::vec3 cubeMin, glm::vec3 cubeMax);

};

