#pragma once
#include "../core/System.h"
#include "../core/Coordinator.h"


namespace zt::system
{
	class CameraControllerSystem : public core::System
	{
	public:
		void init();

		void update(core::Coordinator& coordinator, float aspect, float dt);
	private:
		float moveSpeed{ 3.f };
		float lookSpeed{ 1.5f };
	};
}


