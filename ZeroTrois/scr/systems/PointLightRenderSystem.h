#pragma once
#include "../graphics/Pipeline.h"
#include "../core/System.h"
#include "../core/Coordinator.h"
#include "../graphics/FrameInfo.h"


namespace zt::system
{
	class PointLightRenderSystem : public core::System
	{
	public:
		void init(graphics::Device& device, VkRenderPass renderPass, VkDescriptorSetLayout globalSetLayout);

		void update(core::Coordinator& coordinator, const graphics::FrameInfo& renderUpdateInfo) const;

		void clean(graphics::Device& device) const;
	private:
		void createPipelineLayout(VkDescriptorSetLayout globalSetLayout, graphics::Device& device);
		void createPipeline(VkRenderPass renderPass, graphics::Device& device);

		std::unique_ptr<graphics::Pipeline> pipeline;
		VkPipelineLayout pipelineLayout{};
	};
}
