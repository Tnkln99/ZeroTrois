#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/vector_angle.hpp>

namespace zt::component
{
    struct Transform
    {
	    glm::vec3 translation{};  // (position offset)
        glm::vec3 scale{ 1.f, 1.f, 1.f };
        glm::vec3 rotation{};

        [[nodiscard]] glm::mat4 mat4() const
        {
            const float c3 = glm::cos(rotation.z);
            const float s3 = glm::sin(rotation.z);
            const float c2 = glm::cos(rotation.x);
            const float s2 = glm::sin(rotation.x);
            const float c1 = glm::cos(rotation.y);
            const float s1 = glm::sin(rotation.y);
            return glm::mat4{
                {
                    scale.x * (c1 * c3 + s1 * s2 * s3),
                    scale.x * (c2 * s3),
                    scale.x * (c1 * s2 * s3 - c3 * s1),
                    0.0f,
                },
                {
                    scale.y * (c3 * s1 * s2 - c1 * s3),
                    scale.y * (c2 * c3),
                    scale.y * (c1 * c3 * s2 + s1 * s3),
                    0.0f,
                },
                {
                    scale.z * (c2 * s1),
                    scale.z * (-s2),
                    scale.z * (c1 * c2),
                    0.0f,
                },
                {translation.x, translation.y, translation.z, 1.0f} };
        }
        [[nodiscard]] glm::mat3 normalMatrix() const
        {
            const float c3 = glm::cos(rotation.z);
            const float s3 = glm::sin(rotation.z);
            const float c2 = glm::cos(rotation.x);
            const float s2 = glm::sin(rotation.x);
            const float c1 = glm::cos(rotation.y);
            const float s1 = glm::sin(rotation.y);
            const glm::vec3 invScale = 1.0f / scale;

            return glm::mat3{
                {
                    invScale.x * (c1 * c3 + s1 * s2 * s3),
                    invScale.x * (c2 * s3),
                    invScale.x * (c1 * s2 * s3 - c3 * s1),
                },
                {
                    invScale.y * (c3 * s1 * s2 - c1 * s3),
                    invScale.y * (c2 * c3),
                    invScale.y * (c1 * c3 * s2 + s1 * s3),
                },
                {
                    invScale.z * (c2 * s1),
                    invScale.z * (-s2),
                    invScale.z * (c1 * c2),
                },
            };
        }

        [[nodiscard]] glm::vec3 transformDirection(glm::vec3 direction) const
        {
	        const glm::mat3 rotation(mat4());
            return rotation * direction;
        }

        [[nodiscard]] glm::vec3 rotateTowards(const glm::vec3 from, const glm::vec3 to, const float maxDegrees) const
        {
	        const float angle = glm::angle(from, to);
	        const float maxRadians = glm::radians(maxDegrees);

            if (angle < maxRadians) {
                return to;
            }

	        const glm::vec3 axis = normalize(cross(from, to));
	        const glm::quat q = angleAxis(maxRadians, axis);
            return rotate(q, from);
         }

        [[nodiscard]] glm::vec3 getForwardVector() const
        {
            glm::quat rotation = glm::quat_cast(glm::mat3_cast(rotation));
            const glm::vec3 forward = glm::rotate(rotation, glm::vec3(0.f, 0.f, -1.f));
            return glm::normalize(forward * scale);
        }
    };
}


