#include "Engine.h"
#include "core/Coordinator.h"

#include "components/Camera.h"
#include "components/Light.h"
#include "components/Model.h"
#include "components/Transform.h"
#include "systems/CameraControllerSystem.h"
#include "utilities/Timer.h"

#include "components/BoidMind.h"

#include <cstdlib>
#include <iostream>
#include <random>

#include "systems/BoidsManagerSystem.h"


int main() {
    zt::Engine engine{};
    zt::core::Coordinator coordinator{};
    engine.init(coordinator);

    coordinator.registerComponent<BoidMind>();

    auto cameraControllerSystem = coordinator.registerSystem<zt::system::CameraControllerSystem>();
    {
        zt::core::Signature signature;
        signature.set(coordinator.getComponentType<zt::component::Camera>());
        signature.set(coordinator.getComponentType<zt::component::Transform>());
        coordinator.setSystemSignature<zt::system::CameraControllerSystem>(signature);
    }
    cameraControllerSystem->init();

    auto boidManagerSystem = coordinator.registerSystem<BoidsManagerSystem>();
    {
        zt::core::Signature signature;
        signature.set(coordinator.getComponentType<BoidMind>());
        signature.set(coordinator.getComponentType<zt::component::Transform>());
        coordinator.setSystemSignature<BoidsManagerSystem>(signature);
    }
    boidManagerSystem->init();

    std::vector<zt::core::Entity> boids( 50 );
    std::default_random_engine generator;
    std::uniform_real_distribution randPosition(-1.0f, 1.0f);
    std::uniform_real_distribution randRotation(0.0f, 3.0f);
    std::shared_ptr model = engine.graphicEngine.loadModel("models/capsule.obj");
    for (auto& entity : boids)
    {

        entity = coordinator.createEntity();
        coordinator.addComponent(
            entity,
            zt::component::Transform{
                {randPosition(generator), randPosition(generator), randPosition(generator)},
                { 0.07f, 0.07f, 0.07f },
                //{randRotation(generator), randRotation(generator), randRotation(generator)}
                {1.0f, 1.0f, 1.0f}
            });

        coordinator.addComponent(entity,
            zt::component::Model{ model }
        );
        coordinator.addComponent(entity,
            BoidMind{}
        );
    }

    engine.graphicEngine.camera = coordinator.createEntity();
    zt::component::Transform cameraTransform{};
    cameraTransform.translation.z = -10.0f;
    //cameraTransform.translation.x = 2.0f;
    //cameraTransform.translation.y = 2.0f;
    coordinator.addComponent(
        engine.graphicEngine.camera,
        cameraTransform
    );
    zt::component::Camera cameraComp{};
    coordinator.addComponent(
        engine.graphicEngine.camera,
        cameraComp
    );

    zt::core::Entity light = coordinator.createEntity();
    zt::component::Light compLight{};
    compLight.color = { 0.3f, 0.2f, 1.0f, 1.0f };
    zt::component::Transform lightTransform{};// { glm::vec3{ 2.0f } };

    coordinator.addComponent(
        light,
        compLight
    );
    coordinator.addComponent(
        light,
        lightTransform
    );


    try {
        while (!engine.shouldCloseWindow())
        {
            float dt = zt::utilities::Timer::getDeltaTime();
            engine.update(coordinator);
            cameraControllerSystem->update(coordinator, engine.graphicEngine.getAspectRatio(), dt);
            boidManagerSystem->update(coordinator, dt);
        }
        engine.terminate();
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
