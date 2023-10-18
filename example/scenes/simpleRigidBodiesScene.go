package scenes

import (
	"github.com/kainn9/coldBrew"
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/example/components"
	"github.com/kainn9/tteokbokki/example/constants"
	"github.com/kainn9/tteokbokki/example/factory"
	clientSystems "github.com/kainn9/tteokbokki/example/systems/client"
	renderSystems "github.com/kainn9/tteokbokki/example/systems/render"
	simSystems "github.com/kainn9/tteokbokki/example/systems/sim"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

type SimpleRigidBodiesSceneStruct struct{}

var SimpleRigidBodiesScene = &SimpleRigidBodiesSceneStruct{}

func (SimpleRigidBodiesSceneStruct) Index() string {
	return "simpleRigidBodiesScene"
}

func (SimpleRigidBodiesSceneStruct) New(m *coldBrew.Manager) *coldBrew.Scene {

	clicks := make([]tBokiVec.Vec2, 0)

	scene := coldBrew.NewScene(m, constants.GAME_WIDTH, constants.GAME_HEIGHT)

	scene.AddSystem(renderSystems.NewShapesRenderer())

	scene.AddSystem(clientSystems.NewClickTracker(&clicks))

	scene.AddSystem(simSystems.NewSpawnHandler(&clicks, scene))
	scene.AddSystem(simSystems.NewMovementHandler(scene))
	scene.AddSystem(simSystems.NewCollisionHandler(scene))
	scene.AddSystem(simSystems.NewJointHandler(scene))

	// floor
	floorEntity := scene.AddEntity(components.RigidBodyComponent)
	floorData := factory.BoundsBoxComponent(constants.GAME_WIDTH/2, (constants.GAME_HEIGHT/2)+300, constants.GAME_WIDTH, 40, 0)
	components.RigidBodyComponent.SetValue(floorEntity, floorData)

	// mid slant
	midSlantEntity := scene.AddEntity(components.RigidBodyComponent)
	midSlantData := factory.BoundsBoxComponent(constants.GAME_WIDTH/2+250, constants.GAME_HEIGHT-300, constants.GAME_WIDTH/2, 10, -0.5)
	components.RigidBodyComponent.SetValue(midSlantEntity, midSlantData)

	// walls
	wallLeftEntity := scene.AddEntity(components.RigidBodyComponent)
	wallLeftData := factory.BoundsBoxComponent(10, constants.GAME_HEIGHT/2, 10, constants.GAME_HEIGHT, 0)
	components.RigidBodyComponent.SetValue(wallLeftEntity, wallLeftData)

	wallRightEntity := scene.AddEntity(components.RigidBodyComponent)
	wallRightEntityData := factory.BoundsBoxComponent(constants.GAME_WIDTH-10, constants.GAME_HEIGHT/2, 10, constants.GAME_HEIGHT, 0)
	components.RigidBodyComponent.SetValue(wallRightEntity, wallRightEntityData)

	// Platform & anchor joint example(Angular).
	platformJointEntityAngular := scene.AddEntity(
		components.RigidBodyComponents,
		components.JointConstraintComponent,
	)

	platformBodiesAngular, platformJointDataAngular := factory.MovingPlatformThingy(200, 100, true)

	components.JointConstraintComponent.SetValue(platformJointEntityAngular, *platformJointDataAngular)

	components.RigidBodyComponents.SetValue(platformJointEntityAngular, []*tBokiComponents.RigidBody{
		platformBodiesAngular[0],
		platformBodiesAngular[1],
	})

	// Platform & anchor joint example(Linear).
	platformJointEntityLinear := scene.AddEntity(
		components.RigidBodyComponents,
		components.JointConstraintComponent,
	)

	platformBodiesLinear, platformJointDataLinear := factory.MovingPlatformThingy(220, 400, false)

	components.JointConstraintComponent.SetValue(platformJointEntityLinear, *platformJointDataLinear)

	components.RigidBodyComponents.SetValue(platformJointEntityLinear, []*tBokiComponents.RigidBody{
		platformBodiesLinear[0],
		platformBodiesLinear[1],
	})

	return scene
}
