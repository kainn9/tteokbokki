package scenes

import (
	"github/kainn9/tteobokkiExamples/factory"
	"github/kainn9/tteobokkiExamples/globals"

	"github.com/kainn9/coldBrew"
	"github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/math/vec"

	"github/kainn9/tteobokkiExamples/systems"
)

type SimpleRigidBodiesSceneStruct struct{}

var SimpleRigidBodiesScene = &SimpleRigidBodiesSceneStruct{}

func (SimpleRigidBodiesSceneStruct) Index() string {
	return "simpleRigidBodiesScene"
}

func (SimpleRigidBodiesSceneStruct) New(m *coldBrew.Manager) *coldBrew.Scene {

	clicks := make([]vec.Vec2, 0)

	scene := coldBrew.NewScene(m)

	scene.AddSystem(systems.RenderSystems.RigidBodies.NewRenderShapesSystem())

	scene.AddSystem(systems.ClientSystems.NewClickTrackerSystem(&clicks))

	scene.AddSystem(systems.SimSystems.RigidBodies.NewSpawn(&clicks, scene))

	scene.AddSystem(systems.SimSystems.RigidBodies.NewApplyForcesSystem(scene))
	scene.AddSystem(systems.SimSystems.RigidBodies.NewResolveCollisionsSystem(scene))
	scene.AddSystem(systems.SimSystems.RigidBodies.NewJointConstraintSystem(scene))

	// floor
	floorEntry := scene.AddEntity(globals.RigidBodyComponent)
	floorData := factory.BoundsBoxComponent(globals.GAME_WIDTH/2, (globals.GAME_HEIGHT/2)+300, globals.GAME_WIDTH, 40, 0)
	globals.RigidBodyComponent.SetValue(floorEntry, floorData)

	// top slant
	topSlantEntry := scene.AddEntity(globals.RigidBodyComponent)
	topSlantData := factory.BoundsBoxComponent(globals.GAME_WIDTH/2-200, globals.GAME_HEIGHT-600, globals.GAME_WIDTH/2, 10, 0.3)
	globals.RigidBodyComponent.SetValue(topSlantEntry, topSlantData)

	// mid slant
	midSlantEntry := scene.AddEntity(globals.RigidBodyComponent)
	midSlantData := factory.BoundsBoxComponent(globals.GAME_WIDTH/2+300, globals.GAME_HEIGHT-300, globals.GAME_WIDTH/2, 10, -0.5)
	globals.RigidBodyComponent.SetValue(midSlantEntry, midSlantData)

	// walls
	wallLeftEntry := scene.AddEntity(globals.RigidBodyComponent)
	wallLeftData := factory.BoundsBoxComponent(10, globals.GAME_HEIGHT/2, 10, globals.GAME_HEIGHT, 0)
	globals.RigidBodyComponent.SetValue(wallLeftEntry, wallLeftData)

	wallRightEntry := scene.AddEntity(globals.RigidBodyComponent)
	wallRightEntryData := factory.BoundsBoxComponent(globals.GAME_WIDTH-10, globals.GAME_HEIGHT/2, 10, globals.GAME_HEIGHT, 0)
	globals.RigidBodyComponent.SetValue(wallRightEntry, wallRightEntryData)

	// Circle in Middle
	circleMiddleEntry := scene.AddEntity(globals.RigidBodyComponent)
	circleMiddleData := factory.BoundsCircleComponent(globals.GAME_WIDTH/2, globals.GAME_HEIGHT/2, 100)
	globals.RigidBodyComponent.SetValue(circleMiddleEntry, circleMiddleData)

	// joint object
	jBodyDataA := components.NewRigidBodyCircle(200, 400, 10, 0, false)
	jBodyDataB := components.NewRigidBodyBox(jBodyDataA.Pos.X-120, jBodyDataA.Pos.Y, 120, 30, 5, true)

	jointBodies := []*components.RigidBodyComponent{
		jBodyDataA,
		jBodyDataB,
	}
	jBodyDataB.SetAngularMass(0)
	jBodyDataB.Unstoppable = true

	jointEntry := scene.AddEntity(globals.RigidBodyComponents, globals.JointConstraint)

	globals.RigidBodyComponents.SetValue(jointEntry, jointBodies)

	jcData := components.NewJointConstraint(jBodyDataA, jBodyDataB)

	globals.JointConstraint.SetValue(jointEntry, *jcData)

	return scene
}
