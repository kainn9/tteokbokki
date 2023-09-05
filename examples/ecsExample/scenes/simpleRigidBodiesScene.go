package scenes

import (
	"github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/examples/ecsExample/ecs"
	"github.com/kainn9/tteokbokki/examples/ecsExample/factory"
	"github.com/kainn9/tteokbokki/examples/ecsExample/globals"
	"github.com/kainn9/tteokbokki/math/vec"

	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/examples/ecsExample/systems"
)

func NewSimpleRigidBodiesScene() ecs.SceneAdmin {

	clicks := make([]vec.Vec2, 0)

	sceneAdmin := ecs.NewSceneAdmin()

	sceneAdmin.AddSystem(systems.RenderSystems.RigidBodies.NewRenderShapesSystem())

	sceneAdmin.AddSystem(systems.ClientSystems.NewClickTrackerSystem(&clicks))

	sceneAdmin.AddSystem(systems.SimSystems.RigidBodies.NewSpawn(&clicks, sceneAdmin))

	sceneAdmin.AddSystem(systems.SimSystems.RigidBodies.NewApplyForcesSystem(sceneAdmin))
	sceneAdmin.AddSystem(systems.SimSystems.RigidBodies.NewResolveCollisionsSystem(sceneAdmin))
	sceneAdmin.AddSystem(systems.SimSystems.RigidBodies.NewJointConstraintSystem(sceneAdmin))

	// floor
	floorEntry := sceneAdmin.AddEntity(globals.RigidBodyComponent)
	floorData := factory.BoundsBoxComponent(globals.GAME_WIDTH/2, (globals.GAME_HEIGHT/2)+300, globals.GAME_WIDTH, 40, 0)
	globals.RigidBodyComponent.SetValue(floorEntry, floorData)

	// top slant
	topSlantEntry := sceneAdmin.AddEntity(globals.RigidBodyComponent)
	topSlantData := factory.BoundsBoxComponent(globals.GAME_WIDTH/2-200, globals.GAME_HEIGHT-600, globals.GAME_WIDTH/2, 10, 0.3)
	globals.RigidBodyComponent.SetValue(topSlantEntry, topSlantData)

	// mid slant
	midSlantEntry := sceneAdmin.AddEntity(globals.RigidBodyComponent)
	midSlantData := factory.BoundsBoxComponent(globals.GAME_WIDTH/2+300, globals.GAME_HEIGHT-300, globals.GAME_WIDTH/2, 10, -0.5)
	globals.RigidBodyComponent.SetValue(midSlantEntry, midSlantData)

	// walls
	wallLeftEntry := sceneAdmin.AddEntity(globals.RigidBodyComponent)
	wallLeftData := factory.BoundsBoxComponent(10, globals.GAME_HEIGHT/2, 10, globals.GAME_HEIGHT, 0)
	globals.RigidBodyComponent.SetValue(wallLeftEntry, wallLeftData)

	wallRightEntry := sceneAdmin.AddEntity(globals.RigidBodyComponent)
	wallRightEntryData := factory.BoundsBoxComponent(globals.GAME_WIDTH-10, globals.GAME_HEIGHT/2, 10, globals.GAME_HEIGHT, 0)
	globals.RigidBodyComponent.SetValue(wallRightEntry, wallRightEntryData)

	// Circle in Middle
	circleMiddleEntry := sceneAdmin.AddEntity(globals.RigidBodyComponent)
	circleMiddleData := factory.BoundsCircleComponent(globals.GAME_WIDTH/2, globals.GAME_HEIGHT/2, 100)
	globals.RigidBodyComponent.SetValue(circleMiddleEntry, circleMiddleData)

	// joint object
	jBodyDataA := components.Physics.NewRigidBodyCircle(200, 400, 10, 0, false)
	jBodyDataB := components.Physics.NewRigidBodyBox(jBodyDataA.Pos.X-120, jBodyDataA.Pos.Y, 120, 30, 5, true)

	jointBodies := []*componentsPhysicsTypes.RigidBody{
		jBodyDataA,
		jBodyDataB,
	}
	jBodyDataB.SetAngularMass(0)
	jBodyDataB.Unstoppable = true

	jointEntry := sceneAdmin.AddEntity(globals.RigidBodyComponents, globals.JointConstraint)

	globals.RigidBodyComponents.SetValue(jointEntry, jointBodies)

	jcData := components.Physics.NewJointConstraint(jBodyDataA, jBodyDataB)

	globals.JointConstraint.SetValue(jointEntry, *jcData)

	return sceneAdmin
}
