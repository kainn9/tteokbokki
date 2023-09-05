package rigidBodiesSimSystems

import (
	componentPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/examples/ecsExample/ecs"
	"github.com/kainn9/tteokbokki/examples/ecsExample/globals"
	"github.com/kainn9/tteokbokki/physics"

	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type ApplyForces struct {
	Scene ecs.SceneAdmin
}

func (RigidBodiesStruct) NewApplyForcesSystem(scene ecs.SceneAdmin) ApplyForces {
	return ApplyForces{Scene: scene}
}

func (ApplyForces) Query() *donburi.Query {

	return donburi.NewQuery(
		filter.Or(
			filter.Contains(globals.RigidBodyComponent),
			filter.Contains(globals.RigidBodyComponents),
		),
	)
}

func (sys ApplyForces) Run(dt float64, entry *donburi.Entry) {

	if entry.HasComponent(globals.RigidBodyComponents) {
		bodies := globals.RigidBodyComponents.Get(entry)

		for _, body := range *bodies {

			runHelper(dt, body)
		}

		return
	}

	rb := globals.RigidBodyComponent.Get(entry)
	runHelper(dt, rb)

}

func runHelper(dt float64, body *componentPhysicsTypes.RigidBody) {
	weightForce, _ := physics.Force.NewWeightForce(body.GetMass())
	physics.Transformer.AddForce(body, weightForce)

	physics.Transformer.Integrate(body, dt)

	if body.Polygon != nil {
		body.UpdateVertices()
	}
}
