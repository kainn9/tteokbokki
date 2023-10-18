package simSystems

import (
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/example/components"

	"github.com/kainn9/coldBrew"
	tBokiPhysics "github.com/kainn9/tteokbokki/physics"

	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type MovementHandler struct {
	Scene *coldBrew.Scene
}

func NewMovementHandler(scene *coldBrew.Scene) MovementHandler {
	return MovementHandler{Scene: scene}
}

func (MovementHandler) Query() *donburi.Query {
	return donburi.NewQuery(
		filter.Or(
			filter.Contains(components.RigidBodyComponent),
			filter.Contains(components.RigidBodyComponents),
		),
	)
}

func (sys MovementHandler) Run(dt float64, entity *donburi.Entry) {
	// When the entity has a single RigidBodyComponent.
	if entity.HasComponent(components.RigidBodyComponent) {
		body := components.RigidBodyComponent.Get(entity)
		runHelper(body, dt)
	}

	// When the entity has multiple RigidBodyComponents.
	if entity.HasComponent(components.RigidBodyComponents) {
		bodies := *components.RigidBodyComponents.Get(entity)

		for _, body := range bodies {
			runHelper(body, dt)
		}
	}

}

func runHelper(body *tBokiComponents.RigidBody, dt float64) {
	weightForce, _ := tBokiPhysics.ForceFactory.NewWeightForce(body.GetMass())
	tBokiPhysics.Transformer.AddForce(body, weightForce)

	tBokiPhysics.Transformer.Integrate(body, dt)

	if body.Polygon != nil {
		body.UpdateVertices()
	}
}
