package simSystems

import (
	"github.com/kainn9/coldBrew"
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/example/components"
	tBokiPhysics "github.com/kainn9/tteokbokki/physics"

	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type CollisionHandler struct {
	Scene coldBrew.Scene
}

func NewCollisionHandler(scene *coldBrew.Scene) CollisionHandler {
	return CollisionHandler{Scene: *scene}
}

func (CollisionHandler) CustomQuery() *donburi.Query {
	return donburi.NewQuery(
		filter.Or(
			filter.Contains(components.RigidBodyComponent),
			filter.Contains(components.RigidBodyComponents),
		),
	)
}

func (sys CollisionHandler) Run(dt float64, _ *donburi.Entry) {
	query := sys.CustomQuery()

	w := sys.Scene.World

	bodies := make([]*tBokiComponents.RigidBody, 0)

	query.Each(w, func(entity *donburi.Entry) {

		// When the entity has a single RigidBodyComponent.
		if entity.HasComponent(components.RigidBodyComponent) {
			bodies = append(bodies, components.RigidBodyComponent.Get(entity))
		}

		// When the entity has multiple RigidBodyComponents.
		if entity.HasComponent(components.RigidBodyComponents) {
			bodies = append(bodies, *components.RigidBodyComponents.Get(entity)...)
		}

	})

	bodiesLength := len(bodies)

	// Important to store these outside of the loop.
	penConstraints := make([]*tBokiComponents.PenConstraint, 0)
	aBodies := make([]*tBokiComponents.RigidBody, 0)
	bBodies := make([]*tBokiComponents.RigidBody, 0)

	for i := 0; i <= bodiesLength-1; i++ {
		for j := i + 1; j < bodiesLength; j++ {
			a := bodies[i]
			b := bodies[j]

			if isColliding, contacts := tBokiPhysics.Detector.Detect(a, b); isColliding {

				for _, contact := range contacts {
					penConstraints = append(penConstraints, tBokiComponents.NewPenConstraint(contact, a, b))
					aBodies = append(aBodies, a)
					bBodies = append(bBodies, b)
				}

			}
		}
	}

	// More iterations = more stable but slower.
	// Here we choose 10.
	tBokiPhysics.Solver.Solve(penConstraints, aBodies, bBodies, 10, dt)
}
