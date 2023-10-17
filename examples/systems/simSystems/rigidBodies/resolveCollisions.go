package rigidBodiesSimSystems

import (
	"github.com/kainn9/coldBrew"
	"github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/examples/globals"

	"github.com/kainn9/tteokbokki/physics"
	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type ResolveCollisions struct {
	Scene coldBrew.Scene
}

func (RigidBodiesStruct) NewResolveCollisionsSystem(scene *coldBrew.Scene) ResolveCollisions {
	return ResolveCollisions{Scene: *scene}
}

func (ResolveCollisions) Query() *donburi.Query {

	return donburi.NewQuery(
		filter.Or(
			filter.Contains(globals.RigidBodyComponent),
			filter.Contains(globals.RigidBodyComponents),
		),
	)
}

func (sys ResolveCollisions) Run(dt float64, _ *donburi.Entry) {
	query := sys.Query()
	w := sys.Scene.World

	bodies := make([]*components.RigidBody, 0)

	query.Each(w, func(entry *donburi.Entry) {

		if entry.HasComponent(globals.RigidBodyComponent) {
			bodies = append(bodies, globals.RigidBodyComponent.Get(entry))

		} else {
			groupEntries := globals.RigidBodyComponents.Get(entry)
			bodies = append(bodies, *groupEntries...)
		}

	})

	bodiesLength := len(bodies)

	for i := 0; i <= bodiesLength-1; i++ {
		for j := i + 1; j < bodiesLength; j++ {
			a := bodies[i]
			b := bodies[j]

			if isColliding, contacts := physics.Collision.Check(a, b); isColliding {

				for _, contact := range contacts {
					pc := components.NewPenConstraint(contact, a, b)
					physics.Solver.PreSolvePenConstraint(pc, a, b, dt)
					physics.Solver.SolvePenConstraint(pc, a, b)
				}

			}
		}
	}
}
