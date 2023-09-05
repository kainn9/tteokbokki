package rigidBodiesSimSystems

import (
	"github.com/kainn9/tteokbokki/components"
	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/examples/ecsExample/ecs"
	"github.com/kainn9/tteokbokki/examples/ecsExample/globals"

	"github.com/kainn9/tteokbokki/physics"
	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type ResolveCollisions struct {
	Scene ecs.SceneAdmin
}

func (RigidBodiesStruct) NewResolveCollisionsSystem(scene ecs.SceneAdmin) ResolveCollisions {
	return ResolveCollisions{Scene: scene}
}

func (ResolveCollisions) CustomIteration() bool {
	return true
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

	bodies := make([]*componentsPhysicsTypes.RigidBody, 0)

	query.Each(w, func(entry *donburi.Entry) {

		if entry.HasComponent(globals.RigidBodyComponent) {
			bodies = append(bodies, globals.RigidBodyComponent.Get(entry))

		} else {
			groupEntries := globals.RigidBodyComponents.Get(entry)

			for i := range *groupEntries {
				e := (*groupEntries)[i]
				bodies = append(bodies, e)
			}
		}

	})

	bodiesLength := len(bodies)

	for i := 0; i <= bodiesLength-1; i++ {
		for j := i + 1; j < bodiesLength; j++ {
			a := bodies[i]
			b := bodies[j]

			if isColliding, contact := physics.Collision.Check(a, b); isColliding {
				// physics.Collision.Resolver.Impulse(contact)
				pc := components.Physics.NewPenConstraint(contact)
				physics.Solver.PreSolvePenConstraint(pc, dt)

				for i := 0; i < 100; i++ {
					physics.Solver.SolvePenConstraint(pc)
				}

			}
		}
	}
}
