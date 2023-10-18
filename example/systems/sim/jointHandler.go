package simSystems

import (
	"github.com/kainn9/coldBrew"
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/example/components"
	tBokiPhysics "github.com/kainn9/tteokbokki/physics"

	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type JointHandler struct {
	Scene *coldBrew.Scene
}

func NewJointHandler(scene *coldBrew.Scene) JointHandler {
	return JointHandler{Scene: scene}
}

func (JointHandler) Query() *donburi.Query {

	return donburi.NewQuery(
		filter.Contains(components.JointConstraintComponent),
	)
}

func (JointHandler) Run(dt float64, entity *donburi.Entry) {

	jointConstraint := components.JointConstraintComponent.Get(entity)

	bodies := components.RigidBodyComponents.Get(entity)

	jointConstraints := []*tBokiComponents.JointConstraint{jointConstraint}

	aBodies := []*tBokiComponents.RigidBody{(*bodies)[0]}
	bBodies := []*tBokiComponents.RigidBody{(*bodies)[1]}

	// More iterations = more accurate simulation but less performant.
	// Here we are using 5 iterations.
	tBokiPhysics.Solver.Solve(jointConstraints, aBodies, bBodies, 5, dt)

}
