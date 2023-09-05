package rigidBodiesSimSystems

import (
	"github.com/kainn9/tteokbokki/examples/ecsExample/ecs"
	"github.com/kainn9/tteokbokki/examples/ecsExample/globals"

	"github.com/kainn9/tteokbokki/physics"
	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type JointConstraint struct {
	Scene ecs.SceneAdmin
}

func (RigidBodiesStruct) NewJointConstraintSystem(scene ecs.SceneAdmin) JointConstraint {
	return JointConstraint{Scene: scene}
}

func (JointConstraint) Query() *donburi.Query {

	return donburi.NewQuery(
		filter.Contains(globals.RigidBodyComponents, globals.JointConstraint),
	)
}

func (JointConstraint) Run(dt float64, entry *donburi.Entry) {
	jc := globals.JointConstraint.Get(entry)
	physics.Solver.PreSolveJointConstraintLinear(jc, dt)
	physics.Solver.SolveJointConstraintLinear(jc)
}
