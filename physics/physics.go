package physics

import (
	"github.com/kainn9/tteokbokki/physics/collision"
	"github.com/kainn9/tteokbokki/physics/force"

	"github.com/kainn9/tteokbokki/physics/transform"

	"github.com/kainn9/tteokbokki/physics/constraint"
)

var Collision collision.CollisionChecker = collision.CollisionChecker{}
var Force force.ForcesFactory = force.ForcesFactory{}
var Transformer transform.Transformer = transform.Transformer{}
var Solver constraint.Solver = constraint.Solver{}
