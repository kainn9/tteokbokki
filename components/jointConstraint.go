package components

import (
	"github.com/kainn9/tteokbokki/math/matrix"
	"github.com/kainn9/tteokbokki/math/vec"
)

// A joint constraint between two rigid bodies.
// Can be used with a jointConstraintSolver to handle joint constraints.
type JointConstraint struct {
	A, B                        *RigidBody
	ALocal, BLocal              vec.Vec2
	Jacobian, JacobianTranspose *matrix.MatRC
	CachedLambda                []float64
	Bias                        float64
}

func NewJointConstraint(a, b *RigidBody) *JointConstraint {
	j := matrix.NewMatRC(1, 6)

	return &JointConstraint{
		A:            a,
		B:            b,
		ALocal:       a.WorldToLocalSpace(a.Pos),
		BLocal:       b.WorldToLocalSpace(a.Pos),
		CachedLambda: make([]float64, 1),
		Jacobian:     &j,
	}
}
