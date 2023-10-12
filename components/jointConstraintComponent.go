package components

import (
	"github.com/kainn9/tteokbokki/math/matrix"
	"github.com/kainn9/tteokbokki/math/vec"
)

// A joint constraint between two rigid bodies.
// Can be used with a jointConstraintSolver to handle joint constraints.
type JointConstraintComponent struct {
	A, B                        *RigidBodyComponent
	ALocal, BLocal              vec.Vec2
	Jacobian, JacobianTranspose *matrix.MatRC
	CachedLambda                []float64
	Bias                        float64
}

func NewJointConstraint(a, b *RigidBodyComponent) *JointConstraintComponent {
	j := matrix.NewMatRC(1, 6)

	return &JointConstraintComponent{
		A:            a,
		B:            b,
		ALocal:       a.WorldToLocalSpace(a.Pos),
		BLocal:       b.WorldToLocalSpace(a.Pos),
		CachedLambda: make([]float64, 1),
		Jacobian:     &j,
	}
}
