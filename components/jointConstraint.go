package tBokiComponents

import (
	tBokiMatrix "github.com/kainn9/tteokbokki/math/matrix"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

// A joint constraint between two rigid bodies.
// Can be used with a jointConstraintSolver to handle joint constraints.
type JointConstraint struct {
	ALocal, BLocal              tBokiVec.Vec2
	Jacobian, JacobianTranspose *tBokiMatrix.MatRC
	CachedLambda                []float64
	Bias                        float64
}

func NewJointConstraint(a, b *RigidBody) *JointConstraint {
	j := tBokiMatrix.NewMatRC(1, 6)

	return &JointConstraint{
		ALocal:       a.WorldToLocalSpace(a.Pos),
		BLocal:       b.WorldToLocalSpace(a.Pos),
		CachedLambda: make([]float64, 1),
		Jacobian:     &j,
	}
}
