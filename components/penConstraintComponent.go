package components

import (
	"github.com/kainn9/tteokbokki/math/matrix"
	"github.com/kainn9/tteokbokki/math/vec"
)

type PenConstraintComponent struct {
	A, B                                       *RigidBodyComponent
	ACollisionPointLocal, BCollisionPointLocal vec.Vec2
	CollisionNormal                            vec.Vec2
	Jacobian, JacobianTranspose                *matrix.MatRC
	Friction                                   float64
	CachedLambda                               []float64
	Bias                                       float64
}

func NewPenConstraint(c ContactComponent) *PenConstraintComponent {
	j := matrix.NewMatRC(2, 6)

	aPoint := c.A.WorldToLocalSpace(c.Start)
	bPoint := c.B.WorldToLocalSpace(c.End)
	collisionNormalLocal := c.A.WorldToLocalSpace(c.Normal)

	return &PenConstraintComponent{
		A:                    c.A,
		B:                    c.B,
		ACollisionPointLocal: aPoint,
		BCollisionPointLocal: bPoint,
		CollisionNormal:      collisionNormalLocal,
		Jacobian:             &j,
		CachedLambda:         make([]float64, 2),
	}
}
