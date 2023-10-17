package components

import (
	"github.com/kainn9/tteokbokki/math/matrix"
	"github.com/kainn9/tteokbokki/math/vec"
)

type PenConstraint struct {
	ACollisionPointLocal, BCollisionPointLocal vec.Vec2
	CollisionNormal                            vec.Vec2
	Jacobian, JacobianTranspose                *matrix.MatRC
	Friction                                   float64
	CachedLambda                               []float64
	Bias                                       float64
}

func NewPenConstraint(c Contact, a, b *RigidBody) *PenConstraint {
	j := matrix.NewMatRC(2, 6)

	aPoint := a.WorldToLocalSpace(c.Start)
	bPoint := b.WorldToLocalSpace(c.End)
	collisionNormalLocal := a.WorldToLocalSpace(c.Normal)

	return &PenConstraint{
		ACollisionPointLocal: aPoint,
		BCollisionPointLocal: bPoint,
		CollisionNormal:      collisionNormalLocal,
		Jacobian:             &j,
		CachedLambda:         make([]float64, 2),
	}
}
