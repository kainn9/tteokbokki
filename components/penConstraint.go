package tBokiComponents

import (
	tBokiMatrix "github.com/kainn9/tteokbokki/math/matrix"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

type PenConstraint struct {
	ACollisionPointLocal, BCollisionPointLocal tBokiVec.Vec2
	CollisionNormal                            tBokiVec.Vec2
	Jacobian, JacobianTranspose                *tBokiMatrix.MatRC
	Friction                                   float64
	CachedLambda                               []float64
	Bias                                       float64
}

func NewPenConstraint(c Contact, a, b *RigidBody) *PenConstraint {
	j := tBokiMatrix.NewMatRC(2, 6)

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
