package physics_components

import (
	"github.com/kainn9/tteokbokki/physics/matrix"
	"github.com/kainn9/tteokbokki/vector"
)

type PenConstraint struct {
	ACollisionPointLocal        vector.Vec2Face
	BCollisionPointLocal        vector.Vec2Face
	Normal                      vector.Vec2Face
	Jacobian, JacobianTranspose matrix.Mat2
	CachedLambda                []float64
	Friction                    float64
	Bias                        float64
}

func NewPenConstraint(c Collision) {}
