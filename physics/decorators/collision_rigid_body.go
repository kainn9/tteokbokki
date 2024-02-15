package decorators

import entitysubset "github.com/kainn9/tteokbokki/physics/entity_subset"

type CollisionRigidBodyDecoratorFace interface {
	entitysubset.RigidBodyFace

	InverseMass() float64
	InverseAngularMass() float64
}

type CollisionRigidBodyDecorator struct {
	entitysubset.RigidBodyFace
}

func NewCollisionRigidBodyDecorator(rb entitysubset.RigidBodyFace) CollisionRigidBodyDecoratorFace {
	return CollisionRigidBodyDecorator{
		rb,
	}
}

func (crd CollisionRigidBodyDecorator) InverseMass() float64 {

	if crd.UnstoppableLinear() {
		return 0
	}

	return crd.RigidBodyFace.InverseMass()
}

func (crd CollisionRigidBodyDecorator) InverseMassAngularMass() float64 {

	if crd.UnstoppableAngular() {
		return 0
	}

	return crd.RigidBodyFace.InverseAngularMass()
}
