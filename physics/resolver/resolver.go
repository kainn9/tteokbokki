package resolver

import (
	physics_components "github.com/kainn9/tteokbokki/physics/components"
	"github.com/kainn9/tteokbokki/physics/decorators"
	entitysubset "github.com/kainn9/tteokbokki/physics/entity_subset"

	"github.com/kainn9/tteokbokki/physics/physics"
	"github.com/kainn9/tteokbokki/vector"

	transform_components "github.com/kainn9/tteokbokki/transform/components"
)

func HandleCollision_Multi(
	collision *physics_components.Collision,

	transA, transB transform_components.TransformFace,
	shapeA, shapeB transform_components.ShapeFace,
	physA, physB physics_components.PhysicsFace,
) {
	bodyA := entitysubset.NewRigidBody(transA, shapeA, physA)
	bodyB := entitysubset.NewRigidBody(transB, shapeB, physB)

	HandleCollision(collision, bodyA, bodyB)
}

func HandleCollision(
	collision *physics_components.Collision,
	bodyA, bodyB entitysubset.RigidBodyFace,
) {
	if bodyCannotMoveOrRotate(bodyA) && bodyCannotMoveOrRotate(bodyB) {
		return
	}
	decoratedA := decorators.NewCollisionRigidBodyDecorator(bodyA)
	decoratedB := decorators.NewCollisionRigidBodyDecorator(bodyB)

	resolveWithProjection(collision, decoratedA, decoratedB)
	resolveWithImpulse(collision, decoratedA, decoratedB)
}

func bodyCannotMoveOrRotate(body entitysubset.RigidBodyFace) bool {
	static := physics.Util.IsStaticLinear(body) && physics.Util.IsStaticAngular(body)
	unstoppable := (body.UnstoppableLinear() && body.UnstoppableAngular())

	return static || unstoppable

}

func resolveWithProjection(
	collision *physics_components.Collision,
	bodyA, bodyB decorators.CollisionRigidBodyDecoratorFace,
) {

	// Note: da + db will always equal c.Depth.
	displacementFactorA := collision.Depth /
		(bodyA.InverseMass() + bodyB.InverseMass()) * bodyA.InverseMass()

	displacementFactorB := collision.Depth /
		(bodyA.InverseMass() + bodyB.InverseMass()) * bodyB.InverseMass()

	newAPos := bodyA.Position().Sub(collision.Normal.Scale(displacementFactorA))
	bodyA.SetPosition(newAPos)

	newBPos := bodyB.Position().Add(collision.Normal.Scale(displacementFactorB))
	bodyB.SetPosition(newBPos)

}

func resolveWithImpulse(
	collision *physics_components.Collision,
	bodyA, bodyB decorators.CollisionRigidBodyDecoratorFace,
) {
	linearImpulseA, linearImpulseB, collisionDisplacementA, collisionDisplacementB :=
		calculateResolutionImpulses(
			collision,
			bodyA,
			bodyB,
		)

	physics.ApplyImpulse(bodyA, linearImpulseA, collisionDisplacementA)
	physics.ApplyImpulse(bodyB, linearImpulseB, collisionDisplacementB)

}

func calculateResolutionImpulses(
	collision *physics_components.Collision,
	bodyA, bodyB decorators.CollisionRigidBodyDecoratorFace,
) (
	collisionImpulseA,
	collisionImpulseB,
	collisionDisplacementA,
	collisionDisplacementB vector.Vec2Face,
) {

	// Calculate average elasticity and friction.
	elasticity := (bodyA.Elasticity() + bodyB.Elasticity()) / 2
	friction := (bodyA.Friction() + bodyB.Friction()) / 2

	// Calculate relative positions of body A and body B.
	relativePositionA := collision.End.Sub(bodyA.Position())
	relativePositionB := collision.Start.Sub(bodyB.Position())

	// Calculate relative velocities of body A and body B.
	relativeVelocityA := bodyA.Vel().Add(
		vector.NewVec2(
			-bodyA.AngularVel()*relativePositionA.Y(),
			bodyA.AngularVel()*relativePositionA.X(),
		))
	relativeVelocityB := bodyB.Vel().Add(
		vector.NewVec2(
			-bodyB.AngularVel()*relativePositionB.Y(),
			bodyB.AngularVel()*relativePositionB.X(),
		))
	relativeVelocity := relativeVelocityA.Sub(relativeVelocityB)

	// Calculate relative velocity dot product with the collision normal.
	relativeVelocityDotNormal := relativeVelocity.ScalarProduct(collision.Normal)

	// Calculate impulse direction along the collision normal.
	impulseDirectionNormal := collision.Normal

	// Calculate cross products of relative positions with the collision normal.
	crossNormalA := relativePositionA.CrossProduct(collision.Normal)
	crossNormalASq := crossNormalA * crossNormalA
	crossNormalB := relativePositionB.CrossProduct(collision.Normal)
	crossNormalBSq := crossNormalB * crossNormalB

	// Calculate the denominator for normal impulse calculation.
	abInverseMassSum := bodyA.InverseMass() + bodyB.InverseMass()
	denominatorNormal := abInverseMassSum +
		crossNormalASq*bodyA.InverseAngularMass() +
		crossNormalBSq*bodyB.InverseAngularMass()

	// Calculate magnitude of the normal impulse.
	impulseMagnitudeNormal := -(1 + elasticity) * relativeVelocityDotNormal / denominatorNormal

	// Calculate normal impulse vector.
	impulseNormal := impulseDirectionNormal.Scale(impulseMagnitudeNormal)

	// Calculate tangent vector.
	tangent := collision.Normal.Perpendicular().Norm()

	// Calculate relative velocity dot product with the tangent vector.
	relativeVelocityDotTangent := relativeVelocity.ScalarProduct(tangent)

	// Calculate impulse direction along the tangent.
	impulseDirectionTangent := tangent

	// Calculate cross products of relative positions with the tangent.
	crossTangentA := relativePositionA.CrossProduct(tangent)
	crossTangentASq := crossTangentA * crossTangentA
	crossTangentB := relativePositionB.CrossProduct(tangent)
	crossTangentBSq := crossTangentB * crossTangentB

	// Calculate the denominator for tangent impulse calculation.
	denominatorTangent := abInverseMassSum +
		crossTangentASq*bodyA.InverseAngularMass() +
		crossTangentBSq*bodyB.InverseAngularMass()

	// Calculate magnitude of the tangent impulse.
	impulseMagnitudeTangent := friction * -(1 + elasticity) * relativeVelocityDotTangent / denominatorTangent

	// Calculate tangent impulse vector.
	impulseTangent := impulseDirectionTangent.Scale(impulseMagnitudeTangent)

	// Return the sum of normal and tangent impulses along with relative positions of bodies.
	impulseA := impulseNormal.Add(impulseTangent)
	impulseB := impulseA.Scale(-1)
	return impulseA, impulseB, relativePositionA, relativePositionB
}
