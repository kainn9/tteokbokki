package physics

import (
	physics_components "github.com/kainn9/tteokbokki/physics/components"
	entitysubset "github.com/kainn9/tteokbokki/physics/entity_subset"
	transform_components "github.com/kainn9/tteokbokki/transform/components"
	"github.com/kainn9/tteokbokki/vector"
)

// Forces.
func AddForce(phys physics_components.PhysicsFace, force vector.Vec2Face) {
	phys.SetSumForces(
		phys.SumForces().Add(force),
	)
}

func ClearForces(phys physics_components.PhysicsFace) {
	emptyVec := vector.NewVec2(0, 0)
	phys.SetSumForces(emptyVec)
}

func AddTorque(torque float64, phys physics_components.PhysicsFace) {
	phys.SetSumTorque(
		phys.SumTorque() + torque,
	)
}

func ClearTorque(phys physics_components.PhysicsFace) {
	phys.SetSumTorque(0.0)
}

// Impulses.
func ApplyImpulse(phys physics_components.PhysicsFace, linearImpulse, collisionDisplacement vector.Vec2Face) {

	linearImpulseScaled := linearImpulse.Scale(phys.InverseMass())

	phys.SetVel(phys.Vel().Add(linearImpulseScaled))

	angularImpulseScaled := collisionDisplacement.CrossProduct(linearImpulse) * phys.InverseAngularMass()

	phys.SetAngularVel(
		phys.AngularVel() + angularImpulseScaled,
	)
}

// Integration.
func Integrate_Multi(
	transform transform_components.TransformFace,
	phys physics_components.PhysicsFace,
	shape transform_components.ShapeFace,
	dt float64,
) {
	if phys == nil {
		particle := entitysubset.NewParticle(
			transform,
			phys,
		)

		Integrate(particle, dt)
	} else {
		rigidBody := entitysubset.NewRigidBody(
			transform,
			shape,
			phys,
		)

		Integrate(rigidBody, dt)
	}
}

func Integrate(
	particleOrBody entitysubset.ParticleFace,
	dt float64,
) {
	integrateLinear(particleOrBody, dt)

	if body, ok := particleOrBody.(entitysubset.RigidBodyFace); ok {
		integrateAngular(body, dt)
		body.UpdateWorldVertices()
	}

}

func integrateLinear(
	particle entitysubset.ParticleFace,
	dt float64,
) {
	particle.SetAccel(
		particle.SumForces().Scale(particle.InverseMass()),
	)

	particle.SetVel(
		particle.Vel().Add(particle.Accel().Scale(dt)),
	)

	newPos := particle.Position().Add(particle.Vel().Scale(dt))

	particle.SetPosition(newPos)

	ClearForces(particle)
}

func integrateAngular(body entitysubset.RigidBodyFace, dt float64) {
	body.SetAngularAccel(
		body.SumTorque() * body.InverseAngularMass(),
	)

	body.SetAngularVel(
		body.AngularVel() + (body.AngularAccel() * dt),
	)

	body.SetRotation(
		body.Rotation() + (body.AngularVel() * dt),
	)

	ClearTorque(body)
}
