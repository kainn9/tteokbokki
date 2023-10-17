package transform

import (
	"github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/math/vec"
)

// Apply a force to a RigidBody.
func (Transformer) AddForce(rb *components.RigidBody, f vec.Vec2) {
	rb.SumForces = rb.SumForces.Add(f)
}

// Clear all forces applied to a RigidBody.
func (Transformer) ClearForces(rb *components.RigidBody) {
	rb.SumForces = vec.Vec2{}
}

// Apply a rotation force to a RigidBody.
func (Transformer) AddTorque(t float64, rb *components.RigidBody) {
	rb.SumTorque += t
}

// Clear all rotation forces applied to a RigidBody.
func (Transformer) ClearTorque(rb *components.RigidBody) {
	rb.SumTorque = 0.0
}

// Integrate a RigidBody. Euler integration is used.
func (t Transformer) Integrate(rb *components.RigidBody, dt float64) {
	if rb.IsStatic() {
		return
	}

	t.integrateLinear(rb, dt)
	t.integrateAngular(rb, dt)
}

// Apply a immediate change in velocity to a RigidBody.
func (Transformer) ApplyImpulseLinear(rb *components.RigidBody, linearImpulseFactor vec.Vec2) {
	if rb.IsStatic() {
		return
	}

	impulse := linearImpulseFactor.Scale(rb.InverseMass)
	rb.Vel = rb.Vel.Add(impulse)

}

// Apply a immediate change in angularVelocity to a RigidBody.
// Will not work on linear only bodies.
func (t Transformer) ApplyImpulseAngular(rb *components.RigidBody, angularImpulse float64) {

	if rb.IsStatic() || rb.IsLinearOnly() {
		return
	}

	rb.AngularVel += angularImpulse * rb.InverseAngularMass
}

// Apply a immediate change in velocity && angularVelocity to a RigidBody.
// Will work on both linear and angular bodies.
func (t Transformer) ApplyImpulse(rb *components.RigidBody, linearImpulseFactor, angularImpulseFactor vec.Vec2) {

	if rb.IsStatic() {
		return
	}

	impulseLinear := linearImpulseFactor.Scale(rb.InverseMass)
	rb.Vel = rb.Vel.Add(impulseLinear)

	impulseAngular := angularImpulseFactor.CrossProduct(linearImpulseFactor) * rb.InverseAngularMass
	rb.AngularVel += impulseAngular

}

func (s Transformer) integrateLinear(rb *components.RigidBody, dt float64) {

	rb.Accel = rb.SumForces.Scale(rb.InverseMass)

	rb.Vel = rb.Vel.Add(rb.Accel.Scale(dt))

	rb.Pos = rb.Pos.Add(rb.Vel.Scale(dt))

	s.ClearForces(rb)
}

func (t Transformer) integrateAngular(rb *components.RigidBody, dt float64) {

	rb.AngularAccel = rb.SumTorque * rb.InverseAngularMass

	rb.AngularVel += (rb.AngularAccel * dt)

	rb.Rotation += (rb.AngularVel * dt)

	t.ClearTorque(rb)
}
