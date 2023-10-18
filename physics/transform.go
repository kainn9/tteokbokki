package tBokiPhysics

import (
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

type transformer struct{}

var Transformer transformer

// Apply a force to a RigidBody.
func (transformer) AddForce(rb *tBokiComponents.RigidBody, f tBokiVec.Vec2) {
	rb.SumForces = rb.SumForces.Add(f)
}

// Clear all forces applied to a RigidBody.
func (transformer) ClearForces(rb *tBokiComponents.RigidBody) {
	rb.SumForces = tBokiVec.Vec2{}
}

// Apply a rotation force to a RigidBody.
func (transformer) AddTorque(t float64, rb *tBokiComponents.RigidBody) {
	rb.SumTorque += t
}

// Clear all rotation forces applied to a RigidBody.
func (transformer) ClearTorque(rb *tBokiComponents.RigidBody) {
	rb.SumTorque = 0.0
}

// Integrate a RigidBody. Euler integration is used.
func (t transformer) Integrate(rb *tBokiComponents.RigidBody, dt float64) {
	if rb.IsStatic() {
		return
	}

	t.integrateLinear(rb, dt)
	t.integrateAngular(rb, dt)
}

// Apply a immediate change in velocity to a RigidBody.
func (transformer) ApplyImpulseLinear(rb *tBokiComponents.RigidBody, linearImpulseFactor tBokiVec.Vec2) {
	if rb.IsStatic() {
		return
	}

	impulse := linearImpulseFactor.Scale(rb.InverseMass)
	rb.Vel = rb.Vel.Add(impulse)

}

// Apply a immediate change in angularVelocity to a RigidBody.
// Will not work on linear only bodies.
func (transformer) ApplyImpulseAngular(rb *tBokiComponents.RigidBody, angularImpulse float64) {

	if rb.IsStatic() || rb.IsLinearOnly() {
		return
	}

	rb.AngularVel += angularImpulse * rb.InverseAngularMass
}

// Apply a immediate change in velocity && angularVelocity to a RigidBody.
// Will work on both linear and angular bodies.
func (transformer) ApplyImpulse(rb *tBokiComponents.RigidBody, linearImpulseFactor, angularImpulseFactor tBokiVec.Vec2) {

	if rb.IsStatic() {
		return
	}

	impulseLinear := linearImpulseFactor.Scale(rb.InverseMass)
	rb.Vel = rb.Vel.Add(impulseLinear)

	impulseAngular := angularImpulseFactor.CrossProduct(linearImpulseFactor) * rb.InverseAngularMass
	rb.AngularVel += impulseAngular

}

func (t transformer) integrateLinear(rb *tBokiComponents.RigidBody, dt float64) {

	rb.Accel = rb.SumForces.Scale(rb.InverseMass)

	rb.Vel = rb.Vel.Add(rb.Accel.Scale(dt))

	rb.Pos = rb.Pos.Add(rb.Vel.Scale(dt))

	t.ClearForces(rb)
}

func (t transformer) integrateAngular(rb *tBokiComponents.RigidBody, dt float64) {

	rb.AngularAccel = rb.SumTorque * rb.InverseAngularMass

	rb.AngularVel += (rb.AngularAccel * dt)

	rb.Rotation += (rb.AngularVel * dt)

	t.ClearTorque(rb)
}
