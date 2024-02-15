package physics_components

import (
	"log"
	"math"

	transform_components "github.com/kainn9/tteokbokki/transform/components"
	"github.com/kainn9/tteokbokki/vector"
)

type PhysicsFace interface {
	UnstoppableLinear() bool
	SetUnstoppableLinear(bool)

	UnstoppableAngular() bool
	SetUnstoppableAngular(bool)

	Accel() vector.Vec2Face
	SetAccel(vector.Vec2Face)

	Vel() vector.Vec2Face
	SetVel(vector.Vec2Face)

	SumForces() vector.Vec2Face
	SetSumForces(vector.Vec2Face)

	InverseMass() float64
	SetMass(float64)

	AngularVel() float64
	SetAngularVel(float64)

	AngularAccel() float64
	SetAngularAccel(float64)

	SumTorque() float64
	SetSumTorque(float64)

	InverseAngularMass() float64
	SetAngularMass(float64)

	Friction() float64
	SetFriction(float64)

	Elasticity() float64
	SetElasticity(float64)

	SetAndCalculateAngularMass(transform_components.ShapeFace)
}

type Physics struct {
	unstoppableLinear, unstoppableAngular bool

	accel, vel, sumForces vector.Vec2Face

	inverseMass float64

	angularVel, angularAccel, sumTorque float64
	inverseAngularMass                  float64

	friction, elasticity float64
}

func NewPhysics(mass float64) PhysicsFace {

	inverseMass := mass
	if mass != 0 {
		inverseMass = 1 / mass
	}

	return &Physics{
		accel:       &vector.Vec2{},
		vel:         &vector.Vec2{},
		sumForces:   &vector.Vec2{},
		inverseMass: inverseMass,
	}
}

func (physics Physics) UnstoppableLinear() bool {
	return physics.unstoppableLinear
}

func (physics *Physics) SetUnstoppableLinear(unstoppable bool) {
	physics.unstoppableLinear = unstoppable
}

func (physics Physics) UnstoppableAngular() bool {
	return physics.unstoppableAngular
}

func (physics *Physics) SetUnstoppableAngular(unstoppable bool) {
	physics.unstoppableAngular = unstoppable
}

func (physics Physics) IsStatic() bool {
	return physics.inverseMass == 0
}

func (physics Physics) Accel() vector.Vec2Face {
	return physics.accel
}

func (physics *Physics) SetAccel(accel vector.Vec2Face) {
	physics.accel = accel
}

func (physics Physics) Vel() vector.Vec2Face {
	return physics.vel
}

func (physics *Physics) SetVel(vel vector.Vec2Face) {
	physics.vel = vel
}

func (physics *Physics) SetSumForces(sumForces vector.Vec2Face) {
	physics.sumForces = sumForces
}

func (physics Physics) SumForces() vector.Vec2Face {
	return physics.sumForces
}

func (physics Physics) InverseMass() float64 {
	return physics.inverseMass
}
func (physics *Physics) SetMass(mass float64) {
	if mass == 0 {
		physics.inverseMass = mass
	} else {
		physics.inverseMass = 1 / mass
	}
}

func (physics Physics) AngularVel() float64 {
	return physics.angularVel
}

func (physics *Physics) SetAngularVel(angularVel float64) {
	physics.angularVel = angularVel
}

func (physics Physics) AngularAccel() float64 {
	return physics.angularAccel
}

func (physics *Physics) SetAngularAccel(angularAccel float64) {
	physics.angularAccel = angularAccel
}

func (physics Physics) SumTorque() float64 {
	return physics.sumTorque
}

func (physics *Physics) SetSumTorque(sumTorque float64) {
	physics.sumTorque = sumTorque
}

func (physics Physics) InverseAngularMass() float64 {
	return physics.inverseAngularMass
}

func (physics *Physics) SetAngularMass(angularMass float64) {
	if angularMass == 0 {
		physics.inverseAngularMass = angularMass
	} else {
		physics.inverseAngularMass = 1 / angularMass
	}
}

func (physics Physics) Friction() float64 {
	return physics.friction
}

func (physics *Physics) SetFriction(f float64) {
	physics.friction = f
}

func (physics Physics) Elasticity() float64 {
	return physics.elasticity
}

func (physics *Physics) SetElasticity(e float64) {
	physics.elasticity = e
}

func (physics *Physics) SetAndCalculateAngularMass(s transform_components.ShapeFace) {
	if s.Circle() != nil {
		angularMass := getMomentOfInertiaWithoutMassCircle(s.Circle())
		physics.SetAngularMass(angularMass)
		return
	}

	if s.Polygon().AAB() != nil {
		angularMass := getMomentOfInertiaWithoutMassRect(s.Polygon().AAB())
		physics.SetAngularMass(angularMass)
		return
	}

	if s.Polygon() != nil {
		angularMass := getMomentOfInertiaWithoutMassPolygon(s.Polygon())
		physics.SetAngularMass(angularMass)
		return
	}

	log.Println("shape is malformed, cannot calculate angular mass")
}

func getMomentOfInertiaWithoutMassCircle(circle transform_components.CircleFace) float64 {
	return (0.5 * (circle.Radius() * circle.Radius()))
}

func getMomentOfInertiaWithoutMassRect(rect transform_components.AABFace) float64 {
	return 0.083333 * ((rect.Width() * rect.Width()) + (rect.Height() * rect.Height()))
}

func getMomentOfInertiaWithoutMassPolygon(p transform_components.PolygonFace) float64 {
	// Calculate the centroid of the polygon
	centroid := p.Centroid()

	// Get local vertices of the polygon
	localVertices := p.LocalVertices()

	// Initialize variables for accumulating moments
	var acc0, acc1 float64

	// Iterate over each pair of consecutive vertices
	for i := 0; i < len(localVertices); i++ {
		// Calculate vectors representing edges of the polygon
		a := localVertices[i].Sub(centroid)
		b := localVertices[(i+1)%len(localVertices)].Sub(centroid)

		// Calculate the cross product of vectors a and b
		cross := math.Abs(a.CrossProduct(b))

		// Update acc0 and acc1 using the cross product and dot products of vectors a and b
		acc0 += cross * (a.ScalarProduct(a) + b.ScalarProduct(b) + a.ScalarProduct(b))
		acc1 += cross
	}

	// Calculate the moment of inertia using the accumulated values
	return acc0 / 6 / acc1
}
