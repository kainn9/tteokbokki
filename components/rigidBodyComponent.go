package components

import (
	"fmt"

	"github.com/kainn9/tteokbokki/math/vec"
)

// A rigid body component.
// Can be used by physics package to simulate 2D physics.
type RigidBodyComponent struct {
	Pos, Vel, Accel, SumForces        vec.Vec2
	InverseMass, Elasticity, Friction float64

	// Rigid Body will behave as if it has infinite mass when
	// resolving collisions, but will still be affected by
	// gravity and other linear forces/impulses. Does not apply to
	// angular forces/impulses. Set angular mass to 0 to disable
	// angular forces/impulses.
	Unstoppable bool

	*AngularDataComponent
	Circle  *CircleComponent
	Polygon *PolygonComponent
}

type RigidBodyShapeError struct {
	msg string
}

func (e RigidBodyShapeError) Error() string {
	return e.msg
}

// Returns a new RigidBody with a Circle shape.
func NewRigidBodyCircle(x, y, radius, mass float64, angular bool) *RigidBodyComponent {

	shape := NewCircleShape(radius)
	angularData := NewAngularData(0, 0, 0, 0, 0)

	body := &RigidBodyComponent{
		Pos: vec.Vec2{X: x, Y: y},

		Elasticity:           1.0,
		Friction:             0.5,
		AngularDataComponent: angularData,
		Circle:               shape,
	}

	body.SetMass(mass)

	if angular {
		body.SetAngularMass(shape.GetMomentOfInertiaWithoutMass() * mass)
	} else {
		body.SetAngularMass(0)
	}

	return body
}

// Returns a new RigidBody with a Box shape.
func NewRigidBodyBox(x, y, w, h, mass float64, angular bool) *RigidBodyComponent {

	shape := NewBoxShape(w, h)
	angularData := NewAngularData(0, 0, 0, 0, 0)

	body := &RigidBodyComponent{
		Pos:                  vec.Vec2{X: x, Y: y},
		Elasticity:           0.5,
		Friction:             0.030,
		AngularDataComponent: angularData,
		Polygon:              shape,
	}
	body.SetMass(mass)

	if angular {
		body.SetAngularMass(shape.BoxComponent.GetMomentOfInertiaWithoutMass() * mass)
	} else {
		body.SetAngularMass(0)
	}

	body.UpdateVertices()

	return body
}

// Returns a new RigidBody from an []vertices. Must be convex.
func NewRigidBodyPolygon(x, y, mass float64, vertices []vec.Vec2, angular bool) *RigidBodyComponent {

	shape := NewPolyShape(vertices)
	angularData := NewAngularData(0, 0, 0, 0, 0)

	body := &RigidBodyComponent{
		Pos:                  vec.Vec2{X: x, Y: y},
		Elasticity:           0.3,
		Friction:             0.4,
		AngularDataComponent: angularData,
		Polygon:              shape,
	}

	body.SetMass(mass)

	if angular {
		body.SetAngularMass(shape.GetMomentOfInertiaWithoutMass() * mass)
	} else {
		body.SetAngularMass(0)
	}

	body.UpdateVertices()

	return body
}

func (rb *RigidBodyComponent) Validate() error {
	// count how many shape components are assigned
	count := 0
	if rb.Circle != nil {
		count++
	}

	if rb.Polygon != nil {
		count++
	}

	if count > 1 {

		msg := fmt.Sprintf("RigidBodyEntity can only have one shape, has Circle(%v) Poly(%v).", rb.Circle, rb.Polygon)

		return RigidBodyShapeError{
			msg,
		}
	}

	return nil
}

func (rb *RigidBodyComponent) IsStatic() bool {
	return rb.InverseMass == 0
}

func (rb *RigidBodyComponent) IsLinearOnly() bool {
	return rb.InverseAngularMass == 0
}

func (rb *RigidBodyComponent) IsAngular() bool {
	return !rb.IsLinearOnly()
}

func (rb *RigidBodyComponent) SetMass(mass float64) {
	if mass == 0 {
		rb.InverseMass = 0
		return
	}

	rb.InverseMass = 1 / mass

}

func (rb *RigidBodyComponent) GetMass() float64 {
	if rb.IsStatic() {
		return 0
	}

	return 1 / rb.InverseMass
}

func (rb *RigidBodyComponent) SetAngularMass(angularMass float64) {
	if angularMass == 0 {
		rb.InverseAngularMass = 0
		return
	}

	rb.InverseAngularMass = 1 / angularMass

}

func (rb *RigidBodyComponent) GetAngularMass() float64 {
	if rb.InverseAngularMass == 0 {
		return 0
	}

	return 1 / rb.InverseAngularMass
}

// Update the vertices of a RigidBody's local space to world space.
func (rb *RigidBodyComponent) UpdateVertices() error {
	if rb.Polygon == nil {
		return RigidBodyShapeError{
			"Cannot call UpdateVertices() on rb with nil polygon.",
		}
	}

	for i := 0; i <= len(rb.Polygon.LocalVertices)-1; i++ {

		// rotate
		lVert := (rb.Polygon.LocalVertices)[i]
		(rb.Polygon.WorldVertices)[i] = lVert.Rotate(rb.Rotation)

		// translate
		(rb.Polygon.WorldVertices)[i] = (rb.Polygon.WorldVertices)[i].Add(rb.Pos)

	}

	return nil
}

// returns a point from local space to world space, relative
// to a RigidBody's position and rotation.
func (rb *RigidBodyComponent) LocalToWorldSpace(point vec.Vec2) vec.Vec2 {
	return point.Rotate(rb.Rotation).Add(rb.Pos)
}

// returns a point from world space to local space, relative
// to a RigidBody's position and rotation.
func (rb *RigidBodyComponent) WorldToLocalSpace(point vec.Vec2) vec.Vec2 {
	return point.Sub(rb.Pos).Rotate(-rb.Rotation)
}
