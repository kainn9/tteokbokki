package components

import (
	"github.com/kainn9/tteokbokki/math/vec"
)

// Various shapes that can be assumed by a rigid body.

// A circle shape. Has a radius. Has no vertices.
type CircleComponent struct {
	Radius float64
}

// A polygon shape. Has vertices. Has no radius or width/height(directly).
// Optionally may embed a box struct. If the box shape is embedded,
// the body should be treated as a box, vertices should be dictated
// by the box's width/height/position.
type PolygonComponent struct {
	LocalVertices []vec.Vec2
	WorldVertices []vec.Vec2
	*BoxComponent
}

// A box shape. Has no radius. Has a width/height and vertices.
// If embedded in a polygon, the polygon should be treated as a box.
type BoxComponent struct {
	Width, Height float64
}

// A struct that holds angular data for a rigid body.
// If the inverse angular mass is 0, the body is not affected by angular forces.
// (does not rotate)
type AngularDataComponent struct {
	Rotation, AngularVel, AngularAccel, SumTorque, InverseAngularMass float64
}

func NewAngularData(rotation, angularVel, angularAccel, sumTorque, angularMass float64) *AngularDataComponent {
	return &AngularDataComponent{
		Rotation:           rotation,
		AngularVel:         angularAccel,
		AngularAccel:       angularAccel,
		SumTorque:          sumTorque,
		InverseAngularMass: angularMass,
	}
}

func (c CircleComponent) GetMomentOfInertiaWithoutMass() float64 {
	return (0.5 * (c.Radius * c.Radius))
}

func (b BoxComponent) GetMomentOfInertiaWithoutMass() float64 {

	return 0.083333 * ((b.Width * b.Width) + (b.Height * b.Height))
}

// Default moment of inertia for a polygon will be 6000.
func (p PolygonComponent) GetMomentOfInertiaWithoutMass() float64 {
	return 6000
}

func NewCircleShape(radius float64) *CircleComponent {
	return &CircleComponent{
		Radius: radius,
	}
}

func NewBoxShape(width, height float64) *PolygonComponent {

	p := &PolygonComponent{}

	b := &BoxComponent{
		Width:  width,
		Height: height,
	}

	// local
	p.LocalVertices = append(p.LocalVertices, vec.Vec2{X: -width / 2.0, Y: -height / 2.0}) // top left
	p.LocalVertices = append(p.LocalVertices, vec.Vec2{X: width / 2.0, Y: -height / 2.0})  // top right
	p.LocalVertices = append(p.LocalVertices, vec.Vec2{X: width / 2.0, Y: height / 2.0})   // bottom right
	p.LocalVertices = append(p.LocalVertices, vec.Vec2{X: -width / 2.0, Y: height / 2.0})  // bottom left

	// world
	p.WorldVertices = append(p.WorldVertices, vec.Vec2{X: -width / 2.0, Y: -height / 2.0}) // top left
	p.WorldVertices = append(p.WorldVertices, vec.Vec2{X: width / 2.0, Y: -height / 2.0})  // top right
	p.WorldVertices = append(p.WorldVertices, vec.Vec2{X: width / 2.0, Y: height / 2.0})   // bottom right
	p.WorldVertices = append(p.WorldVertices, vec.Vec2{X: -width / 2.0, Y: height / 2.0})  // bottom left

	p.BoxComponent = b

	return p
}

func NewPolyShape(vertices []vec.Vec2) *PolygonComponent {

	localVertices := make([]vec.Vec2, len(vertices))
	worldVertices := make([]vec.Vec2, len(vertices))

	copy(localVertices, vertices)
	copy(worldVertices, vertices)

	return &PolygonComponent{
		LocalVertices: localVertices,
		WorldVertices: worldVertices,
	}
}