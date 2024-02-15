package factory

import (
	"math"

	physics_components "github.com/kainn9/tteokbokki/physics/components"
	transform_components "github.com/kainn9/tteokbokki/transform/components"
	"github.com/kainn9/tteokbokki/vector"
)

type componentFactory struct{}

var Components = &componentFactory{}

func (componentFactory) NewParticleComponents(x, y, mass float64) (
	transform_components.TransformFace,
	physics_components.PhysicsFace,
) {
	rotation := 0.0
	transform := transform_components.NewTransform(x, y, rotation)
	phys := physics_components.NewPhysics(mass)

	return transform, phys
}

func (cf componentFactory) NewRigidBodyRectangleComponents(x, y, width, height, mass, rotation float64) (
	transform_components.TransformFace,
	transform_components.ShapeFace,
	physics_components.PhysicsFace,
) {
	trans, phys := cf.NewParticleComponents(x, y, mass)
	trans.SetRotation(rotation)

	shape := transform_components.NewPolygonRectangleShape(width, height)
	shape.Polygon().UpdateWorldVertices(trans)

	return trans, shape, phys
}

func (cf componentFactory) NewRigidBodyHexagonComponents(x, y, mass, rotation, size float64) (
	transform_components.TransformFace,
	transform_components.ShapeFace,
	physics_components.PhysicsFace,
) {
	trans, phys := cf.NewParticleComponents(x, y, mass)
	trans.SetRotation(rotation)

	// Calculate the distance from the center to each vertex
	radius := size / math.Sqrt(3)

	// Define the vertices of the hexagon relative to its center (0,0)
	hexagonShape := make([]vector.Vec2Face, 6)
	for i := 0; i < 6; i++ {
		angle := math.Pi / 3.0 * float64(i) // 60 degrees
		vx := radius * math.Cos(angle)
		vy := radius * math.Sin(angle)
		hexagonShape[i] = vector.NewVec2(vx, vy)
	}

	// Create the polygon shape using the vertices
	shape := transform_components.NewPolygonShape(hexagonShape)
	shape.Polygon().UpdateWorldVertices(trans)

	return trans, shape, phys
}
