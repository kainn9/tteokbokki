package entitysubset

import (
	physics_components "github.com/kainn9/tteokbokki/physics/components"
	transform_components "github.com/kainn9/tteokbokki/transform/components"
	"github.com/kainn9/tteokbokki/vector"
)

type RigidBodyFace interface {
	ParticleFace
	transform_components.ShapeFace

	SetPosition(vector.Vec2Face)
	SetRotation(float64)
	SetScale(x, y float64)

	Transform(pos, scale vector.Vec2Face, rotation float64)

	UpdateWorldVertices()
}

type RigidBody struct {
	transform_components.TransformFace
	transform_components.ShapeFace
	physics_components.PhysicsFace
}

func NewRigidBody(
	trans transform_components.TransformFace,
	shape transform_components.ShapeFace,
	phys physics_components.PhysicsFace,
) RigidBodyFace {
	return &RigidBody{
		trans,
		shape,
		phys,
	}
}

func (rb RigidBody) UpdateWorldVertices() {
	rb.Polygon().UpdateWorldVertices(rb.TransformFace)
}

func (rb RigidBody) SetPosition(pos vector.Vec2Face) {
	rb.TransformFace.SetPosition(pos)
	rb.UpdateWorldVertices()
}

func (rb RigidBody) SetRotation(rotation float64) {
	rb.TransformFace.SetRotation(rotation)
	rb.UpdateWorldVertices()
}

func (rb RigidBody) SetScale(x, y float64) {
	rb.TransformFace.SetScale(x, y)
	rb.UpdateWorldVertices()
}
func (rb RigidBody) Transform(pos, scale vector.Vec2Face, rotation float64) {
	rb.TransformFace.SetPosition(pos)
	rb.TransformFace.SetRotation(rotation)
	rb.TransformFace.SetScale(scale.X(), scale.Y())
	rb.UpdateWorldVertices()
}
