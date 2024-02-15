package transform_components

import "github.com/kainn9/tteokbokki/vector"

type TransformFace interface {
	Position() vector.Vec2Face
	SetPosition(vector.Vec2Face)

	Rotation() float64
	SetRotation(rotation float64)

	Scale() vector.Vec2Face
	SetScale(x, y float64)
}

type Transform struct {
	position, scale vector.Vec2Face
	rotation        float64
}

func NewTransform(x, y, rotation float64) TransformFace {
	scale := vector.NewVec2(1, 1)
	return &Transform{
		vector.NewVec2(x, y),
		scale,
		rotation,
	}
}

func (trans Transform) Position() vector.Vec2Face {
	return trans.position
}

func (trans *Transform) SetPosition(pos vector.Vec2Face) {
	trans.position = pos
}

func (trans Transform) Rotation() float64 {
	return trans.rotation
}

func (trans *Transform) SetRotation(newRot float64) {
	trans.rotation = newRot
}

func (trans Transform) Scale() vector.Vec2Face {
	return trans.scale
}

func (trans *Transform) SetScale(x, y float64) {
	trans.scale.Set(x, y)
}
