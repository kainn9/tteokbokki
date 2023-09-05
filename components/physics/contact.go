package componentsPhysics

import "github.com/kainn9/tteokbokki/math/vec"

// Collision data between two rigid bodies.
// Can can be used with resolver to handle collision.
type Contact struct {
	A, B               *RigidBody
	Start, End, Normal vec.Vec2
	Depth              float64
}

func (PhysicsComponentsStruct) NewContact() Contact {
	return Contact{}
}
