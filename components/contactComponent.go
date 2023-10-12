package components

import "github.com/kainn9/tteokbokki/math/vec"

// Collision data between two rigid bodies.
// Can can be used with resolver to handle collision.
type ContactComponent struct {
	A, B               *RigidBodyComponent
	Start, End, Normal vec.Vec2
	Depth              float64
}

func NewContact() ContactComponent {
	return ContactComponent{}
}
