package components

import "github.com/kainn9/tteokbokki/math/vec"

// Collision data between two rigid bodies.
type Contact struct {
	Start, End, Normal vec.Vec2
	Depth              float64
}

func NewContact() Contact {
	return Contact{}
}
