package tBokiComponents

import tBokiVec "github.com/kainn9/tteokbokki/math/vec"

// Collision data between two rigid bodies.
type Contact struct {
	Start, End, Normal tBokiVec.Vec2
	Depth              float64
}

func NewContact() Contact {
	return Contact{}
}
