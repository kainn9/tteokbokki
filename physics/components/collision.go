package physics_components

import (
	"math"

	"github.com/kainn9/tteokbokki/vector"
)

type Collision struct {
	Start, End, Normal vector.Vec2Face
	Depth              float64
}

func NewCollision(start, end, normal vector.Vec2Face, depth float64) *Collision {
	return &Collision{start, end, normal, depth}
}

func (c *Collision) Snap() *Collision {
	if math.Abs(c.Normal.X()) < math.Abs(c.Normal.Y()) {
		c.Normal.Set(0, c.Normal.Y())
	} else {
		c.Normal.Set(c.Normal.X(), 0)
	}

	return c
}
