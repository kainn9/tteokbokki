package resolver

import (
	"math"

	physics_components "github.com/kainn9/tteokbokki/physics/components"
)

type util struct{}

var Util = &util{}

func (util) Snap(c *physics_components.Collision) {
	if math.Abs(c.Normal.X()) < math.Abs(c.Normal.Y()) {
		c.Normal.Set(0, c.Normal.Y())
	} else {
		c.Normal.Set(c.Normal.X(), 0)
	}

}
