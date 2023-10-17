package globals

import (
	"github.com/kainn9/tteokbokki/components"
	"github.com/yohamta/donburi"
)

const (
	GAME_HEIGHT = 720
	GAME_WIDTH  = 1080
)

var (
	RigidBodyComponent  = donburi.NewComponentType[components.RigidBody]()
	RigidBodyComponents = donburi.NewComponentType[[]*components.RigidBody]()
	JointConstraint     = donburi.NewComponentType[components.JointConstraint]()
	BoundsFriction      = 0.5
)
