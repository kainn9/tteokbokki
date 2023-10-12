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
	RigidBodyComponent  = donburi.NewComponentType[components.RigidBodyComponent]()
	RigidBodyComponents = donburi.NewComponentType[[]*components.RigidBodyComponent]()
	JointConstraint     = donburi.NewComponentType[components.JointConstraintComponent]()
	BoundsFriction      = 0.5
)
