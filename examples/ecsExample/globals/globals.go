package globals

import (
	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/yohamta/donburi"
)

const (
	GAME_HEIGHT = 720
	GAME_WIDTH  = 1080
)

var (
	RigidBodyComponent  = donburi.NewComponentType[componentsPhysicsTypes.RigidBody]()
	RigidBodyComponents = donburi.NewComponentType[[]*componentsPhysicsTypes.RigidBody]()
	JointConstraint     = donburi.NewComponentType[componentsPhysicsTypes.JointConstraint]()
	BoundsFriction      = 0.030
)
