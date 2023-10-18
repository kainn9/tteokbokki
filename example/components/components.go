package components

import (
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	"github.com/yohamta/donburi"
)

var (
	RigidBodyComponent       = donburi.NewComponentType[tBokiComponents.RigidBody]()
	RigidBodyComponents      = donburi.NewComponentType[[]*tBokiComponents.RigidBody]()
	JointConstraintComponent = donburi.NewComponentType[tBokiComponents.JointConstraint]()
)
