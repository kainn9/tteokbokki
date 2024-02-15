package physics

import physics_components "github.com/kainn9/tteokbokki/physics/components"

type util struct{}

var Util = &util{}

func (*util) IsStaticLinear(phys physics_components.PhysicsFace) bool {
	return phys.InverseMass() == 0
}

func (*util) IsStaticAngular(phys physics_components.PhysicsFace) bool {
	return phys.InverseAngularMass() == 0
}
