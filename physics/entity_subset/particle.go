package entitysubset

import (
	physics_components "github.com/kainn9/tteokbokki/physics/components"
	transform_components "github.com/kainn9/tteokbokki/transform/components"
)

type ParticleFace interface {
	transform_components.TransformFace
	physics_components.PhysicsFace
}

type Particle struct {
	transform_components.TransformFace
	physics_components.PhysicsFace
}

func NewParticle(
	trans transform_components.TransformFace,
	phys physics_components.PhysicsFace,
) ParticleFace {
	return &Particle{
		trans,
		phys,
	}
}
