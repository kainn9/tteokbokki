package factory

import (
	entitysubset "github.com/kainn9/tteokbokki/physics/entity_subset"
	"github.com/kainn9/tteokbokki/vector"

	physics_components "github.com/kainn9/tteokbokki/physics/components"
	transform_components "github.com/kainn9/tteokbokki/transform/components"
)

type forcesFactory struct {
	PIXELS_PER_METER, DEFAULT_GRAVITY float64
}

var Forces = &forcesFactory{
	PIXELS_PER_METER: 50.0,
	DEFAULT_GRAVITY:  9.8,
}

func (ff forcesFactory) NewWeightForce(
	phys physics_components.PhysicsFace,
	gravity float64,
) vector.Vec2Face {
	if phys.InverseMass() == 0 {
		return vector.NewVec2(0, 0)
	}

	mass := 1 / phys.InverseMass()

	weightForce := vector.NewVec2(
		0.0,
		// Multiply by mass since gravity should not be effected by mass &
		// forces scale based on inverseMass of whatever they are acting upon.
		mass*gravity*ff.PIXELS_PER_METER,
	)

	return weightForce
}

func (forcesFactory) NewDragForce(
	phys physics_components.PhysicsFace,
	dragCoefficient float64,
) vector.Vec2Face {
	velocity := phys.Vel()

	velMagSquared := velocity.MagSquared()

	if velMagSquared <= 0 {
		return vector.NewVec2(0, 0)
	}

	// Calculate the drag direction(opposite of velocity vector).
	direction := velocity.Norm().Scale(-1)

	// Calculate the drag magnitude — dragCoefficient * |v|^2
	dragMag := dragCoefficient * velMagSquared

	// Calculate the drag force vector
	dragForce := direction.Scale(dragMag)

	return dragForce
}

func (forcesFactory) NewFrictionForce(
	phys physics_components.PhysicsFace,
	frictionCoefficient float64,
) vector.Vec2Face {
	// Calculate the friction force direction (opposite of velocity vector).
	frictionDirection := phys.Vel().Norm().Scale(-1)

	// Multiply the normalized direction vector by the friction coefficient.
	frictionForce := frictionDirection.Scale(frictionCoefficient)

	return frictionForce
}

func (ff forcesFactory) NewAttractionForceSoft_Multi(
	transA, transB transform_components.TransformFace,
	physA, physB physics_components.PhysicsFace,
	attractionStrength, minDist, maxDist float64,
) vector.Vec2Face {
	particleA := entitysubset.NewParticle(transA, physA)
	particleB := entitysubset.NewParticle(transB, physB)

	return ff.NewAttractionForceSoft(
		particleA,
		particleB,
		attractionStrength,
		minDist,
		maxDist,
	)
}

func (forcesFactory) NewAttractionForceSoft(
	a, b entitysubset.ParticleFace,
	attractionStrength, minDist, maxDist float64,
) vector.Vec2Face {
	distance := b.Position().Sub(a.Position())

	distSq := distance.MagSquared()
	distSq = clamp(distSq, minDist*maxDist, maxDist*maxDist)

	direction := distance.Norm()

	return attractionForce(a, b, direction, attractionStrength, distSq)
}

func (forcesFactory) NewAttractionForceHard(
	a, b entitysubset.ParticleFace,
	attractionStrength, minDist, maxDist float64,
) vector.Vec2Face {

	distance := b.Position().Sub(a.Position())
	distSq := distance.MagSquared()

	if distance.MagSquared() < minDist*minDist || distance.MagSquared() > maxDist*maxDist {
		return vector.NewVec2(0, 0)
	}

	direction := distance.Norm()

	return attractionForce(a, b, direction, attractionStrength, distSq)
}

func attractionForce(
	a, b entitysubset.ParticleFace,
	direction vector.Vec2Face,
	attractionStrength, distSq float64,
) vector.Vec2Face {
	// Inverse-Square Law: In many physical scenarios, including gravity and electrostatics,
	// forces between two objects decrease with the square of the distance between them.
	massA := 1 / a.InverseMass()
	massB := 1 / b.InverseMass()
	magnitude := attractionStrength * (massA * massB) / distSq

	return direction.Scale(magnitude)
}

// Creates a new spring force to apply spring force between two RigidBodies.
func (forcesFactory) NewSpringForce(
	transform, anchorTransform transform_components.TransformFace,
	restLen, stiffness float64,
) vector.Vec2Face {
	distance := transform.Position().Sub(anchorTransform.Position())

	displacement := distance.Mag() - restLen

	direction := distance.Norm()

	magnitude := -stiffness * displacement

	sForce := direction.Scale(magnitude)

	return sForce
}

func clamp(v, min, max float64) float64 {
	if min > max {
		return v
	}

	if min == max {
		return min
	}

	if v > max {
		return max
	}

	if v < min {
		return min
	}

	return v
}
