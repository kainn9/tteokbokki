package force

import (
	"errors"
	"fmt"

	"github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/constants"
	"github.com/kainn9/tteokbokki/math"
	"github.com/kainn9/tteokbokki/math/vec"
)

type ForcesFactory struct{}

// Creates weight force to apply gravity.
// @params:
// 0 - mass(required)
// 1 - gravity(optional — will use default gravity constant if not provided)
func (ForcesFactory) NewWeightForce(params ...float64) (vec.Vec2, error) {
	length := len(params)

	tooManyArgs := length > 2
	if tooManyArgs {
		errorString := fmt.Sprintf("too many arguments for forces.NewWeightForce(). Expected two, received %v.\n", length)
		return vec.Vec2{}, errors.New(errorString)
	}

	mass := params[0]
	gravity := constants.GRAVITY

	usingCustomGravityFloat := length == 2
	if usingCustomGravityFloat {
		gravity = params[1]
	}

	// Multiply by mass since gravity should not be effected by mass.
	// Forces use the inverseMass of RigidBodies so this will cancel out.
	weightForce := vec.Vec2{X: 0.0, Y: mass * gravity * constants.PIXELS_PER_METER}
	return weightForce, nil
}

// Creates a new drag force to apply drag.
func (ForcesFactory) NewDragForce(velocity vec.Vec2, dragCoefficient float64) vec.Vec2 {
	dragForce := vec.Vec2{}

	velMagSquared := velocity.MagSquared()

	if velMagSquared <= 0 {
		return dragForce
	}

	// Calculate the drag direction (opposite of velocity vector)
	direction := velocity.Norm().Scale(-1)

	// Calculate the drag magnitude — dragCoefficient * |v|^2
	dragMag := dragCoefficient * velMagSquared

	// Calculate the drag force vector
	dragForce = direction.Scale(dragMag)

	return dragForce
}

// Creates a new friction force to apply friction.
func (ForcesFactory) NewFrictionForce(velocity vec.Vec2, frictionCoefficient float64) vec.Vec2 {
	frictionForce := vec.Vec2{}

	// Calculate the friction force direction (opposite of velocity vector)
	frictionDirection := velocity.Norm().Scale(-1)

	// Multiply the normalized direction vector by the friction coefficient
	frictionForce = frictionDirection.Scale(frictionCoefficient)

	return frictionForce
}

// Creates a new attraction force to apply attraction between two RigidBodies.
// Will weaken force based on minDist and maxDist.
func (ForcesFactory) NewAttractionForceSoft(a, b components.RigidBody, aFactor, minDist, maxDist float64) vec.Vec2 {

	distance := b.Pos.Sub(a.Pos)

	distSq := distance.MagSquared()
	distSq = math.Clamp(distSq, minDist*maxDist, maxDist*maxDist)

	direction := distance.Norm()

	// Inverse-Square Law: In many physical scenarios, including gravity and electrostatics,
	// forces between two objects decrease with the square of the distance between them.
	magnitude := aFactor * (a.GetMass() * b.GetMass()) / distSq

	aForce := direction.Scale(magnitude)

	return aForce
}

// Creates a new attraction force to apply attraction between two RigidBodies.
// Will disable force based on minDist and maxDist.
func (ForcesFactory) NewAttractionForceHard(a, b components.RigidBody, aFactor, minDist, maxDist float64) vec.Vec2 {

	distance := b.Pos.Sub(a.Pos)
	distSq := distance.MagSquared()

	if distance.MagSquared() < minDist*minDist || distance.MagSquared() > maxDist*maxDist {
		return vec.Vec2{}
	}

	direction := distance.Norm()

	// Inverse-Square Law: In many physical scenarios, including gravity and electrostatics,
	// forces between two objects decrease with the square of the distance between them.
	magnitude := aFactor * (a.GetMass() * b.GetMass()) / distSq

	aForce := direction.Scale(magnitude)

	return aForce
}

// Creates a new spring force to apply spring force between two RigidBodies.
func (ForcesFactory) NewSpringForce(rigidBody, anchorBody components.RigidBody, restLen, k float64) vec.Vec2 {

	distance := rigidBody.Pos.Sub(anchorBody.Pos)

	displacement := distance.Mag() - restLen

	direction := distance.Norm()

	magnitude := -k * displacement

	sForce := direction.Scale(magnitude)

	return sForce
}
