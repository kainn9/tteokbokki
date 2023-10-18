package tBokiPhysics

import (
	"errors"
	"fmt"

	tBokiComponents "github.com/kainn9/tteokbokki/components"
	tBokiConstants "github.com/kainn9/tteokbokki/constants"
	tBokiMath "github.com/kainn9/tteokbokki/math"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

type forceFactory struct{}

var ForceFactory forceFactory

// Creates weight force to apply gravity.
// @params:
// 0 - mass(required)
// 1 - gravity(optional — will use default gravity constant if not provided)
func (forceFactory) NewWeightForce(params ...float64) (tBokiVec.Vec2, error) {
	length := len(params)

	tooManyArgs := length > 2
	if tooManyArgs {
		errorString := fmt.Sprintf("too many arguments for forces.NewWeightForce(). Expected two, received %v.\n", length)
		return tBokiVec.Vec2{}, errors.New(errorString)
	}

	mass := params[0]
	gravity := tBokiConstants.GRAVITY

	usingCustomGravityFloat := length == 2
	if usingCustomGravityFloat {
		gravity = params[1]
	}

	// Multiply by mass since gravity should not be effected by mass.
	// Forces use the inverseMass of RigidBodies so this will cancel out.
	weightForce := tBokiVec.Vec2{X: 0.0, Y: mass * gravity * tBokiConstants.PIXELS_PER_METER}
	return weightForce, nil
}

// Creates a new drag force to apply drag.
func (forceFactory) NewDragForce(velocity tBokiVec.Vec2, dragCoefficient float64) tBokiVec.Vec2 {
	dragForce := tBokiVec.Vec2{}

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
func (forceFactory) NewFrictionForce(velocity tBokiVec.Vec2, frictionCoefficient float64) tBokiVec.Vec2 {
	frictionForce := tBokiVec.Vec2{}

	// Calculate the friction force direction (opposite of velocity vector)
	frictionDirection := velocity.Norm().Scale(-1)

	// Multiply the normalized direction vector by the friction coefficient
	frictionForce = frictionDirection.Scale(frictionCoefficient)

	return frictionForce
}

// Creates a new attraction force to apply attraction between two RigidBodies.
// Will weaken force based on minDist and maxDist.
func (forceFactory) NewAttractionForceSoft(a, b tBokiComponents.RigidBody, aFactor, minDist, maxDist float64) tBokiVec.Vec2 {

	distance := b.Pos.Sub(a.Pos)

	distSq := distance.MagSquared()
	distSq = tBokiMath.Clamp(distSq, minDist*maxDist, maxDist*maxDist)

	direction := distance.Norm()

	// Inverse-Square Law: In many physical scenarios, including gravity and electrostatics,
	// forces between two objects decrease with the square of the distance between them.
	magnitude := aFactor * (a.GetMass() * b.GetMass()) / distSq

	aForce := direction.Scale(magnitude)

	return aForce
}

// Creates a new attraction force to apply attraction between two RigidBodies.
// Will disable force based on minDist and maxDist.
func (forceFactory) NewAttractionForceHard(a, b tBokiComponents.RigidBody, aFactor, minDist, maxDist float64) tBokiVec.Vec2 {

	distance := b.Pos.Sub(a.Pos)
	distSq := distance.MagSquared()

	if distance.MagSquared() < minDist*minDist || distance.MagSquared() > maxDist*maxDist {
		return tBokiVec.Vec2{}
	}

	direction := distance.Norm()

	// Inverse-Square Law: In many physical scenarios, including gravity and electrostatics,
	// forces between two objects decrease with the square of the distance between them.
	magnitude := aFactor * (a.GetMass() * b.GetMass()) / distSq

	aForce := direction.Scale(magnitude)

	return aForce
}

// Creates a new spring force to apply spring force between two RigidBodies.
func (forceFactory) NewSpringForce(rigidBody, anchorBody tBokiComponents.RigidBody, restLen, k float64) tBokiVec.Vec2 {

	distance := rigidBody.Pos.Sub(anchorBody.Pos)

	displacement := distance.Mag() - restLen

	direction := distance.Norm()

	magnitude := -k * displacement

	sForce := direction.Scale(magnitude)

	return sForce
}
