package constraint

import (
	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/math/matrix"
)

type Solver struct{}

func getInverseMassMatrix(a, b componentsPhysicsTypes.RigidBody) matrix.MatRC {
	matrix := matrix.NewMatRC(6, 6)

	(*(*matrix.Rows)[0])[0] = a.InverseMass
	(*(*matrix.Rows)[1])[1] = a.InverseMass

	(*(*matrix.Rows)[2])[2] = a.InverseAngularMass

	(*(*matrix.Rows)[3])[3] = b.InverseMass
	(*(*matrix.Rows)[4])[4] = b.InverseMass

	(*(*matrix.Rows)[5])[5] = b.InverseAngularMass

	return matrix

}

func getVelocitiesSlice(a, b componentsPhysicsTypes.RigidBody) []float64 {
	velocities := make([]float64, 6)

	velocities[0] = a.Vel.X
	velocities[1] = a.Vel.Y
	velocities[2] = a.AngularVel

	velocities[3] = b.Vel.X
	velocities[4] = b.Vel.Y
	velocities[5] = b.AngularVel

	return velocities
}
