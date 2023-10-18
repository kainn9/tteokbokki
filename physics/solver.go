package tBokiPhysics

import (
	tBokiComponents "github.com/kainn9/tteokbokki/components"
)

type solver struct{}

var Solver solver

// Solve will solve a give set of constraints.
// Currently only supports PenConstraints and JointConstraints(distance).
// Note: a & b must be the same rigid bodies used to generate the constraint.
func (solver) Solve(constraints any, aBodies, bBodies []*tBokiComponents.RigidBody, iterations int, dt float64) {

	if jointConstraint, ok := (constraints).([]*tBokiComponents.JointConstraint); ok {

		for i, constraint := range jointConstraint {

			if bBodies[i].IsAngular() {
				preSolveJointConstraint(constraint, aBodies[i], bBodies[i], dt)
			} else {
				preSolveJointConstraintLinear(constraint, aBodies[i], bBodies[i], dt)
			}

		}

		for j := 0; j < iterations; j++ {
			for k, constraint := range jointConstraint {

				if bBodies[k].IsAngular() {
					solveJointConstraint(constraint, aBodies[k], bBodies[k])
				} else {
					solveJointConstraintLinear(constraint, aBodies[k], bBodies[k])
				}

			}
		}
	}

	if penConstraint, ok := (constraints).([]*tBokiComponents.PenConstraint); ok {

		for i, constraint := range penConstraint {
			preSolvePenConstraint(constraint, aBodies[i], bBodies[i], dt)
		}

		for j := 0; j < iterations; j++ {
			for k, constraint := range penConstraint {
				solvePenConstraint(constraint, aBodies[k], bBodies[k])
			}
		}
	}
}
