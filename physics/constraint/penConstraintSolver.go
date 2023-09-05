package constraint

import (
	"math"

	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	mathHelper "github.com/kainn9/tteokbokki/math"
	"github.com/kainn9/tteokbokki/math/vec"
	"github.com/kainn9/tteokbokki/sliceHelper"
)

func (s *Solver) PreSolvePenConstraint(pc *componentsPhysicsTypes.PenConstraint, dt float64) {
	// get collision points in world space
	pa := pc.A.LocalToWorldSpace(pc.ACollisionPointLocal)
	pb := pc.B.LocalToWorldSpace(pc.BCollisionPointLocal)

	ra := pa.Sub(pc.A.Pos)
	rb := pb.Sub(pc.B.Pos)

	normal := pc.A.LocalToWorldSpace(pc.CollisionNormal)
	inverseNormal := normal.Scale(-1)
	// populate jacobian row 1

	(*(*pc.Jacobian.Rows)[0])[0] = inverseNormal.X
	(*(*pc.Jacobian.Rows)[0])[1] = inverseNormal.Y

	(*(*pc.Jacobian.Rows)[0])[2] = -ra.CrossProduct(normal)

	(*(*pc.Jacobian.Rows)[0])[3] = normal.X
	(*(*pc.Jacobian.Rows)[0])[4] = normal.Y

	(*(*pc.Jacobian.Rows)[0])[5] = rb.CrossProduct(normal)

	// populate jacobian row 2 for friction
	// on the tangent normal
	pc.Friction = (pc.A.Friction + pc.B.Friction) / 2.0

	if pc.Friction > 0.0 {
		tanNormal := normal.Perpendicular()
		(*(*pc.Jacobian.Rows)[1])[0] = -tanNormal.X
		(*(*pc.Jacobian.Rows)[1])[1] = -tanNormal.Y
		(*(*pc.Jacobian.Rows)[1])[2] = -ra.CrossProduct(tanNormal)

		(*(*pc.Jacobian.Rows)[1])[3] = tanNormal.X
		(*(*pc.Jacobian.Rows)[1])[4] = tanNormal.Y
		(*(*pc.Jacobian.Rows)[1])[5] = rb.CrossProduct(tanNormal)
	}

	// warm start
	t := pc.Jacobian.Transpose()
	pc.JacobianTranspose = &t

	impulses, _ := pc.JacobianTranspose.MultiplyBySlice(pc.CachedLambda)

	transformer.ApplyImpulseLinear(pc.A, vec.Vec2{X: impulses[0], Y: impulses[1]})
	transformer.ApplyImpulseAngular(pc.A, impulses[2])
	transformer.ApplyImpulseLinear(pc.B, vec.Vec2{X: impulses[3], Y: impulses[4]})
	transformer.ApplyImpulseAngular(pc.B, impulses[5])

	// setting bias
	va := pc.A.Vel.Add(vec.Vec2{
		X: -pc.A.AngularVel * ra.Y,
		Y: pc.A.AngularVel * ra.X,
	})

	vb := pc.B.Vel.Add(vec.Vec2{
		X: -pc.B.AngularVel * rb.Y,
		Y: pc.B.AngularVel * rb.X,
	})

	relVelDotNormal := va.Sub(vb).ScalarProduct(normal)
	e := (pc.A.Elasticity + pc.B.Elasticity) / 2.0

	beta := 0.2
	positionalError := pb.Sub(pa).ScalarProduct(inverseNormal)
	positionalError = math.Min(0.0, positionalError+0.01)
	pc.Bias = (beta/dt)*positionalError + (relVelDotNormal * e)

}

func (s *Solver) SolvePenConstraint(pc *componentsPhysicsTypes.PenConstraint) {

	velSlice := getVelocitiesSlice(*pc.A, *pc.B)

	invMassMat := getInverseMassMatrix(*pc.A, *pc.B)

	lhs, _ := pc.Jacobian.Multiply(invMassMat)
	lhs, _ = lhs.Multiply(*pc.JacobianTranspose)

	rhs, _ := pc.Jacobian.MultiplyBySlice(velSlice)
	rhs = sliceHelper.MultiplySliceByFloat(rhs, -1.0)
	rhs[0] -= pc.Bias

	lambda := lhs.SolveGaussSeidel(rhs)

	// accumulate impulses and clamp
	oldLambda := pc.CachedLambda
	pc.CachedLambda = sliceHelper.AddSlices(pc.CachedLambda, lambda)
	pc.CachedLambda[0] = math.Max(0.0, pc.CachedLambda[0])

	if pc.Friction > 0.0 {
		maxFriction := pc.Friction * pc.CachedLambda[0]
		pc.CachedLambda[1] = mathHelper.Clamp(pc.CachedLambda[1], -maxFriction, maxFriction)
	}

	lambda = sliceHelper.SubtractSlices(pc.CachedLambda, oldLambda)

	impulses, _ := pc.JacobianTranspose.MultiplyBySlice(lambda)

	transformer.ApplyImpulseLinear(pc.A, vec.Vec2{X: impulses[0], Y: impulses[1]})
	transformer.ApplyImpulseAngular(pc.A, impulses[2])

	transformer.ApplyImpulseLinear(pc.B, vec.Vec2{X: impulses[3], Y: impulses[4]})
	transformer.ApplyImpulseAngular(pc.B, impulses[5])

}
