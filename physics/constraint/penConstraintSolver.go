package constraint

import (
	"math"

	"github.com/kainn9/tteokbokki/components"
	mathHelper "github.com/kainn9/tteokbokki/math"
	"github.com/kainn9/tteokbokki/math/vec"
	"github.com/kainn9/tteokbokki/sliceHelper"
)

// Args a & b should match the order that was used in Check().
func (s *Solver) PreSolvePenConstraint(pc *components.PenConstraint, a, b *components.RigidBody, dt float64) {

	// Get collision points in world space.
	pa := a.LocalToWorldSpace(pc.ACollisionPointLocal)
	pb := b.LocalToWorldSpace(pc.BCollisionPointLocal)

	ra := pa.Sub(a.Pos)
	rb := pb.Sub(b.Pos)

	normal := a.LocalToWorldSpace(pc.CollisionNormal)
	inverseNormal := normal.Scale(-1)

	(*(*pc.Jacobian.Rows)[0])[0] = inverseNormal.X
	(*(*pc.Jacobian.Rows)[0])[1] = inverseNormal.Y

	(*(*pc.Jacobian.Rows)[0])[2] = -ra.CrossProduct(normal)

	(*(*pc.Jacobian.Rows)[0])[3] = normal.X
	(*(*pc.Jacobian.Rows)[0])[4] = normal.Y

	(*(*pc.Jacobian.Rows)[0])[5] = rb.CrossProduct(normal)

	pc.Friction = (a.Friction + b.Friction) / 2.0

	if pc.Friction > 0.0 {
		tanNormal := normal.Perpendicular()
		(*(*pc.Jacobian.Rows)[1])[0] = -tanNormal.X
		(*(*pc.Jacobian.Rows)[1])[1] = -tanNormal.Y
		(*(*pc.Jacobian.Rows)[1])[2] = -ra.CrossProduct(tanNormal)

		(*(*pc.Jacobian.Rows)[1])[3] = tanNormal.X
		(*(*pc.Jacobian.Rows)[1])[4] = tanNormal.Y
		(*(*pc.Jacobian.Rows)[1])[5] = rb.CrossProduct(tanNormal)
	}

	// Warm start.
	t := pc.Jacobian.Transpose()
	pc.JacobianTranspose = &t

	impulses, _ := pc.JacobianTranspose.MultiplyBySlice(pc.CachedLambda)

	transformer.ApplyImpulseLinear(a, vec.Vec2{X: impulses[0], Y: impulses[1]})
	transformer.ApplyImpulseAngular(a, impulses[2])
	transformer.ApplyImpulseLinear(b, vec.Vec2{X: impulses[3], Y: impulses[4]})
	transformer.ApplyImpulseAngular(b, impulses[5])

	// Setting bias.
	va := a.Vel.Add(vec.Vec2{
		X: -a.AngularVel * ra.Y,
		Y: a.AngularVel * ra.X,
	})

	vb := b.Vel.Add(vec.Vec2{
		X: -b.AngularVel * rb.Y,
		Y: b.AngularVel * rb.X,
	})

	relVelDotNormal := va.Sub(vb).ScalarProduct(normal)
	e := (a.Elasticity + b.Elasticity) / 2.0

	beta := 0.2
	positionalError := pb.Sub(pa).ScalarProduct(inverseNormal)
	positionalError = math.Min(0.0, positionalError+0.01)
	pc.Bias = (beta/dt)*positionalError + (relVelDotNormal * e)

}

// SolvePenConstraint solves the penetration constraint.
// Args a & b should match the order that was used in Check().
func (s *Solver) SolvePenConstraint(pc *components.PenConstraint, a, b *components.RigidBody) {

	velSlice := getVelocitiesSlice(*a, *b)

	invMassMat := getInverseMassMatrix(*a, *b)
	if a.Unstoppable {
		(*(*invMassMat.Rows)[0])[0] = 0.0
		(*(*invMassMat.Rows)[1])[1] = 0.0

	}
	if b.Unstoppable {
		(*(*invMassMat.Rows)[3])[3] = 0.0
		(*(*invMassMat.Rows)[4])[4] = 0.0
	}

	lhs, _ := pc.Jacobian.Multiply(invMassMat)
	lhs, _ = lhs.Multiply(*pc.JacobianTranspose)

	rhs, _ := pc.Jacobian.MultiplyBySlice(velSlice)
	rhs = sliceHelper.MultiplySliceByFloat(rhs, -1.0)
	rhs[0] -= pc.Bias

	lambda := lhs.SolveGaussSeidel(rhs)

	// Accumulate impulses and clamp.
	oldLambda := pc.CachedLambda
	pc.CachedLambda = sliceHelper.AddSlices(pc.CachedLambda, lambda)
	pc.CachedLambda[0] = math.Max(0.0, pc.CachedLambda[0])

	if pc.Friction > 0.0 {
		maxFriction := pc.Friction * pc.CachedLambda[0]
		pc.CachedLambda[1] = mathHelper.Clamp(pc.CachedLambda[1], -maxFriction, maxFriction)
	}

	lambda = sliceHelper.SubtractSlices(pc.CachedLambda, oldLambda)

	impulses, _ := pc.JacobianTranspose.MultiplyBySlice(lambda)

	if a.Unstoppable {
		impulses[0] = 0.0
		impulses[1] = 0.0
		impulses[2] = 0.0
	}

	if b.Unstoppable {
		impulses[3] = 0.0
		impulses[4] = 0.0
		impulses[5] = 0.0
	}

	transformer.ApplyImpulseLinear(a, vec.Vec2{X: impulses[0], Y: impulses[1]})
	transformer.ApplyImpulseAngular(a, impulses[2])

	transformer.ApplyImpulseLinear(b, vec.Vec2{X: impulses[3], Y: impulses[4]})
	transformer.ApplyImpulseAngular(b, impulses[5])
}
