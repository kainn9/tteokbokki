package tBokiPhysics

import (
	"math"

	tBokiComponents "github.com/kainn9/tteokbokki/components"
	tBokiMath "github.com/kainn9/tteokbokki/math"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
	tBokiSliceHelper "github.com/kainn9/tteokbokki/sliceHelper"
)

func preSolvePenConstraint(pc *tBokiComponents.PenConstraint, a, b *tBokiComponents.RigidBody, dt float64) {

	// Get collision points in world space.
	pa := a.LocalToWorldSpace(pc.ACollisionPointLocal)
	pb := b.LocalToWorldSpace(pc.BCollisionPointLocal)

	ra := pa.Sub(a.Pos)
	rb := pb.Sub(b.Pos)

	normal := a.LocalToWorldSpace(pc.CollisionNormal)
	inverseNormal := normal.Scale(-1)

	// Row 1 (resolves penetration)
	(*(*pc.Jacobian.Rows)[0])[0] = inverseNormal.X // vel linear A
	(*(*pc.Jacobian.Rows)[0])[1] = inverseNormal.Y // vel linear A

	(*(*pc.Jacobian.Rows)[0])[2] = -ra.CrossProduct(normal) // vel angular A

	(*(*pc.Jacobian.Rows)[0])[3] = normal.X // vel linear B
	(*(*pc.Jacobian.Rows)[0])[4] = normal.Y // vel linear B

	(*(*pc.Jacobian.Rows)[0])[5] = rb.CrossProduct(normal) // vel angular B

	pc.Friction = (a.Friction + b.Friction) / 2.0

	if pc.Friction > 0.0 {

		tanNormal := normal.Perpendicular().Norm()

		// Row 2 (resolves friction)
		(*(*pc.Jacobian.Rows)[1])[0] = -tanNormal.X                // vel linear A
		(*(*pc.Jacobian.Rows)[1])[1] = -tanNormal.Y                // vel linear A
		(*(*pc.Jacobian.Rows)[1])[2] = -ra.CrossProduct(tanNormal) // vel angular A

		(*(*pc.Jacobian.Rows)[1])[3] = tanNormal.X                // vel linear B
		(*(*pc.Jacobian.Rows)[1])[4] = tanNormal.Y                // vel linear B
		(*(*pc.Jacobian.Rows)[1])[5] = rb.CrossProduct(tanNormal) // vel angular B
	}

	// Warm start.
	t := pc.Jacobian.Transpose()
	pc.JacobianTranspose = &t

	impulses, _ := pc.JacobianTranspose.MultiplyBySlice(pc.CachedLambda)

	Transformer.ApplyImpulseLinear(a, tBokiVec.Vec2{X: impulses[0], Y: impulses[1]})
	Transformer.ApplyImpulseAngular(a, impulses[2])
	Transformer.ApplyImpulseLinear(b, tBokiVec.Vec2{X: impulses[3], Y: impulses[4]})
	Transformer.ApplyImpulseAngular(b, impulses[5])

	// Setting bias.
	va := a.Vel.Add(tBokiVec.Vec2{
		X: -a.AngularVel * ra.Y,
		Y: a.AngularVel * ra.X,
	})

	vb := b.Vel.Add(tBokiVec.Vec2{
		X: -b.AngularVel * rb.Y,
		Y: b.AngularVel * rb.X,
	})

	relVelDotNormal := va.Sub(vb).ScalarProduct(normal)
	e := (a.Elasticity + b.Elasticity) / 2.0

	beta := 0.35
	positionalError := pb.Sub(pa).ScalarProduct(inverseNormal)
	positionalError = math.Min(0.0, positionalError+0.01)
	pc.Bias = (beta/dt)*positionalError + (relVelDotNormal * e)

}

func solvePenConstraint(pc *tBokiComponents.PenConstraint, a, b *tBokiComponents.RigidBody) {

	velSlice := VelocitiesSlice(*a, *b)

	invMassMat := InverseMassMatrix(*a, *b)
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
	rhs = tBokiSliceHelper.MultiplySliceByFloat(rhs, -1.0)
	rhs[0] -= pc.Bias

	lambda := lhs.SolveGaussSeidel(rhs)

	// Accumulate impulses and clamp.
	oldLambda := make([]float64, len(pc.CachedLambda))
	copy(oldLambda, pc.CachedLambda)

	pc.CachedLambda = tBokiSliceHelper.AddSlices(pc.CachedLambda, lambda)
	pc.CachedLambda[0] = math.Max(0.0, pc.CachedLambda[0])

	if pc.Friction > 0.0 {
		maxFriction := pc.Friction * pc.CachedLambda[0]
		pc.CachedLambda[1] = tBokiMath.Clamp(pc.CachedLambda[1], -maxFriction, maxFriction)
	}

	lambda = tBokiSliceHelper.SubtractSlices(pc.CachedLambda, oldLambda)

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

	Transformer.ApplyImpulseLinear(a, tBokiVec.Vec2{X: impulses[0], Y: impulses[1]}) // Linear A
	Transformer.ApplyImpulseAngular(a, impulses[2])                                  // Angular A

	Transformer.ApplyImpulseLinear(b, tBokiVec.Vec2{X: impulses[3], Y: impulses[4]}) // Linear B
	Transformer.ApplyImpulseAngular(b, impulses[5])                                  // Angular B

	// Post Clamp.
	pc.CachedLambda[0] = tBokiMath.Clamp(pc.CachedLambda[0], -10000.0, 10000.0)

}
