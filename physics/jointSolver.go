package tBokiPhysics

import (
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
	tBokiSliceHelper "github.com/kainn9/tteokbokki/sliceHelper"
)

func preSolveJointConstraintLinear(jc *tBokiComponents.JointConstraint, a, b *tBokiComponents.RigidBody, dt float64) {

	pa := a.LocalToWorldSpace(jc.ALocal)
	pb := b.Pos

	// A
	j1 := pa.Sub(pb).Scale(2.0)

	(*(*jc.Jacobian.Rows)[0])[0] = j1.X // a linear velocity X
	(*(*jc.Jacobian.Rows)[0])[1] = j1.Y // a linear velocity Y

	// B
	j3 := pb.Sub(pa).Scale(2.0)

	(*(*jc.Jacobian.Rows)[0])[3] = j3.X // b linear velocity X
	(*(*jc.Jacobian.Rows)[0])[4] = j3.Y // b linear velocity Y

	// compute bias
	desiredDistanceSquared :=
		b.LocalToWorldSpace(jc.BLocal).Sub(pb).MagSquared()

	currentDistanceSquared := pb.Sub(pa).MagSquared()

	beta := 0.1 // Adjust this value as needed
	positionalError := currentDistanceSquared - desiredDistanceSquared
	jc.Bias = (beta / dt) * positionalError

}

func solveJointConstraintLinear(jc *tBokiComponents.JointConstraint, a, b *tBokiComponents.RigidBody) {

	invMassMat := InverseMassMatrix(*a, *b)
	velSlice := VelocitiesSlice(*a, *b)
	jacobianTranspose := jc.Jacobian.Transpose()

	rhs, _ := jc.Jacobian.MultiplyBySlice(velSlice)
	rhs = tBokiSliceHelper.MultiplySliceByFloat(rhs, -1.0)
	rhs[0] -= jc.Bias

	lhs, _ := jc.Jacobian.Multiply(invMassMat)

	lhs, _ = lhs.Multiply(jacobianTranspose)

	lambda := lhs.SolveGaussSeidel(rhs)

	impulses, _ := jacobianTranspose.MultiplyBySlice(lambda)

	Transformer.ApplyImpulseLinear(a, tBokiVec.Vec2{X: impulses[0], Y: impulses[1]})
	Transformer.ApplyImpulseLinear(b, tBokiVec.Vec2{X: impulses[3], Y: impulses[4]})

}

func preSolveJointConstraint(jc *tBokiComponents.JointConstraint, a, b *tBokiComponents.RigidBody, dt float64) {

	pa := a.LocalToWorldSpace(jc.ALocal)
	pb := b.LocalToWorldSpace(jc.BLocal)

	ra := pa.Sub(a.Pos)
	rb := pb.Sub(b.Pos)

	// A
	j1 := pa.Sub(pb).Scale(2.0)

	(*(*jc.Jacobian.Rows)[0])[0] = j1.X // a linear velocity X
	(*(*jc.Jacobian.Rows)[0])[1] = j1.Y // a linear velocity Y

	j2 := ra.CrossProduct(pa.Sub(pb)) * 2.0

	(*(*jc.Jacobian.Rows)[0])[2] = j2 // a angular velocity

	// B
	j3 := pb.Sub(pa).Scale(2.0)

	(*(*jc.Jacobian.Rows)[0])[3] = j3.X // b linear velocity X
	(*(*jc.Jacobian.Rows)[0])[4] = j3.Y // b linear velocity Y

	j4 := rb.CrossProduct(pb.Sub(pa)) * 2.0
	(*(*jc.Jacobian.Rows)[0])[5] = j4 // b angular velocity

	// Warm Starting
	t := jc.Jacobian.Transpose()
	jc.JacobianTranspose = &t

	impulses, _ := jc.JacobianTranspose.MultiplyBySlice(jc.CachedLambda)

	Transformer.ApplyImpulseLinear(a, tBokiVec.Vec2{X: impulses[0], Y: impulses[1]})
	Transformer.ApplyImpulseAngular(a, impulses[2])
	Transformer.ApplyImpulseLinear(b, tBokiVec.Vec2{X: impulses[3], Y: impulses[4]})
	Transformer.ApplyImpulseAngular(b, impulses[5])

	// compute bias
	beta := 0.1
	positionalError := pb.Sub(pa).MagSquared()
	jc.Bias = (beta / dt) * positionalError
}

func solveJointConstraint(jc *tBokiComponents.JointConstraint, a, b *tBokiComponents.RigidBody) {

	velSlice := VelocitiesSlice(*a, *b)

	invMassMat := InverseMassMatrix(*a, *b)

	lhs, _ := jc.Jacobian.Multiply(invMassMat)
	lhs, _ = lhs.Multiply(*jc.JacobianTranspose)

	rhs, _ := jc.Jacobian.MultiplyBySlice(velSlice)
	rhs = tBokiSliceHelper.MultiplySliceByFloat(rhs, -1.0)
	rhs[0] -= jc.Bias

	lambda := lhs.SolveGaussSeidel(rhs)

	jc.CachedLambda = tBokiSliceHelper.AddSlices(jc.CachedLambda, lambda)
	impulses, _ := jc.JacobianTranspose.MultiplyBySlice(lambda)

	Transformer.ApplyImpulseLinear(a, tBokiVec.Vec2{X: impulses[0], Y: impulses[1]})
	Transformer.ApplyImpulseAngular(a, impulses[2])
	Transformer.ApplyImpulseLinear(b, tBokiVec.Vec2{X: impulses[3], Y: impulses[4]})
	Transformer.ApplyImpulseAngular(b, impulses[5])

}
