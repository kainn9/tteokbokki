package constraint

import (
	"github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/math/vec"
	"github.com/kainn9/tteokbokki/physics/transform"
	"github.com/kainn9/tteokbokki/sliceHelper"
)

var transformer = transform.Transformer{}

func (s *Solver) PreSolveJointConstraintLinear(jc *components.JointConstraintComponent, dt float64) {

	pa := jc.A.LocalToWorldSpace(jc.ALocal)
	pb := jc.B.Pos

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
		jc.B.LocalToWorldSpace(jc.BLocal).Sub(pb).MagSquared()

	currentDistanceSquared := pb.Sub(pa).MagSquared()

	beta := 0.1 // Adjust this value as needed
	positionalError := currentDistanceSquared - desiredDistanceSquared
	jc.Bias = (beta / dt) * positionalError

}

func (s *Solver) SolveJointConstraintLinear(jc *components.JointConstraintComponent) {

	invMassMat := getInverseMassMatrix(*jc.A, *jc.B)
	velSlice := getVelocitiesSlice(*jc.A, *jc.B)
	jacobianTranspose := jc.Jacobian.Transpose()

	rhs, _ := jc.Jacobian.MultiplyBySlice(velSlice)
	rhs = sliceHelper.MultiplySliceByFloat(rhs, -1.0)
	rhs[0] -= jc.Bias

	lhs, _ := jc.Jacobian.Multiply(invMassMat)

	lhs, _ = lhs.Multiply(jacobianTranspose)

	lambda := lhs.SolveGaussSeidel(rhs)

	impulses, _ := jacobianTranspose.MultiplyBySlice(lambda)

	transformer.ApplyImpulseLinear(jc.A, vec.Vec2{X: impulses[0], Y: impulses[1]})
	transformer.ApplyImpulseLinear(jc.B, vec.Vec2{X: impulses[3], Y: impulses[4]})

}

func (s *Solver) PreSolveJointConstraint(jc *components.JointConstraintComponent, dt float64) {

	a := jc.A
	b := jc.B

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

	transformer.ApplyImpulseLinear(a, vec.Vec2{X: impulses[0], Y: impulses[1]})
	transformer.ApplyImpulseAngular(a, impulses[2])
	transformer.ApplyImpulseLinear(b, vec.Vec2{X: impulses[3], Y: impulses[4]})
	transformer.ApplyImpulseAngular(b, impulses[5])

	// compute bias
	beta := 0.1
	positionalError := pb.Sub(pa).MagSquared()
	jc.Bias = (beta / dt) * positionalError
}

func (s *Solver) SolveJointConstraint(jc *components.JointConstraintComponent) {
	a := jc.A
	b := jc.B

	velSlice := getVelocitiesSlice(*a, *b)

	invMassMat := getInverseMassMatrix(*a, *b)

	lhs, _ := jc.Jacobian.Multiply(invMassMat)
	lhs, _ = lhs.Multiply(*jc.JacobianTranspose)

	rhs, _ := jc.Jacobian.MultiplyBySlice(velSlice)
	rhs = sliceHelper.MultiplySliceByFloat(rhs, -1.0)
	rhs[0] -= jc.Bias

	lambda := lhs.SolveGaussSeidel(rhs)

	jc.CachedLambda = sliceHelper.AddSlices(jc.CachedLambda, lambda)
	impulses, _ := jc.JacobianTranspose.MultiplyBySlice(lambda)

	transformer.ApplyImpulseLinear(a, vec.Vec2{X: impulses[0], Y: impulses[1]})
	transformer.ApplyImpulseAngular(a, impulses[2])
	transformer.ApplyImpulseLinear(b, vec.Vec2{X: impulses[3], Y: impulses[4]})
	transformer.ApplyImpulseAngular(b, impulses[5])

}
