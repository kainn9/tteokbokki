package tBokiPhysics

import (
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

type resolver struct{}

var Resolver = resolver{}

// Updates position and applies velocity to resolve collisions based on mass/angular-mass.
// Note: a & b must be the same rigid bodies used to generate the contact.
func (resolver) Resolve(a, b *tBokiComponents.RigidBody, contacts tBokiComponents.Contacts) {
	c := contacts.Data[0]

	bothBodiesStatic :=
		(a.IsStatic() || a.Unstoppable) && (b.IsStatic() || b.Unstoppable)

	if bothBodiesStatic {
		return
	}

	project(c, a, b)

	impulse, angularImpulseFactorA, angularImpulseFactorB := getImpulses(c, a, b)

	if !a.Unstoppable {
		Transformer.ApplyImpulse(a, impulse, angularImpulseFactorA)
	}

	if !b.Unstoppable {
		Transformer.ApplyImpulse(b, impulse.Scale(-1), angularImpulseFactorB)
	}

}

// Updates the position to resolve collisions based on mass.
// Note: Project the verb, not the noun — as in projection.
func project(c tBokiComponents.Contact, a, b *tBokiComponents.RigidBody) {

	inverseMassA := a.InverseMass
	if a.Unstoppable {
		inverseMassA = 0
	}

	inverseMassB := b.InverseMass
	if b.Unstoppable {
		inverseMassB = 0
	}

	// da + db will always equal c.Depth
	da := c.Depth / (inverseMassA + inverseMassB) * inverseMassA
	db := c.Depth / (inverseMassA + inverseMassB) * inverseMassB

	a.Pos = a.Pos.Sub(c.Normal.Scale(da))
	b.Pos = b.Pos.Add(c.Normal.Scale(db))

	a.UpdateVertices()
	b.UpdateVertices()

}

func getImpulses(c tBokiComponents.Contact, a, b *tBokiComponents.RigidBody) (tBokiVec.Vec2, tBokiVec.Vec2, tBokiVec.Vec2) {

	inverseMassA := a.InverseMass
	if a.Unstoppable {
		inverseMassA = 0
	}

	inverseMassB := b.InverseMass
	if b.Unstoppable {
		inverseMassB = 0
	}

	e := (a.Elasticity + b.Elasticity) / 2

	f := (a.Friction + b.Friction) / 2

	// Reduce friction coefficient by factor of 10.
	// This is a arbitrary value, that results in the
	// the friction coefficient(f) being nicer to work with
	// && also more similar to solver results given the same
	// value.
	f = f / 10

	ra := c.End.Sub(a.Pos)
	rb := c.Start.Sub(b.Pos)

	va := a.Vel.Add(tBokiVec.Vec2{
		X: -a.AngularVel * ra.Y,
		Y: a.AngularVel * ra.X,
	})

	vb := b.Vel.Add(tBokiVec.Vec2{
		X: -b.AngularVel * rb.Y,
		Y: b.AngularVel * rb.X,
	})

	relVel := va.Sub(vb)

	relVelDotNormal := relVel.ScalarProduct(c.Normal)

	impulseDirectionN := c.Normal

	crossNormalA := ra.CrossProduct(c.Normal)
	crossNormalASq := crossNormalA * crossNormalA

	crossNormalB := rb.CrossProduct(c.Normal)
	crossNormalBSq := crossNormalB * crossNormalB

	abInverseMassSum := inverseMassA + inverseMassB

	denomN := abInverseMassSum + crossNormalASq*a.InverseAngularMass + crossNormalBSq*b.InverseAngularMass

	impulseMagN := -(1 + e) * relVelDotNormal / denomN

	impulseN := impulseDirectionN.Scale(impulseMagN)

	tangent := c.Normal.Perpendicular().Norm()

	velRelDotTan := relVel.ScalarProduct(tangent)

	impulseDirectionT := tangent

	crossTanA := ra.CrossProduct(tangent)
	crossTanASq := crossTanA * crossTanA

	crossTanB := rb.CrossProduct(tangent)
	crossTanBSq := crossTanB * crossTanB

	denomT := abInverseMassSum + crossTanASq*a.InverseAngularMass + crossTanBSq*b.InverseAngularMass

	impulseMagT := f * -(1 + e) * velRelDotTan / denomT

	impulseT := impulseDirectionT.Scale(impulseMagT)

	return impulseN.Add(impulseT), ra, rb
}

// Converted to Golang from:
// https://paulbourke.net/geometry/pointlineplane/javascript.txt
func (resolver) LineIntersection(v1, v2, v3, v4 tBokiVec.Vec2) (bool, map[string]float64) {

	x1 := v1.X
	y1 := v1.Y
	x2 := v2.X
	y2 := v2.Y
	x3 := v3.X
	y3 := v3.Y
	x4 := v4.X
	y4 := v4.Y

	// Check if none of the lines are of length 0
	if (x1 == x2 && y1 == y2) || (x3 == x4 && y3 == y4) {
		return false, nil
	}

	denominator := (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)

	// Lines are parallel
	if denominator == 0 {
		return false, nil
	}

	ua := ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denominator
	ub := ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denominator

	// Is the intersection along the segments
	if ua < 0 || ua > 1 || ub < 0 || ub > 1 {
		return false, nil
	}

	// Return a map with the x and y coordinates of the intersection
	intersection := map[string]float64{"x": x1 + ua*(x2-x1), "y": y1 + ua*(y2-y1)}

	return true, intersection
}
