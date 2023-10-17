package collision

import (
	"github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/math/vec"
	"github.com/kainn9/tteokbokki/physics/transform"
)

type Resolver struct{}

var transformer = transform.Transformer{}

// Updates position and applies velocity to resolve collisions based on mass/angular-mass.
// Args a & b should match the order that was used in Check().
func (r Resolver) Impulse(c components.Contact, a, b *components.RigidBody) {

	bothBodiesStatic :=
		(a.IsStatic() || a.Unstoppable) && (b.IsStatic() || b.Unstoppable)

	if bothBodiesStatic {
		return
	}

	r.project(c, a, b)

	impulse, angularImpulseFactorA, angularImpulseFactorB := getImpulses(c, a, b)

	if !a.Unstoppable {
		transformer.ApplyImpulse(a, impulse, angularImpulseFactorA)
	}

	if !b.Unstoppable {
		transformer.ApplyImpulse(b, impulse.Scale(-1), angularImpulseFactorB)
	}

}

// Updates the position to resolve collisions based on mass.
// Note: Project the verb, not the noun — as in projection.
func (Resolver) project(c components.Contact, a, b *components.RigidBody) {

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

func getImpulses(c components.Contact, a, b *components.RigidBody) (vec.Vec2, vec.Vec2, vec.Vec2) {

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

	ra := c.End.Sub(a.Pos)
	rb := c.Start.Sub(b.Pos)

	va := a.Vel.Add(vec.Vec2{
		X: -a.AngularVel * ra.Y,
		Y: a.AngularVel * ra.X,
	})

	vb := b.Vel.Add(vec.Vec2{
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

	impulseMagT := f * (-(1 + e) * velRelDotTan / denomT)

	impulseT := impulseDirectionT.Scale(impulseMagT)

	return impulseN.Add(impulseT), ra, rb
}
