package tBokiPhysics

import (
	"math"

	tBokiComponents "github.com/kainn9/tteokbokki/components"
	tBokiMatrix "github.com/kainn9/tteokbokki/math/matrix"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

type util struct{}

var Util util

// FindMinSep finds the minimum separation between two rigid bodies.
// Requires vertices and will not work with circles.
// Note: reference edge is the edge with the greatest penetration.
func (u util) FindMinSep(a, b tBokiComponents.RigidBody) (minSep float64, indexReferenceEdge int, penPoint tBokiVec.Vec2) {

	sep := -math.MaxFloat64

	for i, va := range a.Polygon.WorldVertices {

		currentEdge, _, _ := u.Edge(i, a)
		normal := currentEdge.Perpendicular().Norm()

		minSep = math.MaxFloat64
		minVert := tBokiVec.Vec2{}

		for _, vb := range b.Polygon.WorldVertices {

			projection := vb.Sub(va).ScalarProduct(normal)

			if projection < minSep {
				minSep = projection
				minVert = vb
			}

		}

		if minSep > sep {
			sep = minSep
			indexReferenceEdge = i
			penPoint = minVert
		}
	}

	return sep, indexReferenceEdge, penPoint
}

// Incident edge is the edge of the incident edge is
// with the normal most unaligned with the reference edge normal.
func (u util) FindIncidentEdgeIndex(incidentBody *tBokiComponents.RigidBody, refEdgeNormal tBokiVec.Vec2) int {
	var incidentEdgeIndex int
	minProjection := math.MaxFloat64

	for i := range incidentBody.Polygon.WorldVertices {
		edgeNormal, _, _ := u.Edge(i, *incidentBody)
		edgeNormal = edgeNormal.Perpendicular().Norm()

		projection := edgeNormal.ScalarProduct(refEdgeNormal)

		if projection < minProjection {
			minProjection = projection
			incidentEdgeIndex = i
		}
	}

	return incidentEdgeIndex
}

// Edge returns the edge, and the two vertices that make up the edge.
func (util) Edge(index int, rb tBokiComponents.RigidBody) (edge, v1, v2 tBokiVec.Vec2) {

	nextIndex := (index + 1) % len(rb.Polygon.WorldVertices)

	vb := (rb.Polygon.WorldVertices)[nextIndex]
	va := (rb.Polygon.WorldVertices)[index]

	return vb.Sub(va), va, vb
}

// Clipping function used to help
// produce more stable contact-points/impulses
// when two polygon rigid bodies are colliding.
func (util) ClipSegmentToLine(contactPoints []tBokiVec.Vec2, c0, c1 tBokiVec.Vec2) (count int, contactsOut []tBokiVec.Vec2) {
	contactsOut = make([]tBokiVec.Vec2, 2)

	normal := c1.Sub(c0).Perpendicular().Norm()

	// get distance from contact points to c0-c1 line
	dist0 := contactPoints[0].Sub(c0).ScalarProduct(normal)
	dist1 := contactPoints[1].Sub(c0).ScalarProduct(normal)

	// If both are negative the incident edge is
	// not intersecting the clipping plane
	if dist0 <= 0 {
		contactsOut[count] = contactPoints[0]
		count += 1
	}

	if dist1 <= 0 {
		contactsOut[count] = contactPoints[1]
		count += 1
	}

	// This indicates a negative and positive dist.
	// This means the incident edge is intersecting the clipping plane.
	if dist0*dist1 < 0 {
		// Use lerp to find the intersection point.
		totalDist := (dist0 - dist1)
		d := dist0 / totalDist
		contactsOut[count] = contactPoints[0].Add(contactPoints[1].Sub(contactPoints[0]).Scale(d))
		count += 1
	}

	return count, contactsOut

}

func InverseMassMatrix(a, b tBokiComponents.RigidBody) tBokiMatrix.MatRC {
	matrix := tBokiMatrix.NewMatRC(6, 6)

	// a
	(*(*matrix.Rows)[0])[0] = a.InverseMass
	(*(*matrix.Rows)[1])[1] = a.InverseMass
	(*(*matrix.Rows)[2])[2] = a.InverseAngularMass

	// b
	(*(*matrix.Rows)[3])[3] = b.InverseMass
	(*(*matrix.Rows)[4])[4] = b.InverseMass
	(*(*matrix.Rows)[5])[5] = b.InverseAngularMass

	return matrix

}

func VelocitiesSlice(a, b tBokiComponents.RigidBody) []float64 {
	velocities := make([]float64, 6)

	velocities[0] = a.Vel.X
	velocities[1] = a.Vel.Y
	velocities[2] = a.AngularVel

	velocities[3] = b.Vel.X
	velocities[4] = b.Vel.Y
	velocities[5] = b.AngularVel

	return velocities
}
