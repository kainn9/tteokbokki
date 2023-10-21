package tBokiPhysics

import (
	"math"

	tBokiComponents "github.com/kainn9/tteokbokki/components"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

type detector struct{}

var Detector detector

// Detect performs a collision check between two rigid bodies.
// It returns true if the two rigid bodies are colliding and a slice of contact objects
// that can be used for collision resolution with a resolver or to create a penetration
// constraint for solving the collision with a penetration solver.
// If useResolver is true, the returned contacts should be used with a resolver
// instead of a penetration solver.
func (detector) Detect(a, b *tBokiComponents.RigidBody, useResolver bool) (bool, []tBokiComponents.Contact) {
	if a.Circle != nil && b.Circle != nil {
		return detectCircleCollision(a, b)
	}

	if a.Polygon != nil && b.Polygon != nil {
		if useResolver {
			return detectPolygonCollision(a, b)
		}
		return detectPolygonCollisions(a, b)
	}

	if a.Polygon != nil && b.Circle != nil {
		return detectPolygonCircleCollision(a, b, false)
	}

	if a.Circle != nil && b.Polygon != nil {
		return detectPolygonCircleCollision(b, a, true)
	}

	return false, make([]tBokiComponents.Contact, 0)
}

func detectCircleCollision(a, b *tBokiComponents.RigidBody) (isColliding bool, contacts []tBokiComponents.Contact) {

	contacts = append(contacts, tBokiComponents.NewContact())
	cPtr := &contacts[0]

	distanceBetween := b.Pos.Sub(a.Pos)

	radiusSum := b.Circle.Radius + a.Circle.Radius

	// Use squared versions to avoid call to sqrt(Mag()).
	isColliding = distanceBetween.MagSquared() <= (radiusSum * radiusSum)

	if !isColliding {
		return isColliding, contacts
	}

	cPtr.Normal = distanceBetween.Norm()

	cPtr.Start = b.Pos.Sub(cPtr.Normal.Scale(b.Circle.Radius))
	cPtr.End = a.Pos.Add(cPtr.Normal.Scale(a.Circle.Radius))

	cPtr.Depth = cPtr.End.Sub(cPtr.Start).Mag()

	return isColliding, contacts

}

func detectPolygonCollision(a, b *tBokiComponents.RigidBody) (isColliding bool, contacts []tBokiComponents.Contact) {

	minSepA, incidentEdgeIndexA, penPointA := Util.FindMinSep(*a, *b)
	incidentEdgeA, _, _ := Util.Edge(incidentEdgeIndexA, *a)

	if minSepA >= 0 {
		return false, contacts
	}

	minSepB, incidentEdgeIndexB, penPointB := Util.FindMinSep(*b, *a)
	incidentEdgeB, _, _ := Util.Edge(incidentEdgeIndexB, *b)

	if minSepB >= 0 {
		return false, contacts
	}

	c := tBokiComponents.NewContact()

	if minSepA > minSepB {
		c.Depth = -minSepA
		c.Normal = incidentEdgeA.Perpendicular().Norm()
		c.Start = penPointA
		c.End = c.Start.Add(c.Normal.Scale(c.Depth))

	} else {

		c.Depth = -minSepB
		c.Normal = incidentEdgeB.Perpendicular().Norm().Scale(-1)
		c.Start = penPointB.Sub(c.Normal.Scale(c.Depth))
		c.End = penPointB
	}

	return true, append(contacts, c)
}

func detectPolygonCollisions(a, b *tBokiComponents.RigidBody) (isColliding bool, contacts []tBokiComponents.Contact) {
	// Reference edge is the edge with the greatest penetration.
	// The incident edge the edge from the penetrating body that has
	// the normal most unaligned with the reference edge normal.
	minSepA, aIndexReferenceEdge, _ := Util.FindMinSep(*a, *b)

	// Positive separation means no collision(no penetration).
	if minSepA >= 0 {
		return false, contacts
	}
	minSepB, bIndexReferenceEdge, _ := Util.FindMinSep(*b, *a)
	if minSepB >= 0 {
		return false, contacts
	}

	// Determine which body is the reference body
	// and which is the incident body, based on the
	// minimum separation.
	var refBody *tBokiComponents.RigidBody
	var incidentBody *tBokiComponents.RigidBody
	var indexReferenceEdge int

	if minSepA > minSepB {
		refBody = a
		incidentBody = b
		indexReferenceEdge = aIndexReferenceEdge
	} else {
		refBody = b
		incidentBody = a
		indexReferenceEdge = bIndexReferenceEdge
	}

	referenceEdge, _, _ := Util.Edge(indexReferenceEdge, *refBody)
	referenceEdgeNormal := referenceEdge.Perpendicular().Norm()

	incidentEdgeIndex := Util.FindIncidentEdgeIndex(incidentBody, referenceEdgeNormal)
	incidentEdgeNextIndex := (incidentEdgeIndex + 1) % len(incidentBody.Polygon.WorldVertices)

	// -- Clipping --------------------------------------------------
	// - Clip the incident edge against the ref edge's
	//   side-planes/clip-planes, but not the ref edge itself.
	// - Only consider points with negative separation(penetration)
	// --------------------------------------------------------------
	// Clipping planes example(incident body not shown):
	//
	//               |               |
	//			     |               |
	// 			     |               |
	// clipPlane0 -->|               | <--- clipPlane1
	//     _______ c0_______________c1______________ another clipPlane
	//               |               |
	//               |    RefBody    |
	//               |               |
	//     __________|_______________|_____________ another clipPlane
	//               |			     |
	//    	         |               |
	//               |               |
	//
	// Note:
	// - The incident edge is not shown in the diagram above.
	// --------------------------------------------------------------
	// Incident edge vertices.
	v0 := incidentBody.Polygon.WorldVertices[incidentEdgeIndex]
	v1 := incidentBody.Polygon.WorldVertices[incidentEdgeNextIndex]
	contactPoints := make([]tBokiVec.Vec2, 2)
	contactPoints[0] = v0
	contactPoints[1] = v1

	// Loop through all the vertices/edges of the reference body.
	// Clip the incident edge/vertices against the reference edge's clip-planes.
	for i := range refBody.Polygon.WorldVertices {

		// Don't clip against the reference edge.
		if i == indexReferenceEdge {
			continue
		}

		clipPlane0Vert := refBody.Polygon.WorldVertices[i]
		clipPlane1Vert := refBody.Polygon.WorldVertices[(i+1)%len(refBody.Polygon.WorldVertices)]

		numClipped, clippedPoints := Util.ClipSegmentToLine(contactPoints, clipPlane0Vert, clipPlane1Vert)

		// Something went wrong we should have at least two points.
		if numClipped < 2 {
			break
		}

		contactPoints = clippedPoints
	}

	vertRefEdge := refBody.Polygon.WorldVertices[indexReferenceEdge]

	// Only consider clipped points that are behind the reference edge
	// (penetrating the reference edge).
	for _, vertClip := range contactPoints {
		separation := vertClip.Sub(vertRefEdge).ScalarProduct(referenceEdgeNormal)

		if separation > 0 {
			continue
		}

		c := tBokiComponents.NewContact()
		c.Normal = referenceEdgeNormal
		c.Start = vertClip
		c.End = vertClip.Add(referenceEdgeNormal.Scale(-separation))

		if minSepB >= minSepA {
			c.Normal = c.Normal.Scale(-1)
			c.Start, c.End = c.End, c.Start

		}

		contacts = append(contacts, c)
	}

	return true, contacts
}

func detectPolygonCircleCollision(polygonBody, circleBody *tBokiComponents.RigidBody, abSwapped bool) (isColliding bool, contacts []tBokiComponents.Contact) {
	// If abSwapped is true, we need to swap the normal and
	// start/end. This is because the normal is always calculated
	// from the polygon to the circle(vertToCircleCenter).
	defer func() {
		if !abSwapped {
			return
		}

		for i := range contacts {
			contacts[i].Normal = contacts[i].Normal.Scale(-1)
			contacts[i].Start, contacts[i].End = contacts[i].End, contacts[i].Start
		}
	}()

	contacts = append(contacts, tBokiComponents.NewContact())
	cPtr := &contacts[0]

	// getting the nearest edge/vertices to the circle center
	circleCenterOutside := false
	var minCurrVert, minNextVert tBokiVec.Vec2
	distCircleEdge := -math.MaxFloat64

	for i, currVert := range polygonBody.Polygon.WorldVertices {

		edge, _, nextVert := Util.Edge(i, *polygonBody)
		normal := edge.Perpendicular().Norm()

		vertToCircleCenter := circleBody.Pos.Sub(currVert)
		penCheckProjection := vertToCircleCenter.ScalarProduct(normal)

		if penCheckProjection > 0 {

			distCircleEdge = penCheckProjection
			minCurrVert = currVert
			minNextVert = nextVert

			circleCenterOutside = true
			break

		} else if penCheckProjection > distCircleEdge {

			distCircleEdge = penCheckProjection
			minCurrVert = currVert
			minNextVert = nextVert
		}

	}

	// the circle center is outside the polygon
	// (penetrating the nearest edge)
	// but we might still be colliding...
	if circleCenterOutside {

		// left of edge bounds
		vertToCircleCenter := circleBody.Pos.Sub(minCurrVert)
		nearestEdge := minNextVert.Sub(minCurrVert)

		circleCenterLeftOfEdge := vertToCircleCenter.ScalarProduct(nearestEdge) < 0

		if circleCenterLeftOfEdge {
			mag := vertToCircleCenter.Mag()

			if mag > circleBody.Circle.Radius {
				return false, contacts
			} else {
				cPtr.Depth = circleBody.Circle.Radius - mag
				cPtr.Normal = vertToCircleCenter.Norm()
				cPtr.Start = circleBody.Pos.Add(cPtr.Normal.Scale(-circleBody.Circle.Radius))
				cPtr.End = cPtr.Start.Add(cPtr.Normal.Scale(cPtr.Depth))
				return true, contacts
			}

		}

		// right of edge bounds
		vertToCircleCenter = circleBody.Pos.Sub(minNextVert)
		nearestEdge = minCurrVert.Sub(minNextVert)

		circleCenterRightOfEdge := vertToCircleCenter.ScalarProduct(nearestEdge) < 0

		if circleCenterRightOfEdge {
			mag := vertToCircleCenter.Mag()

			if mag > circleBody.Circle.Radius {
				return false, contacts
			} else {
				cPtr.Depth = circleBody.Circle.Radius - mag
				cPtr.Normal = vertToCircleCenter.Norm()
				cPtr.Start = circleBody.Pos.Add(cPtr.Normal.Scale(-circleBody.Circle.Radius))
				cPtr.End = cPtr.Start.Add(cPtr.Normal.Scale(cPtr.Depth))
				return true, contacts
			}

		}

		// circle center is within the edge bounds
		if distCircleEdge > circleBody.Circle.Radius {
			return false, contacts
		}

		cPtr.Depth = circleBody.Circle.Radius - distCircleEdge
		cPtr.Normal = minNextVert.Sub(minCurrVert).Perpendicular().Norm()

		cPtr.Start = circleBody.Pos.Sub(cPtr.Normal.Scale(circleBody.Circle.Radius))
		cPtr.End = cPtr.Start.Add(cPtr.Normal.Scale(cPtr.Depth))

		return true, contacts

	}

	// If circle center is inside the polygon
	// we are definitely colliding
	cPtr.Depth = circleBody.Circle.Radius - distCircleEdge
	cPtr.Normal = minNextVert.Sub(minCurrVert).Perpendicular().Norm()
	cPtr.Start = circleBody.Pos.Sub(cPtr.Normal.Scale(circleBody.Circle.Radius))
	cPtr.End = cPtr.Start.Add(cPtr.Normal.Scale(cPtr.Depth))
	return true, contacts
}
