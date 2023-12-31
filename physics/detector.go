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
func (detector) Detect(a, b *tBokiComponents.RigidBody, cType tBokiComponents.ContactsType) (bool, tBokiComponents.Contacts) {

	// First Check using the broad phase skin(circles).
	broadPhaseCheck, contacts := detectCircleCollision(a, b, true)
	contacts.Type = cType

	// If the broad phase check fails, we can return early
	// its not possible for the two bodies to be colliding.
	if !broadPhaseCheck {
		return false, contacts
	}

	// If the broad phase check succeeds, we can skip
	// the narrow phase check if both bodies are circles,
	// since the broad phase skin is a circle itself.
	if a.Circle != nil && b.Circle != nil {
		return broadPhaseCheck, contacts
	}

	// Otherwise, we need to perform a narrow phase check
	// for other shapes/bodies.
	if a.Polygon != nil && b.Polygon != nil {

		if cType == tBokiComponents.ResolverType {
			isColliding, contacts := detectPolygonCollision(a, b)
			contacts.Type = cType
			return isColliding, contacts
		}

		isColliding, contacts := detectPolygonCollisions(a, b)
		contacts.Type = cType
		return isColliding, contacts
	}

	if a.Polygon != nil && b.Circle != nil {
		return detectPolygonCircleCollision(a, b, false)
	}

	if a.Circle != nil && b.Polygon != nil {
		return detectPolygonCircleCollision(b, a, true)
	}

	return false, tBokiComponents.NewContacts(cType)
}

func detectCircleCollision(a, b *tBokiComponents.RigidBody, broadPhaseCheck bool) (isColliding bool, contacts tBokiComponents.Contacts) {

	circleA := a.Circle
	circleB := b.Circle

	if broadPhaseCheck {
		circleA = a.BroadPhaseSkin
		circleB = b.BroadPhaseSkin
	}

	contacts.Data = append(contacts.Data, tBokiComponents.NewContact())
	cPtr := &contacts.Data[0]

	distanceBetween := b.Pos.Sub(a.Pos)

	radiusSum := circleB.Radius + circleA.Radius

	// Use squared versions to avoid call to sqrt(Mag()).
	isColliding = distanceBetween.MagSquared() <= (radiusSum * radiusSum)

	if !isColliding {
		return isColliding, contacts
	}

	// If we are using this function for a broad phase check,
	// we only need to populate the contact data if the two
	// underlying bodies are circles, since we can reuse the contact data.
	// Otherwise we can return early, and let the ensuing narrow
	// phase check populate the contact data for other shapes.
	if broadPhaseCheck && a.Circle != nil && b.Circle != nil {
		cPtr.Normal = distanceBetween.Norm()

		cPtr.Start = b.Pos.Sub(cPtr.Normal.Scale(circleB.Radius))
		cPtr.End = a.Pos.Add(cPtr.Normal.Scale(circleA.Radius))

		cPtr.Depth = cPtr.End.Sub(cPtr.Start).Mag()

		return isColliding, contacts
	}

	return isColliding, contacts
}

func detectPolygonCollision(a, b *tBokiComponents.RigidBody) (isColliding bool, contacts tBokiComponents.Contacts) {

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

	c.IncidentEdge = map[*tBokiComponents.RigidBody][]int{
		a: {incidentEdgeIndexA, (incidentEdgeIndexA + 1) % len(a.Polygon.WorldVertices)},
		b: {incidentEdgeIndexB, (incidentEdgeIndexB + 1) % len(b.Polygon.WorldVertices)},
	}

	contacts.Data = append(contacts.Data, c)

	return true, contacts
}

func detectPolygonCollisions(a, b *tBokiComponents.RigidBody) (isColliding bool, contacts tBokiComponents.Contacts) {
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

		contacts.Data = append(contacts.Data, c)
	}

	return true, contacts
}

func detectPolygonCircleCollision(polygonBody, circleBody *tBokiComponents.RigidBody, abSwapped bool) (isColliding bool, contacts tBokiComponents.Contacts) {
	// If abSwapped is true, we need to swap the normal and
	// start/end. This is because the normal is always calculated
	// from the polygon to the circle(vertToCircleCenter).
	defer func() {
		if !abSwapped {
			return
		}

		for i := range contacts.Data {
			contacts.Data[i].Normal = contacts.Data[i].Normal.Scale(-1)
			contacts.Data[i].Start, contacts.Data[i].End = contacts.Data[i].End, contacts.Data[i].Start
		}
	}()

	contacts.Data = append(contacts.Data, tBokiComponents.NewContact())
	cPtr := &contacts.Data[0]

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
