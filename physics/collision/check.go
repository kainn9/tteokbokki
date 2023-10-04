package collision

import (
	"math"

	"github.com/kainn9/tteokbokki/components"
	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/math/vec"
)

// Check performs collision detection between two rigid bodies.
// Returns true if the two rigid bodies are colliding, and a contact object.
func (CollisionChecker) Check(a, b *componentsPhysicsTypes.RigidBody) (bool, []componentsPhysicsTypes.Contact) {
	if a.Circle != nil && b.Circle != nil {
		return checkCircleCollision(a, b)
	}

	if a.Polygon != nil && b.Polygon != nil {
		return checkPolygonCollision(a, b)
	}

	if a.Polygon != nil && b.Circle != nil {
		return checkPolygonCircleCollision(a, b)
	}

	if a.Circle != nil && b.Polygon != nil {
		return checkPolygonCircleCollision(b, a)
	}

	return false, make([]componentsPhysicsTypes.Contact, 0)
}

// FindMinSep finds the minimum separation between two rigid bodies.
// Requires vertices and will not work with circles.
// Note: reference edge is the edge with the greatest penetration.
func FindMinSep(a, b componentsPhysicsTypes.RigidBody) (minSep float64, indexReferenceEdge int, penPoint vec.Vec2) {

	sep := -math.MaxFloat64

	for i, va := range a.Polygon.WorldVertices {

		currentEdge, _, _ := GetEdge(i, a)
		normal := currentEdge.Perpendicular().Norm()

		minSep = math.MaxFloat64
		minVert := vec.Vec2{}

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
func FindIncidentEdgeIndex(incidentBody *componentsPhysicsTypes.RigidBody, refEdgeNormal vec.Vec2) int {
	var incidentEdgeIndex int
	minProjection := math.MaxFloat64

	for i := range incidentBody.Polygon.WorldVertices {
		edgeNormal, _, _ := GetEdge(i, *incidentBody)
		edgeNormal = edgeNormal.Perpendicular().Norm()

		projection := edgeNormal.ScalarProduct(refEdgeNormal)

		if projection < minProjection {
			minProjection = projection
			incidentEdgeIndex = i
		}
	}

	return incidentEdgeIndex
}

// GetEdge returns the edge, and the two vertices that make up the edge.
func GetEdge(index int, rb componentsPhysicsTypes.RigidBody) (edge, v1, v2 vec.Vec2) {

	nextIndex := (index + 1) % len(rb.Polygon.WorldVertices)

	vb := (rb.Polygon.WorldVertices)[nextIndex]
	va := (rb.Polygon.WorldVertices)[index]

	return vb.Sub(va), va, vb
}

func checkCircleCollision(a, b *componentsPhysicsTypes.RigidBody) (isColliding bool, contacts []componentsPhysicsTypes.Contact) {

	contacts = append(contacts, components.Physics.NewContact())
	cPtr := &contacts[0]

	distanceBetween := b.Pos.Sub(a.Pos)

	radiusSum := b.Circle.Radius + a.Circle.Radius

	// Use squared versions to avoid call to sqrt(Mag()).
	isColliding = distanceBetween.MagSquared() <= (radiusSum * radiusSum)

	if !isColliding {
		return isColliding, contacts
	}

	cPtr.A = a
	cPtr.B = b
	cPtr.Normal = distanceBetween.Norm()

	cPtr.Start = b.Pos.Sub(cPtr.Normal.Scale(b.Circle.Radius))
	cPtr.End = a.Pos.Add(cPtr.Normal.Scale(a.Circle.Radius))

	cPtr.Depth = cPtr.End.Sub(cPtr.Start).Mag()

	return isColliding, contacts

}

func checkPolygonCollision(a, b *componentsPhysicsTypes.RigidBody) (isColliding bool, contacts []componentsPhysicsTypes.Contact) {
	// Reference edge is the edge with the greatest penetration.
	// The incident edge the edge from the penetrating body that has
	// the normal most unaligned with the reference edge normal.
	minSepA, aIndexReferenceEdge, _ := FindMinSep(*a, *b)

	// Positive separation means no collision(no penetration).
	if minSepA >= 0 {
		return false, contacts
	}
	minSepB, bIndexReferenceEdge, _ := FindMinSep(*b, *a)
	if minSepB >= 0 {
		return false, contacts
	}

	// Determine which body is the reference body
	// and which is the incident body, based on the
	// minimum separation.
	var refBody *componentsPhysicsTypes.RigidBody
	var incidentBody *componentsPhysicsTypes.RigidBody
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

	referenceEdge, _, _ := GetEdge(indexReferenceEdge, *refBody)
	referenceEdgeNormal := referenceEdge.Perpendicular().Norm()

	incidentEdgeIndex := FindIncidentEdgeIndex(incidentBody, referenceEdgeNormal)
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
	contactPoints := make([]vec.Vec2, 2)
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

		numClipped, clippedPoints := clipSegmentToLine(contactPoints, clipPlane0Vert, clipPlane1Vert)

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

		c := components.Physics.NewContact()
		c.A = a
		c.B = b
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

func checkPolygonCircleCollision(polygonBody, circleBody *componentsPhysicsTypes.RigidBody) (isColliding bool, contacts []componentsPhysicsTypes.Contact) {
	contacts = append(contacts, components.Physics.NewContact())
	cPtr := &contacts[0]

	// getting the nearest edge/vertices to the circle center
	circleCenterOutside := false
	var minCurrVert, minNextVert vec.Vec2
	distCircleEdge := -math.MaxFloat64

	for i, currVert := range polygonBody.Polygon.WorldVertices {

		edge, _, nextVert := GetEdge(i, *polygonBody)
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
				cPtr.A = polygonBody
				cPtr.B = circleBody
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
				cPtr.A = polygonBody
				cPtr.B = circleBody
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

		cPtr.A = polygonBody
		cPtr.B = circleBody
		cPtr.Depth = circleBody.Circle.Radius - distCircleEdge
		cPtr.Normal = minNextVert.Sub(minCurrVert).Perpendicular().Norm()

		cPtr.Start = circleBody.Pos.Sub(cPtr.Normal.Scale(circleBody.Circle.Radius))
		cPtr.End = cPtr.Start.Add(cPtr.Normal.Scale(cPtr.Depth))

		return true, contacts

	}

	// If circle center is inside the polygon
	// we are definitely colliding
	cPtr.A = polygonBody
	cPtr.B = circleBody
	cPtr.Depth = circleBody.Circle.Radius - distCircleEdge
	cPtr.Normal = minNextVert.Sub(minCurrVert).Perpendicular().Norm()
	cPtr.Start = circleBody.Pos.Sub(cPtr.Normal.Scale(circleBody.Circle.Radius))
	cPtr.End = cPtr.Start.Add(cPtr.Normal.Scale(cPtr.Depth))
	return true, contacts
}

func clipSegmentToLine(contactPoints []vec.Vec2, c0, c1 vec.Vec2) (count int, contactsOut []vec.Vec2) {
	contactsOut = make([]vec.Vec2, 2)

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
