package collision

import (
	"math"

	"github.com/kainn9/tteokbokki/components"
	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/math/vec"
)

// Check performs collision detection between two rigid bodies.
// Returns true if the two rigid bodies are colliding, and a contact object.
func (CollisionChecker) Check(a, b *componentsPhysicsTypes.RigidBody) (bool, componentsPhysicsTypes.Contact) {
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

	c := components.Physics.NewContact()
	return false, c
}

// FindMinSep finds the minimum separation between two rigid bodies.
// Requires vertices and will not work with circles.
func FindMinSep(a, b componentsPhysicsTypes.RigidBody) (minSep float64, axisNormal, penPoint vec.Vec2) {

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
			axisNormal = normal
			penPoint = minVert
		}
	}

	return sep, axisNormal, penPoint
}

// GetEdge returns the edge, and the two vertices that make up the edge.
func GetEdge(index int, rb componentsPhysicsTypes.RigidBody) (edge, v1, v2 vec.Vec2) {

	nextIndex := (index + 1) % len(rb.Polygon.WorldVertices)

	vb := (rb.Polygon.WorldVertices)[nextIndex]
	va := (rb.Polygon.WorldVertices)[index]

	return vb.Sub(va), va, vb
}

func checkCircleCollision(a, b *componentsPhysicsTypes.RigidBody) (isColliding bool, c componentsPhysicsTypes.Contact) {

	c = components.Physics.NewContact()

	distanceBetween := b.Pos.Sub(a.Pos)

	radiusSum := b.Circle.Radius + a.Circle.Radius

	// use squared versions to avoid call to sqrt
	isColliding = distanceBetween.MagSquared() <= (radiusSum * radiusSum)

	if !isColliding {
		return isColliding, c
	}

	c.A = a
	c.B = b
	c.Normal = distanceBetween.Norm()

	c.Start = b.Pos.Sub(c.Normal.Scale(b.Circle.Radius))
	c.End = a.Pos.Add(c.Normal.Scale(a.Circle.Radius))

	c.Depth = c.End.Sub(c.Start).Mag()

	return isColliding, c

}

func checkPolygonCollision(a, b *componentsPhysicsTypes.RigidBody) (isColliding bool, c componentsPhysicsTypes.Contact) {
	c = components.Physics.NewContact()
	minSepA, aAxisNormal, aPoint := FindMinSep(*a, *b)

	if minSepA >= 0 {
		return false, c
	}

	minSepB, bAxisNormal, bPoint := FindMinSep(*b, *a)
	if minSepB >= 0 {
		return false, c
	}

	c.A = a
	c.B = b

	if minSepA > minSepB {
		c.Depth = -minSepA
		c.Normal = aAxisNormal
		c.Start = aPoint
		c.End = c.Start.Add(c.Normal.Scale(c.Depth))

	} else {

		c.Depth = -minSepB
		c.Normal = bAxisNormal.Scale(-1)
		c.Start = bPoint.Sub(c.Normal.Scale(c.Depth))
		c.End = bPoint
	}

	return true, c
}

func checkPolygonCircleCollision(polygonBody, circleBody *componentsPhysicsTypes.RigidBody) (isColliding bool, c componentsPhysicsTypes.Contact) {
	c = components.Physics.NewContact()

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
				return false, c
			} else {
				c.A = polygonBody
				c.B = circleBody
				c.Depth = circleBody.Circle.Radius - mag
				c.Normal = vertToCircleCenter.Norm()
				c.Start = circleBody.Pos.Add(c.Normal.Scale(-circleBody.Circle.Radius))
				c.End = c.Start.Add(c.Normal.Scale(c.Depth))
				return true, c
			}

		}

		// right of edge bounds
		vertToCircleCenter = circleBody.Pos.Sub(minNextVert)
		nearestEdge = minCurrVert.Sub(minNextVert)

		circleCenterRightOfEdge := vertToCircleCenter.ScalarProduct(nearestEdge) < 0

		if circleCenterRightOfEdge {
			mag := vertToCircleCenter.Mag()

			if mag > circleBody.Circle.Radius {
				return false, c
			} else {
				c.A = polygonBody
				c.B = circleBody
				c.Depth = circleBody.Circle.Radius - mag
				c.Normal = vertToCircleCenter.Norm()
				c.Start = circleBody.Pos.Add(c.Normal.Scale(-circleBody.Circle.Radius))
				c.End = c.Start.Add(c.Normal.Scale(c.Depth))
				return true, c
			}

		}

		// circle center is within the edge bounds
		if distCircleEdge > circleBody.Circle.Radius {
			return false, c
		}

		c.A = polygonBody
		c.B = circleBody
		c.Depth = circleBody.Circle.Radius - distCircleEdge
		c.Normal = minNextVert.Sub(minCurrVert).Perpendicular().Norm()

		c.Start = circleBody.Pos.Sub(c.Normal.Scale(circleBody.Circle.Radius))
		c.End = c.Start.Add(c.Normal.Scale(c.Depth))

		return true, c

	}

	// if circle center is inside the polygon
	// we are definitely colliding
	c.A = polygonBody
	c.B = circleBody
	c.Depth = circleBody.Circle.Radius - distCircleEdge
	c.Normal = minNextVert.Sub(minCurrVert).Perpendicular().Norm()
	c.Start = circleBody.Pos.Sub(c.Normal.Scale(circleBody.Circle.Radius))
	c.End = c.Start.Add(c.Normal.Scale(c.Depth))
	return true, c
}
