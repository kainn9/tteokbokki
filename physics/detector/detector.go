package detector

import (
	"math"

	physics_components "github.com/kainn9/tteokbokki/physics/components"
	entitysubset "github.com/kainn9/tteokbokki/physics/entity_subset"
	transform_components "github.com/kainn9/tteokbokki/transform/components"

	"github.com/kainn9/tteokbokki/vector"
)

// Fix for shape sub
func CheckAAB_Multi(
	transA, transB transform_components.TransformFace,
	shapeA, shapeB transform_components.ShapeFace,
) bool {

	bodyA := entitysubset.NewRigidBody(transA, shapeA, nil)
	bodyB := entitysubset.NewRigidBody(transB, shapeB, nil)

	return CheckAAB(bodyA, bodyB)
}

func CheckAAB(bodyA, bodyB entitysubset.RigidBodyFace) bool {
	aabA := bodyA.Polygon().AAB()
	aabB := bodyB.Polygon().AAB()

	// Calculate half-width and half-height for each AABB
	halfWidthA := aabA.ScaledWidth(bodyA) / 2
	halfHeightA := aabA.ScaledHeight(bodyA) / 2
	halfWidthB := aabB.ScaledWidth(bodyB) / 2
	halfHeightB := aabB.ScaledHeight(bodyB) / 2

	// Calculate the left, right, top, and bottom coordinates of each AABB
	xA := bodyA.Position().X()
	yA := bodyA.Position().Y()
	xB := bodyB.Position().X()
	yB := bodyB.Position().Y()

	leftA := xA - halfWidthA
	rightA := xA + halfWidthA
	topA := yA - halfHeightA
	bottomA := yA + halfHeightA

	leftB := xB - halfWidthB
	rightB := xB + halfWidthB
	topB := yB - halfHeightB
	bottomB := yB + halfHeightB

	// Check if the bounding boxes of the two entities intersect along each axis.
	if rightA < leftB || leftA > rightB {
		return false // No intersection along the X-axis.
	}
	if bottomA < topB || topA > bottomB {
		return false // No intersection along the Y-axis.
	}
	// Add more checks for other axes if needed.

	return true // Bounding boxes intersect along all axes, potential collision.
}

func CheckCollision_Multi(
	transA, transB transform_components.TransformFace,
	shapeA, shapeB transform_components.ShapeFace,
	physA, physB physics_components.PhysicsFace,
) (
	isColliding bool,
	collision *physics_components.Collision,
) {
	bodyA := entitysubset.NewRigidBody(
		transA,
		shapeA,
		physA,
	)

	bodyB := entitysubset.NewRigidBody(
		transB,
		shapeB,
		physB,
	)

	return CheckCollision(bodyA, bodyB)
}

func CheckCollision(bodyA, bodyB entitysubset.RigidBodyFace) (
	isColliding bool,
	collision *physics_components.Collision,
) {
	if isCircleCollision(bodyA, bodyB) {
		return checkCircleCollision(
			bodyA.Circle(),
			bodyB.Circle(),
			bodyA.Position(),
			bodyB.Position(),
			false,
		)
	}

	ok, aIsPoly, bIsPoly := isCirclePolygonCollision(bodyA, bodyB)

	if ok && aIsPoly {
		if !broadPhasePolyCircle(bodyA, bodyB) {
			return false, nil
		}
		return checkPolygonCircleCollision(bodyA, bodyB, false)
	}

	if ok && bIsPoly {
		if !broadPhasePolyCircle(bodyB, bodyA) {
			return false, nil
		}
		return checkPolygonCircleCollision(bodyB, bodyA, true)
	}

	if isPolygonCollision(bodyA, bodyB) {
		if !broadPhasePoly(bodyB, bodyA) {
			return false, nil
		}
		return checkPolygonCollision(bodyA.Polygon(), bodyB.Polygon())
	}

	return false, nil
}

func isCircleCollision(shapeA, shapeB transform_components.ShapeFace) bool {
	return shapeA.Circle() != nil && shapeB.Circle() != nil
}

func isCirclePolygonCollision(shapeA, shapeB transform_components.ShapeFace) (
	isCirclePolygonCollision, aIsPolygon, bIsPolygon bool,
) {
	isCirclePolygonCollision = shapeA.Circle() != nil && shapeB.Polygon() != nil
	aIsPolygon = shapeA.Polygon() != nil
	bIsPolygon = shapeB.Polygon() != nil

	return isCirclePolygonCollision, aIsPolygon, bIsPolygon
}

func isPolygonCollision(shapeA, shapeB transform_components.ShapeFace) bool {
	return shapeA.Polygon() != nil && shapeB.Polygon() != nil
}

func checkCircleCollision(
	circleA, circleB transform_components.CircleFace,
	posA, posB vector.Vec2Face,
	// No need to save  if only being used for broad phase detection.
	skipCollisionData bool,
) (
	isColliding bool,
	collision *physics_components.Collision,

) {
	distanceBetween := posB.Sub(posA)

	radiusSum := circleB.Radius() + circleA.Radius()

	// Use squared versions to avoid call to sqrt(Mag()).
	notColliding := distanceBetween.MagSquared() > (radiusSum * radiusSum)

	if notColliding {
		return false, nil
	}

	if skipCollisionData {
		return true, nil
	}

	normal := distanceBetween.Norm()
	start := posB.Sub(normal.Scale(circleB.Radius()))
	end := posA.Add(normal.Scale(circleA.Radius()))
	depth := end.Sub(start).Mag()

	collision = physics_components.NewCollision(start, end, normal, depth)

	return true, collision
}

func checkPolygonCircleCollision(
	polygonBody, circleBody entitysubset.RigidBodyFace,
	swap bool,
) (
	isColliding bool,
	collision *physics_components.Collision,
) {
	defer func() {
		if !swap {
			return
		}

		collision.Normal = collision.Normal.Scale(-1)
		collision.Start, collision.End = collision.End, collision.Start
	}()

	// Determine the nearest edge/vertices to the circle center.
	circleCenterOutside := false
	var minCurrVert, minNextVert vector.Vec2Face
	distCircleEdge := -math.MaxFloat64

	polygon := polygonBody.Polygon()
	circle := circleBody.Circle()

	for i, currVert := range polygon.WorldVertices() {

		edge, _, nextVert := polygon.Edge(i)
		normal := edge.Perpendicular().Norm()

		vertToCircleCenter := circleBody.Position().Sub(currVert)
		penCheck := vertToCircleCenter.ScalarProduct(normal)

		if penCheck > 0 {

			distCircleEdge = penCheck
			minCurrVert = currVert
			minNextVert = nextVert

			circleCenterOutside = true
			break

		} else if penCheck > distCircleEdge {

			distCircleEdge = penCheck
			minCurrVert = currVert
			minNextVert = nextVert
		}

	}

	if circleCenterOutside {
		vertToCircleCenter := circleBody.Position().Sub(minCurrVert)
		nearestEdge := minNextVert.Sub(minCurrVert)
		circleCenterLeftOfEdge := vertToCircleCenter.ScalarProduct(nearestEdge) < 0

		if circleCenterLeftOfEdge {
			mag := vertToCircleCenter.Mag()

			if mag > circle.Radius() {
				return false, nil
			} else {

				depth := circle.Radius() - mag
				normal := vertToCircleCenter.Norm()
				start := circleBody.Position().Add(
					collision.Normal.Scale(-circle.Radius()),
				)
				end := start.Add(normal.Scale(depth))

				collision = physics_components.NewCollision(start, end, normal, depth)

				return true, collision
			}

		}

		vertToCircleCenter = circleBody.Position().Sub(minNextVert)
		nearestEdge = minCurrVert.Sub(minNextVert)

		circleCenterRightOfEdge := vertToCircleCenter.ScalarProduct(nearestEdge) < 0

		if circleCenterRightOfEdge {
			mag := vertToCircleCenter.Mag()

			if mag > circle.Radius() {
				return false, nil
			} else {

				depth := circle.Radius() - mag
				normal := vertToCircleCenter.Norm()
				start := circleBody.Position().Add(
					normal.Scale(-circle.Radius()),
				)
				end := start.Add(normal.Scale(depth))

				collision = physics_components.NewCollision(start, end, normal, depth)

				return true, collision
			}

		}

		if distCircleEdge > circle.Radius() {
			return false, nil
		}

		depth := circle.Radius() - distCircleEdge
		normal := minNextVert.Sub(minCurrVert).Perpendicular().Norm()
		start := circleBody.Position().Sub(
			normal.Scale(circle.Radius()),
		)
		end := start.Add(normal.Scale(depth))

		collision = physics_components.NewCollision(start, end, normal, depth)

		return true, collision
	}

	depth := circle.Radius() - distCircleEdge
	normal := minNextVert.Sub(minCurrVert).Perpendicular().Norm()
	start := circleBody.Position().Sub(
		normal.Scale(circle.Radius()),
	)
	end := start.Add(normal.Scale(depth))

	collision = physics_components.NewCollision(start, end, normal, depth)

	return true, collision
}

func checkPolygonCollision(polygonA, polygonB transform_components.PolygonFace) (
	isColliding bool,
	collision *physics_components.Collision,
) {
	minSepA, incidentEdgeIndexA, penPointA := findMinSep(polygonA, polygonB)
	incidentEdgeA, _, _ := polygonA.Edge(incidentEdgeIndexA)

	if minSepA >= 0 {
		return false, nil
	}

	minSepB, incidentEdgeIndexB, penPointB := findMinSep(polygonB, polygonA)
	incidentEdgeB, _, _ := polygonB.Edge(incidentEdgeIndexB)

	if minSepB >= 0 {
		return false, nil
	}

	if minSepA > minSepB {
		depth := -minSepA
		normal := incidentEdgeA.Perpendicular().Norm()
		start := penPointA
		end := start.Add(normal.Scale(depth))

		collision = physics_components.NewCollision(start, end, normal, depth)
	} else {
		depth := -minSepB
		normal := incidentEdgeB.Perpendicular().Norm().Scale(-1)
		start := penPointB.Sub(normal.Scale(depth))
		end := penPointB

		collision = physics_components.NewCollision(start, end, normal, depth)
	}

	return true, collision
}

func findMinSep(
	polygonA, polygonB transform_components.PolygonFace,
) (
	sep float64,
	indexReferenceEdge int,
	penPoint vector.Vec2Face,
) {
	sep = -math.MaxFloat64

	for i, va := range polygonA.WorldVertices() {

		currentEdge, _, _ := polygonA.Edge(i)
		normal := currentEdge.Perpendicular().Norm()

		minSep := math.MaxFloat64

		minVert := polygonA.LocalVertices()[0].Clone()
		minVert.Set(0, 0)

		for _, vb := range polygonB.WorldVertices() {

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

func broadPhasePolyCircle(polyBody, circleBody entitysubset.RigidBodyFace) bool {
	check, _ := checkCircleCollision(
		polyBody.Polygon().CircleSkin(),
		circleBody.Circle(),
		polyBody.Position(),
		circleBody.Position(),
		true,
	)

	return check
}
func broadPhasePoly(bodyA, bodyB entitysubset.RigidBodyFace) bool {
	if bodyA.Polygon().AAB() != nil && bodyB.Polygon().AAB() != nil {
		check := CheckAAB(bodyA, bodyB)
		return check
	}

	check, _ := checkCircleCollision(
		bodyA.Polygon().CircleSkin(),
		bodyB.Polygon().CircleSkin(),
		bodyA.Position(),
		bodyB.Position(),
		true,
	)

	return check
}
