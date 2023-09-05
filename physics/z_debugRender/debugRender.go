package debugRender

// Intended to be used for debugging purposes in order to draw
// the shape of the RigidBodies.
// This is a non-essential package and needs to be imported
// separately from the physics package.
// Also depends on the ebiten/v2.

import (
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/math/vec"
)

func DrawCircleBody(screen *ebiten.Image, rb componentsPhysicsTypes.RigidBody, color color.RGBA) {

	vector.StrokeCircle(screen, float32(rb.Pos.X), float32(rb.Pos.Y), float32(rb.Circle.Radius), 1.0, color, false)

	endpoint := vec.Vec2{
		X: rb.Pos.X + rb.Circle.Radius,
		Y: rb.Pos.Y,
	}

	endpoint = endpoint.RotateAroundPoint(rb.Rotation, rb.Pos)

	vector.StrokeLine(screen, float32(rb.Pos.X), float32(rb.Pos.Y), float32(endpoint.X), float32(endpoint.Y), 1.0, color, false)
}

func DrawPolygonBody(screen *ebiten.Image, rb componentsPhysicsTypes.RigidBody, color color.RGBA) {

	length := len(rb.Polygon.WorldVertices)

	for i := 0; i <= length-1; i++ {
		vert := (rb.Polygon.WorldVertices)[i]

		nextVertIdx := (i + 1) % length

		vert2 := (rb.Polygon.WorldVertices)[nextVertIdx]

		vector.StrokeLine(screen, float32(vert.X), float32(vert.Y), float32(vert2.X), float32(vert2.Y), 1.0, color, false)
	}

	vector.StrokeCircle(screen, float32(rb.Pos.X), float32(rb.Pos.Y), 4, 2.0, color, false)

}
