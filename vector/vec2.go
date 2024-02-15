package vector

import (
	"math"
)

type Vec2Face interface {
	X() float64
	Y() float64
	Scale(n float64) Vec2Face
	ScalarProduct(Vec2Face) float64
	CrossProduct(Vec2Face) float64
	Add(Vec2Face) Vec2Face
	Sub(Vec2Face) Vec2Face
	Perpendicular() Vec2Face
	Mag() float64
	MagSquared() float64
	Norm() Vec2Face
	Rotate(radians float64) Vec2Face
	RotateAroundPoint(radians float64, point Vec2Face) Vec2Face
	Equal(Vec2Face) bool
	Clone() Vec2Face
	Set(x, y float64)
}

type Vec2 struct {
	x, y float64
}

func NewVec2(x, y float64) *Vec2 {
	return &Vec2{x, y}
}

func (v2 Vec2) Scale(n float64) Vec2Face {

	x := v2.X() * n
	y := v2.Y() * n
	v2 = Vec2{x, y}

	return &v2
}

func (v2a Vec2) ScalarProduct(v2b Vec2Face) float64 {
	return (v2a.x * v2b.X()) + (v2a.y * v2b.Y())
}

func (v2a Vec2) CrossProduct(v2b Vec2Face) float64 {
	return (v2a.x * v2b.Y()) - (v2a.y * v2b.X())
}

func (v2a Vec2) Add(v2b Vec2Face) Vec2Face {
	result := Vec2{x: v2a.x + v2b.X(), y: v2a.y + v2b.Y()}
	return &result
}

func (v2a Vec2) Sub(v2b Vec2Face) Vec2Face {
	v2a.x -= v2b.X()
	v2a.y -= v2b.Y()
	return &v2a
}

func (v2 Vec2) Perpendicular() Vec2Face {
	x := v2.x
	y := v2.y

	v2.x = y
	v2.y = -x

	return &v2
}

func (v2 Vec2) Mag() float64 {
	return math.Sqrt((v2.x * v2.x) + (v2.y * v2.y))
}

func (v2 Vec2) MagSquared() float64 {
	return (v2.x * v2.x) + (v2.y * v2.y)
}

func (v2 Vec2) Norm() Vec2Face {
	len := v2.Mag()

	if len != 0 {
		v2.x = v2.x / len
		v2.y = v2.y / len
	}

	return &v2
}

func (v2 Vec2) Rotate(radians float64) Vec2Face {
	newX := v2.x*math.Cos(radians) - v2.y*math.Sin(radians)
	newY := v2.x*math.Sin(radians) + v2.y*math.Cos(radians)

	v2.x = newX
	v2.y = newY

	return &v2
}

func (v2 Vec2) RotateAroundPoint(radians float64, point Vec2Face) Vec2Face {
	origin := Vec2{x: 0, y: 0}
	offset := origin.Sub(point)

	result := v2.Add(offset)

	result = result.Rotate(radians)

	result = result.Sub(offset)

	return result

}

func (v2 Vec2) Equal(v2b Vec2Face) bool {
	return v2.x == v2b.X() && v2.y == v2b.Y()
}

func (v2 Vec2) Clone() Vec2Face {
	return &Vec2{x: v2.x, y: v2.y}
}

func (v2 Vec2) X() float64 {
	return v2.x
}

func (v2 Vec2) Y() float64 {
	return v2.y
}

func (v2 *Vec2) Set(x, y float64) {
	v2.x = x
	v2.y = y
}
