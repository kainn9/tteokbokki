package tBokiVec

import "math"

type Vec2 struct {
	X, Y float64
}

func (v2 Vec2) Scale(n float64) Vec2 {
	v2.X *= n
	v2.Y *= n

	return v2
}

func (v2a Vec2) ScalarProduct(v2b Vec2) float64 {
	return (v2a.X * v2b.X) + (v2a.Y * v2b.Y)
}

func (v2a Vec2) CrossProduct(v2b Vec2) float64 {
	return (v2a.X * v2b.Y) - (v2a.Y * v2b.X)
}

func (v2a Vec2) Add(v2b Vec2) Vec2 {
	v2a.X += v2b.X
	v2a.Y += v2b.Y
	return v2a
}

func (v2a Vec2) Sub(v2b Vec2) Vec2 {
	v2a.X -= v2b.X
	v2a.Y -= v2b.Y
	return v2a
}

func (v2 Vec2) Perpendicular() Vec2 {
	x := v2.X
	y := v2.Y

	v2.X = y
	v2.Y = -x
	return v2
}

func (v2 Vec2) Mag() float64 {
	return math.Sqrt((v2.X * v2.X) + (v2.Y * v2.Y))
}

func (v2 Vec2) MagSquared() float64 {
	return (v2.X * v2.X) + (v2.Y * v2.Y)
}

func (v2 Vec2) Norm() Vec2 {
	len := v2.Mag()

	if len != 0 {
		v2.X = v2.X / len
		v2.Y = v2.Y / len
	}

	return v2
}

func (v2 Vec2) Rotate(radians float64) Vec2 {
	newX := v2.X*math.Cos(radians) - v2.Y*math.Sin(radians)
	newY := v2.X*math.Sin(radians) + v2.Y*math.Cos(radians)

	v2.X = newX
	v2.Y = newY
	return v2
}

func (v2 Vec2) RotateAroundPoint(radians float64, point Vec2) Vec2 {
	origin := Vec2{X: 0, Y: 0}

	offset := origin.Sub(point)

	v2 = v2.Add(offset)

	v2 = v2.Rotate(radians)

	v2 = v2.Sub(offset)

	return v2

}

func (v2 Vec2) Compare(v2b Vec2) bool {
	return v2.X == v2b.X && v2.Y == v2b.Y
}
