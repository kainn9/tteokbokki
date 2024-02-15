package transform_components

import (
	"math"

	"github.com/kainn9/tteokbokki/vector"
)

type ShapeFace interface {
	Circle() CircleFace
	SetCircle(radius float64)

	Polygon() PolygonFace
	SetPolygon(polygon PolygonFace)
	Area() float64
}
type Shape struct {
	circle  CircleFace
	polygon PolygonFace
}

type CircleFace interface {
	Radius() float64
	Area() float64
}
type Circle struct {
	radius float64
}

type PolygonFace interface {
	LocalVertices() []vector.Vec2Face

	WorldVertices() []vector.Vec2Face
	UpdateWorldVertices(trans TransformFace)

	AAB() AABFace
	SetAAB(width, height float64)

	Edge(index int) (edge, v1, v2 vector.Vec2Face)

	Area() float64
	Centroid() vector.Vec2Face

	CircleSkin() CircleFace
	CalculateAndSetCircleSkin()
}

type Polygon struct {
	localVertices []vector.Vec2Face
	worldVertices []vector.Vec2Face
	aab           AABFace
	circleSkin    CircleFace
}

// Axis-Aligned Bounding Box (Rectangle with no rotation)
type AABFace interface {
	Width() float64
	Height() float64

	ScaledWidth(t TransformFace) float64
	ScaledHeight(t TransformFace) float64
}
type AAB struct {
	width  float64
	height float64
}

// Factory Methods.
func NewCircleShape(radius float64) ShapeFace {
	return &Shape{
		circle: newCircle(radius),
	}
}

func newCircle(radius float64) CircleFace {
	return &Circle{
		radius: radius,
	}
}

func NewPolygonShape(localVertices []vector.Vec2Face) ShapeFace {
	return &Shape{
		polygon: newPolygon(localVertices),
	}
}

func newPolygon(localVertices []vector.Vec2Face) PolygonFace {
	return &Polygon{
		localVertices: localVertices,
		worldVertices: make([]vector.Vec2Face, len(localVertices)),
	}
}

func NewPolygonRectangleShape(width, height float64) ShapeFace {
	shape := NewPolygonShape(make([]vector.Vec2Face, 4))

	shape.Polygon().SetAAB(width, height)

	return shape
}

func newAAB(width, height float64) AABFace {
	return &AAB{
		width:  width,
		height: height,
	}
}

// Shape Methods.
func (shape Shape) Circle() CircleFace {
	return shape.circle
}

func (shape *Shape) SetCircle(radius float64) {
	if shape.polygon != nil {
		shape.polygon = nil
	}

	shape.circle = newCircle(radius)
}

func (shape Shape) Polygon() PolygonFace {
	return shape.polygon
}

func (shape *Shape) SetPolygon(polygon PolygonFace) {
	if shape.circle != nil {
		shape.circle = nil
	}

	shape.polygon = polygon
}

func (shape Shape) Area() float64 {
	circle := shape.Circle()
	poly := shape.Polygon()

	if circle != nil {
		return circle.Area()
	}

	if poly != nil {
		return poly.Area()
	}

	return 0

}

// Circle Methods.
func (circle Circle) Radius() float64 {
	return circle.radius
}

func (circle Circle) Area() float64 {
	return math.Pi * circle.radius * circle.radius
}

// Circles only scale with 1 dimension so we use X.
func (circle Circle) ScaledRadius(t TransformFace) float64 {
	return circle.radius * t.Scale().X()
}

// Polygon Methods.
func (polygon Polygon) LocalVertices() []vector.Vec2Face {
	return polygon.localVertices
}

func (polygon Polygon) WorldVertices() []vector.Vec2Face {
	return polygon.worldVertices
}

func (polygon *Polygon) UpdateWorldVertices(trans TransformFace) {
	for i := 0; i < len(polygon.localVertices); i++ {
		// Scale
		scaled := polygon.localVertices[i].Clone()
		scaled.Set(
			scaled.X()*trans.Scale().X(),
			scaled.Y()*trans.Scale().Y(),
		)

		// Rotate
		rotated := scaled.Rotate(trans.Rotation())

		// Translate
		translated := rotated.Add(trans.Position())

		polygon.worldVertices[i] = translated

		if trans.Rotation() != 0 {
			polygon.aab = nil // Invalidate AABB if rotation applied
		}

	}
}

func (polygon Polygon) AAB() AABFace {
	return polygon.aab
}

func (polygon *Polygon) SetAAB(width, height float64) {
	newRect := newAAB(width, height)

	newLocalVertices := []vector.Vec2Face{
		vector.NewVec2(-width/2, -height/2),
		vector.NewVec2(width/2, -height/2),
		vector.NewVec2(width/2, height/2),
		vector.NewVec2(-width/2, height/2),
	}

	polygon.localVertices = newLocalVertices

	polygon.aab = newRect
}

func (polygon Polygon) Edge(index int) (edge, v1, v2 vector.Vec2Face) {

	nextIndex := (index + 1) % len(polygon.worldVertices)

	vb := (polygon.worldVertices)[nextIndex]
	va := (polygon.worldVertices)[index]

	return vb.Sub(va), va, vb
}

func (p Polygon) Area() float64 {
	// Get local vertices of the polygon
	localVertices := p.LocalVertices()

	// Initialize the area of the polygon
	area := 0.0

	// Iterate over each pair of consecutive vertices
	for i := 0; i < len(localVertices); i++ {
		// Calculate the cross product of consecutive vertices
		j := (i + 1) % len(localVertices)
		area += localVertices[i].CrossProduct(localVertices[j])
	}

	// Calculate and return the area of the polygon
	return area / 2.0
}

func (p Polygon) Centroid() vector.Vec2Face {
	// Get local vertices of the polygon
	localVertices := p.LocalVertices()

	// Initialize the centroid of the polygon
	centroid := localVertices[0].Clone()
	centroid.Set(0, 0)

	// Iterate over each pair of consecutive vertices
	for i := 0; i < len(localVertices); i++ {
		// Calculate the indices of the consecutive vertices
		j := (i + 1) % len(localVertices)

		// Calculate the cross product of consecutive vertices
		cross := localVertices[i].CrossProduct(localVertices[j])

		// Calculate the sum of the vectors scaled by the cross product
		sum := localVertices[i].Add(localVertices[j]).Scale(cross)

		// Update the centroid by adding the scaled sum
		centroid = centroid.Add(sum)
	}

	// Scale the centroid by the reciprocal of
	// 6 times the polygon's area
	return centroid.Scale(1.0 / 6.0 / p.Area())
}

func (p Polygon) CircleSkin() CircleFace {
	if p.circleSkin == nil {
		p.CalculateAndSetCircleSkin()
	}

	return p.circleSkin
}

func (p *Polygon) CalculateAndSetCircleSkin() {

	var longestDistanceFromCenter float64

	for _, vert := range p.LocalVertices() {
		distFromCenter := vert.Sub(vector.NewVec2(0, 0)).Mag()
		longestDistanceFromCenter = math.Max(longestDistanceFromCenter, distFromCenter)
	}

	p.circleSkin = newCircle(longestDistanceFromCenter)

}

// Rectangle Methods.
func (rectangle AAB) Width() float64 {
	return rectangle.width
}

func (rectangle AAB) Height() float64 {
	return rectangle.height
}

func (rectangle AAB) ScaledWidth(t TransformFace) float64 {
	return rectangle.width * t.Scale().X()
}

func (rectangle AAB) ScaledHeight(t TransformFace) float64 {
	return rectangle.height * t.Scale().Y()
}
