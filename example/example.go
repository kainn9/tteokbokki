package main

// Crappy example for testing refactor(WIP)!s
import (
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	ebitenDraw "github.com/hajimehoshi/ebiten/v2/vector"

	"github.com/kainn9/tteokbokki/physics/detector"
	entitysubset "github.com/kainn9/tteokbokki/physics/entity_subset"
	"github.com/kainn9/tteokbokki/physics/factory"
	"github.com/kainn9/tteokbokki/physics/physics"
	"github.com/kainn9/tteokbokki/physics/resolver"
	"github.com/kainn9/tteokbokki/vector"
)

type game struct{}

const (
	WIDTH          = 640
	HEIGHT         = 360
	WALL_THICKNESS = 30
)

var (

	// Particle.
	particlePosX = 100.0
	particlePosY = 100.0
	particleMass = 1.0

	particleTransform, particlePhysics = factory.Components.NewParticleComponents(
		particlePosX, particlePosY, particleMass,
	)

	particleEn = entitysubset.NewParticle(
		particleTransform, particlePhysics,
	)

	// Rectangle.
	rectPosX     = 300.0
	rectPosY     = 40.0
	rectMass     = 1.0
	rectWidth    = 50.0
	rectHeight   = 50.0
	rectRotation = 0.0

	rectTransform, rectShape, rectPhysics = factory.Components.NewRigidBodyRectangleComponents(
		rectPosX,
		rectPosY,
		rectWidth,
		rectHeight,
		rectMass,
		rectRotation,
	)

	rectEn = entitysubset.NewRigidBody(
		rectTransform, rectShape, rectPhysics,
	)

	// Hexagon
	hexagonPosX     = 100.0
	hexagonPosY     = 100.0
	hexagonMass     = 1.0
	hexagonRotation = 0.0
	hexagonSize     = 80.0

	hexagonTransform, hexagonShape, hexagonPhys = factory.Components.NewRigidBodyHexagonComponents(
		hexagonPosX,
		hexagonPosY,
		hexagonMass,
		hexagonRotation,
		hexagonSize,
	)

	hexagonEn = entitysubset.NewRigidBody(
		hexagonTransform, hexagonShape, hexagonPhys,
	)

	// Floor.
	floorRectPosX     = float64(WIDTH / 2)
	floorRectPosY     = float64(HEIGHT) - 50.0
	floorRectWidth    = float64(WIDTH)
	floorRectHeight   = 50.0
	floorRectMass     = 0.0
	floorRectRotation = 0.0

	floorRectTransform, floorRectShape, floorRectPhysics = factory.Components.NewRigidBodyRectangleComponents(
		floorRectPosX,
		floorRectPosY,
		floorRectWidth,
		floorRectHeight,
		floorRectMass,
		floorRectRotation,
	)

	floorRectEn = entitysubset.NewRigidBody(
		floorRectTransform, floorRectShape, floorRectPhysics,
	)
)

func (g *game) Layout(w, h int) (int, int) {
	return WIDTH, HEIGHT
}

func main() {

	rectEn.SetAndCalculateAngularMass(rectEn)
	rectEn.SetFriction(1)

	hexagonEn.SetAndCalculateAngularMass(hexagonEn)
	hexagonEn.SetAngularMass(0)
	hexagonEn.SetFriction(1)

	sim := NewSim()
	ebiten.RunGame(sim)
}

func NewSim() *game {
	g := &game{}

	ebiten.SetWindowTitle("Testing!")
	ebiten.SetWindowSize(WIDTH, HEIGHT)

	return g

}

func (g *game) Update() error {
	if ebiten.IsKeyPressed(ebiten.KeyRight) {
		physics.AddForce(rectEn, vector.NewVec2(20, 0))
	} else if ebiten.IsKeyPressed(ebiten.KeyLeft) {
		physics.AddForce(rectEn, vector.NewVec2(-20, 0))
	}

	physics.AddForce(particleEn, vector.NewVec2(0, 80))
	physics.AddForce(rectEn, vector.NewVec2(0, 20))
	physics.AddForce(hexagonEn, vector.NewVec2(0, 20))

	dt := 1.0 / 60.0

	physics.Integrate(particleEn, dt)
	physics.Integrate(rectEn, dt)
	physics.Integrate(hexagonEn, dt)
	physics.Integrate(floorRectEn, dt)

	hexagonEn.SetScale(hexagonEn.Scale().X()+0.001, hexagonEn.Scale().Y()+0.001)

	if isColliding, collision := detector.CheckCollision(
		rectEn,
		floorRectEn,
	); isColliding {
		resolver.HandleCollision(collision, rectEn, floorRectEn)
	}

	if isColliding, collision := detector.CheckCollision(
		hexagonEn,
		floorRectEn,
	); isColliding {
		resolver.HandleCollision(collision, hexagonEn, floorRectEn)
	}

	return nil
}

func (g *game) Draw(screen *ebiten.Image) {
	red := color.RGBA{255, 0, 0, 255}
	blue := color.RGBA{0, 0, 255, 255}
	yellow := color.RGBA{255, 255, 0, 255}

	drawParticle(screen, particleEn, red)
	drawParticle(screen, rectEn, red)
	drawParticle(screen, hexagonEn, red)

	drawPolygonShape(screen, rectEn, yellow)
	drawPolygonShape(screen, hexagonEn, yellow)
	drawPolygonShape(screen, floorRectEn, blue)

}

func drawParticle(screen *ebiten.Image, particle entitysubset.ParticleFace, color color.RGBA) {
	radius := float32(2)
	lineThickness := float32(1)
	antiAlias := false

	ebitenDraw.StrokeCircle(
		screen,
		float32(particle.Position().X()),
		float32(particle.Position().Y()),
		radius,
		lineThickness,
		color,
		antiAlias,
	)
}

func drawPolygonShape(screen *ebiten.Image, shape entitysubset.RigidBodyFace, color color.RGBA) {
	lineThickness := float32(1)
	antiAlias := false

	length := len(shape.Polygon().WorldVertices())

	for i := 0; i <= length-1; i++ {
		vert := (shape.Polygon().WorldVertices())[i]

		nextVertIdx := (i + 1) % length

		vert2 := (shape.Polygon().WorldVertices())[nextVertIdx]

		ebitenDraw.StrokeLine(
			screen,
			float32(vert.X()),
			float32(vert.Y()),
			float32(vert2.X()),
			float32(vert2.Y()),
			lineThickness,
			color,
			antiAlias,
		)
	}

}
