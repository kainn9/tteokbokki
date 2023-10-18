package renderSystems

import (
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"

	tBokiComponents "github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/debugRender"
	"github.com/kainn9/tteokbokki/example/components"
	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type ShapesRenderer struct{}

func NewShapesRenderer() ShapesRenderer {
	return ShapesRenderer{}
}

func (ShapesRenderer) Query() *donburi.Query {

	return donburi.NewQuery(
		filter.Or(
			filter.Contains(components.RigidBodyComponent),
			filter.Contains(components.RigidBodyComponents),
		),
	)
}

func (ShapesRenderer) Draw(screen *ebiten.Image, entity *donburi.Entry) {
	red := color.RGBA{R: 255, G: 0, B: 0, A: 255}

	// When the entity has a single RigidBodyComponent.
	if entity.HasComponent(components.RigidBodyComponent) {
		body := components.RigidBodyComponent.Get(entity)
		drawHelper(body, screen, red)
	}

	// When the entity has multiple RigidBodyComponents.
	if entity.HasComponent(components.RigidBodyComponents) {
		bodies := components.RigidBodyComponents.Get(entity)

		for _, body := range *bodies {
			drawHelper(body, screen, red)
		}
	}
}

func drawHelper(body *tBokiComponents.RigidBody, screen *ebiten.Image, red color.RGBA) {

	if body.Circle != nil {
		debugRender.DrawCircleBody(screen, *body, red)

	} else {
		debugRender.DrawPolygonBody(screen, *body, red)
	}
}
