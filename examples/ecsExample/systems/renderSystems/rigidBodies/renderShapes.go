package rigidBodiesRenderSystems

import (
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"

	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/examples/ecsExample/globals"
	debugRender "github.com/kainn9/tteokbokki/physics/z_debugRender"
	"github.com/yohamta/donburi"
	"github.com/yohamta/donburi/filter"
)

type RenderShapes struct{}

func (RigidBodiesStruct) NewRenderShapesSystem() RenderShapes {
	return RenderShapes{}
}

func (RenderShapes) Query() *donburi.Query {

	return donburi.NewQuery(
		filter.Or(
			filter.Contains(globals.RigidBodyComponent),
			filter.Contains(globals.RigidBodyComponents),
		),
	)
}

func (RenderShapes) Draw(screen *ebiten.Image, entry *donburi.Entry) {

	if entry.HasComponent(globals.RigidBodyComponents) {
		bodies := globals.RigidBodyComponents.Get(entry)

		for _, body := range *bodies {
			drawHelper(screen, body)
		}

		return
	}

	rigidBody := globals.RigidBodyComponent.Get(entry)
	drawHelper(screen, rigidBody)
}

func drawHelper(screen *ebiten.Image, body *componentsPhysicsTypes.RigidBody) {

	red := color.RGBA{R: 255, G: 0, B: 0, A: 255}

	if body.Circle != nil {
		debugRender.DrawCircleBody(screen, *body, red)

	} else {
		debugRender.DrawPolygonBody(screen, *body, red)
	}
}
