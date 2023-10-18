package simSystems

import (
	"math/rand"

	"github.com/kainn9/coldBrew"
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	"github.com/kainn9/tteokbokki/example/components"
	"github.com/kainn9/tteokbokki/example/factory"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
	tBokiSliceHelper "github.com/kainn9/tteokbokki/sliceHelper"
	"github.com/yohamta/donburi"
)

type SpawnHandler struct {
	SceneAdmin *coldBrew.Scene
	Clicks     *[]tBokiVec.Vec2
}

func NewSpawnHandler(clicks *[]tBokiVec.Vec2, scene *coldBrew.Scene) SpawnHandler {
	return SpawnHandler{
		SceneAdmin: scene,
		Clicks:     clicks,
	}
}

func (sys SpawnHandler) Run(dt float64, _ *donburi.Entry) {

	sortedClicks := tBokiSliceHelper.Invert(*sys.Clicks)

	if len(sortedClicks) == 0 {
		return
	}

	click := tBokiSliceHelper.Pop(&sortedClicks)

	randomNumber := rand.Intn(4)

	entity := sys.SceneAdmin.AddEntity(components.RigidBodyComponent)

	var data tBokiComponents.RigidBody = tBokiComponents.RigidBody{}

	switch randomNumber {

	case 0:
		data = factory.BoxPhysicsObject(click.X, click.Y, 35, 70, 0)
	case 1:
		data = factory.BoxPhysicsObjectLinear(click.X, click.Y, 50, 50)
	case 2:
		data = factory.HexagonPhysicsObject(click.X, click.Y, float64(rand.Intn(10)), hexagonVertices())
	case 3:
		data = factory.CirclePhysicsObject(click.X, click.Y, 20, float64(rand.Intn(10)))
	}

	components.RigidBodyComponent.SetValue(entity, data)

	*sys.Clicks = sortedClicks

}

func hexagonVertices() []tBokiVec.Vec2 {
	vertices := make([]tBokiVec.Vec2, 6)

	vertices[0] = tBokiVec.Vec2{X: -30, Y: 0}
	vertices[1] = tBokiVec.Vec2{X: -12, Y: -30}
	vertices[2] = tBokiVec.Vec2{X: 12, Y: -30}
	vertices[3] = tBokiVec.Vec2{X: 30, Y: 0}
	vertices[4] = tBokiVec.Vec2{X: 12, Y: 30}
	vertices[5] = tBokiVec.Vec2{X: -12, Y: 30}
	return vertices
}
