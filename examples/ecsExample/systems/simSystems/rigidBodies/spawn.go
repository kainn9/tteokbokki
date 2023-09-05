package rigidBodiesSimSystems

import (
	"math/rand"

	"github.com/kainn9/tteokbokki/examples/ecsExample/ecs"
	"github.com/kainn9/tteokbokki/examples/ecsExample/factory"
	"github.com/kainn9/tteokbokki/examples/ecsExample/globals"

	componentPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/math/vec"
	"github.com/kainn9/tteokbokki/sliceHelper"
	"github.com/yohamta/donburi"
)

type Spawn struct {
	Scene  ecs.SceneAdmin
	Clicks *[]vec.Vec2
}

func (RigidBodiesStruct) NewSpawn(clicks *[]vec.Vec2, scene ecs.SceneAdmin) Spawn {
	return Spawn{
		Scene:  scene,
		Clicks: clicks,
	}
}

func (sys Spawn) Run(dt float64, entry *donburi.Entry) {

	sortedClicks := sliceHelper.Invert(*sys.Clicks)

	if len(sortedClicks) == 0 {
		return
	}

	click := sliceHelper.Pop(&sortedClicks)

	randomNumber := rand.Intn(4) // Generate a random integer between 0 and 2 (inclusive)

	entry = sys.Scene.AddEntity(globals.RigidBodyComponent)

	var data componentPhysicsTypes.RigidBody = componentPhysicsTypes.RigidBody{}

	switch randomNumber {
	case 0:
		data = factory.BoxPhysicsObject(click.X, click.Y, 25, 50, float64(rand.Intn(10)))
	case 1:
		data = factory.BoxPhysicsObjectLinear(click.X, click.Y, 50, 50)
	case 2:
		data = factory.HexagonPhysicsObject(click.X, click.Y, float64(rand.Intn(10)), hexagonVertices())
	case 3:
		data = factory.CirclePhysicsObject(click.X, click.Y, 35, float64(rand.Intn(10)))
	}

	globals.RigidBodyComponent.SetValue(entry, data)

	*sys.Clicks = sortedClicks

}

func hexagonVertices() []vec.Vec2 {
	vertices := make([]vec.Vec2, 6)

	vertices[0] = vec.Vec2{X: -30, Y: 0}
	vertices[1] = vec.Vec2{X: -12, Y: -30}
	vertices[2] = vec.Vec2{X: 12, Y: -30}
	vertices[3] = vec.Vec2{X: 30, Y: 0}
	vertices[4] = vec.Vec2{X: 12, Y: 30}
	vertices[5] = vec.Vec2{X: -12, Y: 30}
	return vertices
}
