package main

import (
	"github.com/kainn9/tteokbokki/examples/ecsExample/ecs"
	"github.com/kainn9/tteokbokki/examples/ecsExample/globals"
	"github.com/kainn9/tteokbokki/examples/ecsExample/scenes"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

type game struct {
	width, height int
	currentScene  ecs.SceneAdmin
}

var (
	allScenes         = make([]ecs.SceneAdmin, 0)
	currentSceneIndex = 1

	// TEMP

)

func main() {

	particleScene := scenes.NewSimpleParticleScene()
	rigidBodyScene := scenes.NewSimpleRigidBodiesScene()

	allScenes = append(allScenes, particleScene)
	allScenes = append(allScenes, rigidBodyScene)

	game := NewGame(allScenes[currentSceneIndex])

	ebiten.RunGame(game)

}

func NewGame(firstScene ecs.SceneAdmin) *game {

	g := &game{
		width:        globals.GAME_WIDTH,
		height:       globals.GAME_HEIGHT,
		currentScene: firstScene,
	}

	ebiten.SetWindowTitle("Physics!")
	ebiten.SetWindowSize(g.width, g.height)

	return g

}

func (g *game) Update() error {

	// scene rotation hack for now
	if inpututil.IsKeyJustPressed(ebiten.Key0) {
		nextIndex := (currentSceneIndex + 1) % len(allScenes)

		currentSceneIndex = nextIndex

		g.currentScene = allScenes[currentSceneIndex]
	}

	deltaTime := (0.017)

	g.currentScene.Sync()
	g.currentScene.Sim(deltaTime)

	return nil
}

func (g *game) Draw(screen *ebiten.Image) {
	g.currentScene.Draw(screen)
}

func (g *game) Layout(w, h int) (int, int) {
	return g.width, g.height
}
