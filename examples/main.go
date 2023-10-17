package main

import (
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/kainn9/coldBrew"
	"github.com/kainn9/tteokbokki/examples/globals"
	"github.com/kainn9/tteokbokki/examples/scenes"
)

type game struct {
	width, height int
	manager       *coldBrew.Manager
}

const (
	SCENE_CACHE_LIMIT = 5
)

func main() {
	game := NewGame()
	ebiten.RunGame(game)
}

func NewGame() *game {

	manager := coldBrew.NewManager(5, ebiten.NewImage(globals.GAME_WIDTH, globals.GAME_HEIGHT))
	firstScene := scenes.SimpleRigidBodiesScene
	manager.LoadScene(firstScene)

	g := &game{
		width:   globals.GAME_WIDTH,
		height:  globals.GAME_HEIGHT,
		manager: manager,
	}

	ebiten.SetWindowTitle("Demo!")
	ebiten.SetWindowSize(g.width, g.height)

	return g

}

func (g *game) Update() error {

	deltaTime := (0.017)
	g.manager.GetActiveScene().Load()
	g.manager.GetActiveScene().Sync()
	g.manager.GetActiveScene().Sim(deltaTime)

	return nil
}

func (g *game) Draw(screen *ebiten.Image) {
	g.manager.GetActiveScene().Draw(screen)
}

func (g *game) Layout(w, h int) (int, int) {
	return g.width, g.height
}
