package main

import (
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/kainn9/coldBrew"
	"github.com/kainn9/tteokbokki/example/constants"
	"github.com/kainn9/tteokbokki/example/scenes"
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

	manager := coldBrew.NewManager(5, ebiten.NewImage(constants.GAME_WIDTH, constants.GAME_HEIGHT))
	firstScene := scenes.SimpleRigidBodiesScene
	manager.LoadScene(firstScene)

	g := &game{
		width:   constants.GAME_WIDTH,
		height:  constants.GAME_HEIGHT,
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
	ebitenutil.DebugPrint(screen, "Hello! This is a very basic demo of tteokbokki that does not include all functionality. You can click to spawn random rigid bodies, Enjoy!")
	g.manager.GetActiveScene().Draw(screen)
}

func (g *game) Layout(w, h int) (int, int) {
	return g.width, g.height
}
