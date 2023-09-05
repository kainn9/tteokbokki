package clientSystems

import (
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/kainn9/tteokbokki/math/vec"
	"github.com/yohamta/donburi"
)

type ClickTracker struct {
	Clicks *[]vec.Vec2
}

func (ClientStruct) NewClickTrackerSystem(clicks *[]vec.Vec2) ClickTracker {
	return ClickTracker{
		Clicks: clicks,
	}
}

func (sys ClickTracker) Sync(*donburi.Entry) {
	if inpututil.IsMouseButtonJustReleased(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()

		click := vec.Vec2{
			X: float64(x),
			Y: float64(y),
		}

		*sys.Clicks = append(*sys.Clicks, click)
	}
}
