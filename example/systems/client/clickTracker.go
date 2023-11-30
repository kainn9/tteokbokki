package clientSystems

import (
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
	"github.com/yohamta/donburi"
)

type ClickTracker struct {
	Clicks *[]tBokiVec.Vec2
}

func NewClickTracker(clicks *[]tBokiVec.Vec2) ClickTracker {
	return ClickTracker{
		Clicks: clicks,
	}
}

func (sys ClickTracker) Sync(_ *donburi.Entry) {

	if inpututil.IsMouseButtonJustReleased(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()

		click := tBokiVec.Vec2{
			X: float64(x),
			Y: float64(y),
		}

		*sys.Clicks = append(*sys.Clicks, click)
	}

	sys.handleMobile()

}

func (sys *ClickTracker) handleMobile() {

	tIds := []ebiten.TouchID{}
	tIds = inpututil.AppendJustPressedTouchIDs(tIds)

	if len(tIds) > 0 {
		x, y := ebiten.TouchPosition(tIds[0])
		click := tBokiVec.Vec2{
			X: float64(x),
			Y: float64(y),
		}

		*sys.Clicks = append(*sys.Clicks, click)

	}

}
