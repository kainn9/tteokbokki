package ecs

import (
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/yohamta/donburi"
)

type SystemType int

const (
	ClientType SystemType = iota
	RenderType
	SimType
)

type SceneAdmin struct {
	World   donburi.World
	Systems []System
}

type System interface{}

// Query interface for systems that need to query a world for entities.
// By default, the system will iterate through all entities returned by the query.
// If the system needs to iterate through the entities in a different way, it can implement
// the CustomIteration interface.
type Query interface {
	Query() *donburi.Query
}

// Sim interface for systems that simulate the game logic.
type Sim interface {
	Run(dt float64, entry *donburi.Entry)
}

// Render interface for systems that render the game, based on the game state.
type Render interface {
	Draw(screen *ebiten.Image, entry *donburi.Entry)
}

// Client interface for systems that handle
// "client logic" (i.e. input, sounds, etc).
type Client interface {
	Sync(entry *donburi.Entry)
}

// For multiple queries.
type Queries interface {
	Queries() []*donburi.Query
}

// For systems that need to iterate through entities in a different way
// other than the default iteration, return true.
type CustomIteration interface {
	CustomIteration() bool
}

func NewSceneAdmin() SceneAdmin {

	return SceneAdmin{
		World: donburi.NewWorld(),
	}
}

// Adds an entity to the world and returns its entry variable.
func (sa SceneAdmin) AddEntity(components ...donburi.IComponentType) *donburi.Entry {
	entity := sa.World.Entry(sa.World.Create(components...))

	return entity
}

// Adds a system to the scene admin.
func (s *SceneAdmin) AddSystem(newSystem System) {
	s.Systems = append(s.Systems, newSystem)
}

func (s SceneAdmin) Sync() {
	s.processSystem(ClientType, nil, nil)
}

func (s SceneAdmin) Sim(dt float64) {
	s.processSystem(SimType, dt, nil)
}

func (s SceneAdmin) Draw(screen *ebiten.Image) {

	s.processSystem(RenderType, 0, screen)
}

// Iterates through all systems added to the scene admin and runs them.
func (s SceneAdmin) processSystem(sysType SystemType, args ...interface{}) {

	for _, system := range s.Systems {

		query := &donburi.Query{}
		customIteration := false

		if q, ok := system.(Query); ok {
			query = q.Query()

		} else {
			query = nil
		}

		if sysWithCustomIterMethod, ok := system.(CustomIteration); ok {
			customIteration = sysWithCustomIterMethod.CustomIteration()
		}

		runOnce := query == nil || customIteration

		if sysType == ClientType {

			if clientSys, ok := system.(Client); ok {

				if runOnce {
					clientSys.Sync(nil)
					continue
				}

				query.Each(s.World, func(entry *donburi.Entry) {
					clientSys.Sync(entry)
				})
				continue
			}
		}

		if sysType == SimType {

			dt := args[0].(float64)

			if simSys, ok := system.(Sim); ok {

				if runOnce {
					simSys.Run(dt, nil)
					continue
				}

				query.Each(s.World, func(entry *donburi.Entry) {
					simSys.Run(dt, entry)
				})

				continue
			}
		}

		if sysType == RenderType {

			screen := args[1].(*ebiten.Image)

			if renderSys, ok := system.(Render); ok {

				if runOnce {
					renderSys.Draw(screen, nil)
					continue
				}

				query.Each(s.World, func(entry *donburi.Entry) {
					renderSys.Draw(screen, entry)
				})
				continue
			}
		}
	}
}
