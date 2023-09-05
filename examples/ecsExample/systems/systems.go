package systems

import (
	"github.com/kainn9/tteokbokki/examples/ecsExample/systems/clientSystems"
	"github.com/kainn9/tteokbokki/examples/ecsExample/systems/renderSystems"
	"github.com/kainn9/tteokbokki/examples/ecsExample/systems/simSystems"
)

var ClientSystems = clientSystems.ClientStruct{}
var RenderSystems = renderSystems.RenderStruct{}
var SimSystems = simSystems.SimStruct{}
