package factory

import (
	"github.com/kainn9/tteokbokki/components"
	componentsPhysicsTypes "github.com/kainn9/tteokbokki/components/physics"
	"github.com/kainn9/tteokbokki/examples/ecsExample/globals"
	"github.com/kainn9/tteokbokki/math/vec"
)

func BoundsBoxComponent(x, y, w, h, rotation float64) componentsPhysicsTypes.RigidBody {

	body := components.Physics.NewRigidBodyBox(x, y, w, h, 0, true)
	body.Friction = globals.BoundsFriction
	body.Elasticity = 0.2
	body.Rotation = rotation
	body.UpdateVertices()

	return *body

}

func BoundsCircleComponent(x, y, radius float64) componentsPhysicsTypes.RigidBody {

	body := components.Physics.NewRigidBodyCircle(x, y, radius, 0, true)
	body.Friction = globals.BoundsFriction
	body.Elasticity = 0.2

	return *body

}

func HexagonPhysicsObject(x, y, rotation float64, vertices []vec.Vec2) componentsPhysicsTypes.RigidBody {

	body := components.Physics.NewRigidBodyPolygon(x, y, 10, vertices, true)
	body.Elasticity = 0.3
	body.Friction = 0.5
	body.Rotation = rotation
	body.UpdateVertices()
	body.SetAngularMass(6000)

	return *body
}

func CirclePhysicsObject(x, y, radius, rotation float64) componentsPhysicsTypes.RigidBody {

	body := components.Physics.NewRigidBodyCircle(x, y, radius, 10, true)
	body.Elasticity = 0.3
	body.Friction = 0.5
	body.Rotation = rotation

	return *body
}

func BoxPhysicsObject(x, y, w, h, rotation float64) componentsPhysicsTypes.RigidBody {

	body := components.Physics.NewRigidBodyBox(x, y, w, h, 10, true)
	body.Elasticity = 0.3
	body.Friction = 0.030
	body.Rotation = rotation
	body.UpdateVertices()

	return *body
}

func BoxPhysicsObjectLinear(x, y, w, h float64) componentsPhysicsTypes.RigidBody {

	body := components.Physics.NewRigidBodyBox(x, y, w, h, 10, false)
	body.Elasticity = 0.5
	body.Friction = 0.015
	body.UpdateVertices()

	return *body
}