package factory

import (
	tBokiComponents "github.com/kainn9/tteokbokki/components"
	tBokiVec "github.com/kainn9/tteokbokki/math/vec"
)

var (
	boxFriction    = 0.025
	boundsFriction = 0.5
)

func BoundsBoxComponent(x, y, w, h, rotation float64) tBokiComponents.RigidBody {

	body := tBokiComponents.NewRigidBodyBox(x, y, w, h, 0, true)
	body.Friction = boundsFriction
	body.Elasticity = 0.2
	body.Rotation = rotation
	body.UpdateVertices()

	return *body

}

func BoundsCircleComponent(x, y, radius float64) tBokiComponents.RigidBody {

	body := tBokiComponents.NewRigidBodyCircle(x, y, radius, 0, true)
	body.Friction = boundsFriction
	body.Elasticity = 0.2

	return *body

}

func HexagonPhysicsObject(x, y, rotation float64, vertices []tBokiVec.Vec2) tBokiComponents.RigidBody {

	body := tBokiComponents.NewRigidBodyPolygon(x, y, 10, vertices, true)
	body.Elasticity = 0.3
	body.Friction = 0.5
	body.Rotation = rotation
	body.UpdateVertices()
	body.SetAngularMass(6000)

	return *body
}

func CirclePhysicsObject(x, y, radius, rotation float64) tBokiComponents.RigidBody {

	body := tBokiComponents.NewRigidBodyCircle(x, y, radius, 10, true)
	body.Elasticity = 0.3
	body.Friction = 0.5
	body.Rotation = rotation

	return *body
}

func BoxPhysicsObject(x, y, w, h, rotation float64) tBokiComponents.RigidBody {

	body := tBokiComponents.NewRigidBodyBox(x, y, w, h, 10, true)
	body.Elasticity = 0.3
	body.Friction = boxFriction
	body.Rotation = rotation
	body.UpdateVertices()

	return *body
}

func BoxPhysicsObjectLinear(x, y, w, h float64) tBokiComponents.RigidBody {

	body := tBokiComponents.NewRigidBodyBox(x, y, w, h, 10, false)
	body.Elasticity = 0.5
	body.Friction = boxFriction
	body.UpdateVertices()

	return *body
}

func MovingPlatformThingy(x, y float64, angular bool) ([]*tBokiComponents.RigidBody, *tBokiComponents.JointConstraint) {
	platformAnchor := tBokiComponents.NewRigidBodyCircle(x, y, 10, 0, false)

	platform := tBokiComponents.NewRigidBodyBox(platformAnchor.Pos.X-120, platformAnchor.Pos.Y, 120, 30, 5, angular)
	platform.Unstoppable = true

	platformJointData := tBokiComponents.NewJointConstraint(
		platformAnchor,
		platform,
	)

	bodies := []*tBokiComponents.RigidBody{platformAnchor,
		platform,
	}

	return bodies, platformJointData

}
