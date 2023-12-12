package tBokiComponents

import tBokiVec "github.com/kainn9/tteokbokki/math/vec"

// Collision data between two rigid bodies.
type Contact struct {
	Start, End, Normal tBokiVec.Vec2
	Depth              float64

	// Only for polygon on polygon ResolverType (otherwise nil).
	// The Edge of deepest penetration.
	// Key is pointer to penetrated rigid body.
	// Vec is indexes of edge vertices.
	IncidentEdge map[*RigidBody][]int
}

type ContactsType int

const (
	ResolverType ContactsType = iota
	SolverType
)

type Contacts struct {
	Type ContactsType
	Data []Contact
}

func NewContact() Contact {
	return Contact{}
}

func NewContacts(cType ContactsType) Contacts {
	return Contacts{
		Type: cType,
		Data: make([]Contact, 0),
	}

}
