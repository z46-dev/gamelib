package physics

import (
	"fmt"

	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	// ReconnectionPolicy controls how broken rope links are restored.
	ReconnectionPolicy uint8

	// RopeConfig controls shared rope construction and constraint behavior.
	RopeConfig[T constraints.Float] struct {
		Segments                                                                                        int
		Radius, Mass, Compliance, Damping, LinearDamping, AngularDamping, BreakForce, ReconnectDistance T
		Material                                                                                        Material[T]
		Filter                                                                                          CollisionFilter
		Reconnection                                                                                    ReconnectionPolicy
	}

	// Rope2 owns the particles and links created for a two-dimensional rope.
	Rope2[T constraints.Float] struct {
		Bodies            []*Body2[T]
		Constraints       []*DistanceConstraint2[T]
		Reconnection      ReconnectionPolicy
		ReconnectDistance T
	}

	// Rope3 owns the particles and links created for a three-dimensional rope.
	Rope3[T constraints.Float] struct {
		Bodies            []*Body3[T]
		Constraints       []*DistanceConstraint3[T]
		Reconnection      ReconnectionPolicy
		ReconnectDistance T
	}
)

const (
	// ReconnectNever leaves broken rope links broken until explicitly repaired.
	ReconnectNever ReconnectionPolicy = iota
	// ReconnectWhenClose repairs a link once its endpoints return within range.
	ReconnectWhenClose
)

// AddRope creates a chain of circular bodies between two existing 2D bodies.
func (w *World2[T]) AddRope(firstID, secondID BodyID, config RopeConfig[T]) (rope *Rope2[T], err error) {
	var first, second *Body2[T]
	var firstFound, secondFound bool
	first, firstFound = w.bodies[firstID]
	second, secondFound = w.bodies[secondID]
	if !firstFound || !secondFound || first == second || config.Segments < 1 || config.Radius <= 0 {
		err = fmt.Errorf("physics: rope requires two bodies, at least one segment, and a positive radius")
		return
	}
	if config.Material == (Material[T]{}) {
		config.Material = DefaultMaterial[T]()
	}
	if config.Filter == (CollisionFilter{}) {
		config.Filter = DefaultCollisionFilter()
	}
	rope = &Rope2[T]{Bodies: make([]*Body2[T], 0, config.Segments+1), Constraints: make([]*DistanceConstraint2[T], 0, config.Segments), Reconnection: config.Reconnection, ReconnectDistance: config.ReconnectDistance}
	rope.Bodies = append(rope.Bodies, first)
	for i := 1; i < config.Segments; i++ {
		var ratio T = T(i) / T(config.Segments)
		var body *Body2[T]
		body, err = w.AddBody(Body2Config[T]{Type: DynamicBody, Shape: NewCircle2(config.Radius), Position: vector.Vec2[T]{X: first.Position.X + (second.Position.X-first.Position.X)*ratio, Y: first.Position.Y + (second.Position.Y-first.Position.Y)*ratio}, Mass: config.Mass, Material: config.Material, Filter: config.Filter, GravityScale: 1, LinearDamping: config.LinearDamping, AngularDamping: config.AngularDamping})
		if err != nil {
			return
		}
		rope.Bodies = append(rope.Bodies, body)
	}
	rope.Bodies = append(rope.Bodies, second)
	var restLength T = first.Position.Dist(&second.Position) / T(config.Segments)
	for i := 0; i < config.Segments; i++ {
		var link *DistanceConstraint2[T]
		link, err = w.AddDistanceConstraint(DistanceConstraintConfig[T]{First: rope.Bodies[i].ID, Second: rope.Bodies[i+1].ID, RestLength: restLength, Compliance: config.Compliance, Damping: config.Damping, BreakForce: config.BreakForce})
		if err != nil {
			return
		}
		rope.Constraints = append(rope.Constraints, link)
	}
	w.ropes = append(w.ropes, rope)
	return
}

// AddRope creates a chain of spherical bodies between two existing 3D bodies.
func (w *World3[T]) AddRope(firstID, secondID BodyID, config RopeConfig[T]) (rope *Rope3[T], err error) {
	var first, second *Body3[T]
	var firstFound, secondFound bool
	first, firstFound = w.bodies[firstID]
	second, secondFound = w.bodies[secondID]
	if !firstFound || !secondFound || first == second || config.Segments < 1 || config.Radius <= 0 {
		err = fmt.Errorf("physics: rope requires two bodies, at least one segment, and a positive radius")
		return
	}
	if config.Material == (Material[T]{}) {
		config.Material = DefaultMaterial[T]()
	}
	if config.Filter == (CollisionFilter{}) {
		config.Filter = DefaultCollisionFilter()
	}
	rope = &Rope3[T]{Bodies: make([]*Body3[T], 0, config.Segments+1), Constraints: make([]*DistanceConstraint3[T], 0, config.Segments), Reconnection: config.Reconnection, ReconnectDistance: config.ReconnectDistance}
	rope.Bodies = append(rope.Bodies, first)
	for i := 1; i < config.Segments; i++ {
		var ratio T = T(i) / T(config.Segments)
		var body *Body3[T]
		body, err = w.AddBody(Body3Config[T]{Type: DynamicBody, Shape: NewSphere3(config.Radius), Position: vector.Vec3[T]{X: first.Position.X + (second.Position.X-first.Position.X)*ratio, Y: first.Position.Y + (second.Position.Y-first.Position.Y)*ratio, Z: first.Position.Z + (second.Position.Z-first.Position.Z)*ratio}, Mass: config.Mass, Material: config.Material, Filter: config.Filter, GravityScale: 1, LinearDamping: config.LinearDamping, AngularDamping: config.AngularDamping})
		if err != nil {
			return
		}
		rope.Bodies = append(rope.Bodies, body)
	}
	rope.Bodies = append(rope.Bodies, second)
	var restLength T = first.Position.Dist(&second.Position) / T(config.Segments)
	for i := 0; i < config.Segments; i++ {
		var link *DistanceConstraint3[T]
		link, err = w.AddDistanceConstraint(DistanceConstraintConfig[T]{First: rope.Bodies[i].ID, Second: rope.Bodies[i+1].ID, RestLength: restLength, Compliance: config.Compliance, Damping: config.Damping, BreakForce: config.BreakForce})
		if err != nil {
			return
		}
		rope.Constraints = append(rope.Constraints, link)
	}
	w.ropes = append(w.ropes, rope)
	return
}

// UpdateReconnection repairs eligible broken links in a 2D rope.
func (r *Rope2[T]) UpdateReconnection() {
	if r.Reconnection != ReconnectWhenClose || r.ReconnectDistance <= 0 {
		return
	}
	for _, link := range r.Constraints {
		var first, second vector.Vec2[T] = bodyAnchor2(link.First, link.LocalAnchorFirst), bodyAnchor2(link.Second, link.LocalAnchorSecond)
		if link.Broken && first.Dist(&second) <= r.ReconnectDistance {
			link.Repair(link.RestLength)
		}
	}
}

// UpdateReconnection repairs eligible broken links in a 3D rope.
func (r *Rope3[T]) UpdateReconnection() {
	if r.Reconnection != ReconnectWhenClose || r.ReconnectDistance <= 0 {
		return
	}
	for _, link := range r.Constraints {
		var first, second vector.Vec3[T] = bodyAnchor3(link.First, link.LocalAnchorFirst), bodyAnchor3(link.Second, link.LocalAnchorSecond)
		if link.Broken && first.Dist(&second) <= r.ReconnectDistance {
			link.Repair(link.RestLength)
		}
	}
}
