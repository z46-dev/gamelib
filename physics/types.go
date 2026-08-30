package physics

import (
	"github.com/z46-dev/gamelib/hshg"
	"golang.org/x/exp/constraints"
)

type (
	// BodyType controls whether a body is simulated, user-driven, or immovable.
	BodyType uint8

	// BodyID uniquely identifies a body within one physics world.
	BodyID uint64

	// Material controls density, restitution, and Coulomb friction.
	Material[T constraints.Float] struct {
		Density, Restitution, StaticFriction, DynamicFriction T
	}

	// CollisionFilter controls which categories of bodies may collide.
	CollisionFilter struct {
		Category, Mask uint32
	}

	// WorldConfig controls gravity, solver iterations, and penetration correction.
	WorldConfig[T constraints.Float] struct {
		GravityX, GravityY, GravityZ T
		VelocityIterations           int
		PositionIterations           int
		PenetrationSlop              T
		PositionCorrection           T
		RestitutionThreshold         T
		EnableWarmStarting           bool
		EnableSleeping               bool
		SleepLinearThreshold         T
		SleepAngularThreshold        T
		SleepTime                    T
		EnableCCD                    bool
		CCDMaxSubsteps               int
		CCDMotionThreshold           T
		MaxStepDelta                 T
		ParallelWorkers              int
		MinimumParallelIslandBodies  int
		SpatialHash                  hshg.SpatialHashConfig
	}
)

const (
	// StaticBody is immovable and has infinite effective mass.
	StaticBody BodyType = iota
	// KinematicBody moves from user-controlled velocity but ignores forces.
	KinematicBody
	// DynamicBody responds to forces, gravity, and contacts.
	DynamicBody
)

// DefaultMaterial returns a practical general-purpose material.
func DefaultMaterial[T constraints.Float]() (material Material[T]) {
	material = Material[T]{Density: 1, Restitution: 0.1, StaticFriction: 0.6, DynamicFriction: 0.4}
	return
}

// DefaultCollisionFilter returns a filter that collides with every category.
func DefaultCollisionFilter() (filter CollisionFilter) {
	filter = CollisionFilter{Category: 1, Mask: ^uint32(0)}
	return
}

// DefaultWorldConfig returns stable baseline solver settings with no gravity.
func DefaultWorldConfig[T constraints.Float]() (config WorldConfig[T]) {
	config = WorldConfig[T]{
		VelocityIterations: 8, PositionIterations: 3, PenetrationSlop: 0.005, PositionCorrection: 0.2, RestitutionThreshold: 0.5,
		EnableWarmStarting: true, EnableSleeping: true, SleepLinearThreshold: 0.05, SleepAngularThreshold: 0.05, SleepTime: 0.5,
		EnableCCD: true, CCDMaxSubsteps: 32, CCDMotionThreshold: 0.25, MaxStepDelta: T(1.0 / 60.0), MinimumParallelIslandBodies: 32,
	}
	return
}

// filtersCollide reports whether two collision filters mutually opt in.
func filtersCollide(first, second CollisionFilter) (collides bool) {
	collides = first.Category&second.Mask != 0 && second.Category&first.Mask != 0
	return
}

// clampMaterial normalizes material values used by the solver.
func clampMaterial[T constraints.Float](material Material[T]) (clamped Material[T]) {
	clamped = material
	clamped.Density = max(clamped.Density, 0)
	clamped.Restitution = min(max(clamped.Restitution, 0), 1)
	clamped.StaticFriction = max(clamped.StaticFriction, 0)
	clamped.DynamicFriction = max(clamped.DynamicFriction, 0)
	return
}
