package physics

import (
	"math"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	// Shape3 supplies transformed bounds and mass properties for a three-dimensional body.
	Shape3[T constraints.Float] interface {
		GetAABB() *hshg.AABB3[T]
		Transform(position vector.Vec3[T], rotation Quaternion[T])
		Volume() T
		MomentOfInertia(mass T) vector.Vec3[T]
		Clone() Shape3[T]
	}

	// Sphere3 is an analytic sphere collider.
	Sphere3[T constraints.Float] struct {
		Radius   T
		Position vector.Vec3[T]
		aabb     hshg.AABB3[T]
	}

	// Polyhedron3 adapts a triangle-mesh solid for use as a rigid collider.
	Polyhedron3[T constraints.Float] struct {
		Polyhedron *poly.Polyhedron[T]
		Scale      vector.Vec3[T]
		volume     T
		size       vector.Vec3[T]
	}
)

// NewSphere3 creates a sphere collider with a non-negative radius.
func NewSphere3[T constraints.Float](radius T) (sphere *Sphere3[T]) {
	sphere = &Sphere3[T]{Radius: max(radius, 0)}
	sphere.Transform(vector.Vec3[T]{}, IdentityQuaternion[T]())
	return
}

// Transform updates the sphere's world-space center and bounds.
func (s *Sphere3[T]) Transform(position vector.Vec3[T], rotation Quaternion[T]) {
	_ = rotation
	s.Position = position
	s.aabb = hshg.AABB3[T]{X1: position.X - s.Radius, Y1: position.Y - s.Radius, Z1: position.Z - s.Radius, X2: position.X + s.Radius, Y2: position.Y + s.Radius, Z2: position.Z + s.Radius}
}

// GetAABB returns the sphere's cached bounds.
func (s *Sphere3[T]) GetAABB() (aabb *hshg.AABB3[T]) {
	aabb = &s.aabb
	return
}

// Volume returns the sphere's volume.
func (s *Sphere3[T]) Volume() (volume T) {
	volume = T(4.0/3.0*math.Pi) * s.Radius * s.Radius * s.Radius
	return
}

// MomentOfInertia returns the sphere's diagonal inertia tensor.
func (s *Sphere3[T]) MomentOfInertia(mass T) (inertia vector.Vec3[T]) {
	inertia.X = mass * s.Radius * s.Radius * T(2.0/5.0)
	inertia.Y, inertia.Z = inertia.X, inertia.X
	return
}

// Clone returns an independent sphere collider.
func (s *Sphere3[T]) Clone() (clone Shape3[T]) {
	clone = &Sphere3[T]{Radius: s.Radius, Position: s.Position, aabb: s.aabb}
	return
}

// NewPolyhedron3 creates a collider from an independently copied polyhedron and scale.
func NewPolyhedron3[T constraints.Float](source *poly.Polyhedron[T], scale vector.Vec3[T]) (shape *Polyhedron3[T]) {
	if scale == (vector.Vec3[T]{}) {
		scale = vector.Vec3[T]{X: 1, Y: 1, Z: 1}
	}
	shape = &Polyhedron3[T]{Polyhedron: source.Copy(), Scale: scale}
	shape.Polyhedron.Transform(poly.PolyhedronTransform[T]{Scale: scale})
	shape.volume = shape.Polyhedron.Volume()
	shape.size = vector.Vec3[T]{X: shape.Polyhedron.AABB.X2 - shape.Polyhedron.AABB.X1, Y: shape.Polyhedron.AABB.Y2 - shape.Polyhedron.AABB.Y1, Z: shape.Polyhedron.AABB.Z2 - shape.Polyhedron.AABB.Z1}
	return
}

// Transform updates the polyhedron collider's world-space geometry and BVH.
func (p *Polyhedron3[T]) Transform(position vector.Vec3[T], rotation Quaternion[T]) {
	p.Polyhedron.Transform(poly.PolyhedronTransform[T]{Position: position, Rotation: rotation.Euler(), Scale: p.Scale})
}

// GetAABB returns the polyhedron's cached bounds.
func (p *Polyhedron3[T]) GetAABB() (aabb *hshg.AABB3[T]) {
	aabb = p.Polyhedron.AABB
	return
}

// Volume returns the scaled polyhedron volume.
func (p *Polyhedron3[T]) Volume() (volume T) {
	volume = p.volume
	return
}

// MomentOfInertia approximates inertia using the polyhedron's local bounding box.
func (p *Polyhedron3[T]) MomentOfInertia(mass T) (inertia vector.Vec3[T]) {
	inertia = vector.Vec3[T]{X: mass * (p.size.Y*p.size.Y + p.size.Z*p.size.Z) / 12, Y: mass * (p.size.X*p.size.X + p.size.Z*p.size.Z) / 12, Z: mass * (p.size.X*p.size.X + p.size.Y*p.size.Y) / 12}
	return
}

// Clone returns an independent polyhedron collider.
func (p *Polyhedron3[T]) Clone() (clone Shape3[T]) {
	clone = &Polyhedron3[T]{Polyhedron: p.Polyhedron.Copy(), Scale: p.Scale, volume: p.volume, size: p.size}
	return
}
