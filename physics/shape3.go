package physics

import (
	"math"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	inertiaTensorShape3[T constraints.Float] interface {
		InertiaTensor(mass T) [9]T
	}

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

// InertiaTensor returns the sphere's isotropic local inertia tensor.
func (s *Sphere3[T]) InertiaTensor(mass T) (tensor [9]T) {
	var inertia vector.Vec3[T] = s.MomentOfInertia(mass)
	tensor[0], tensor[4], tensor[8] = inertia.X, inertia.Y, inertia.Z
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
	var local *poly.Polyhedron[T] = source.Copy()
	local.Transform(poly.IdentityPolyhedronTransform[T]())
	var centroid vector.Vec3[T] = local.Centroid()
	var centered []vector.Vec3[T] = make([]vector.Vec3[T], len(local.Reference))
	for index := range local.Reference {
		centered[index] = vector.Vec3[T]{X: local.Reference[index].X - centroid.X, Y: local.Reference[index].Y - centroid.Y, Z: local.Reference[index].Z - centroid.Z}
	}
	var err error
	if local, err = poly.NewPolyhedron(centered, local.Triangles); err != nil {
		panic(err)
	}
	shape = &Polyhedron3[T]{Polyhedron: local, Scale: scale}
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

// MomentOfInertia returns the diagonal of the volume-integrated local inertia tensor.
func (p *Polyhedron3[T]) MomentOfInertia(mass T) (inertia vector.Vec3[T]) {
	var tensor [9]T = p.InertiaTensor(mass)
	inertia = vector.Vec3[T]{X: tensor[0], Y: tensor[4], Z: tensor[8]}
	return
}

// InertiaTensor integrates the closed triangle mesh as signed origin tetrahedra.
func (p *Polyhedron3[T]) InertiaTensor(mass T) (tensor [9]T) {
	var (
		volume   T
		centroid vector.Vec3[T]
		second   [6]T
	)
	for _, triangle := range p.Polyhedron.Triangles {
		var a, b, c vector.Vec3[T] = p.Polyhedron.Points[triangle.A], p.Polyhedron.Points[triangle.B], p.Polyhedron.Points[triangle.C]
		var determinant T = a.X*(b.Y*c.Z-b.Z*c.Y) + a.Y*(b.Z*c.X-b.X*c.Z) + a.Z*(b.X*c.Y-b.Y*c.X)
		var tetraVolume T = determinant / 6
		volume += tetraVolume
		centroid.X += (a.X + b.X + c.X) * tetraVolume / 4
		centroid.Y += (a.Y + b.Y + c.Y) * tetraVolume / 4
		centroid.Z += (a.Z + b.Z + c.Z) * tetraVolume / 4
		second[0] += tetraSecondMoment3(a.X, b.X, c.X, a.X, b.X, c.X, tetraVolume)
		second[1] += tetraSecondMoment3(a.Y, b.Y, c.Y, a.Y, b.Y, c.Y, tetraVolume)
		second[2] += tetraSecondMoment3(a.Z, b.Z, c.Z, a.Z, b.Z, c.Z, tetraVolume)
		second[3] += tetraSecondMoment3(a.X, b.X, c.X, a.Y, b.Y, c.Y, tetraVolume)
		second[4] += tetraSecondMoment3(a.X, b.X, c.X, a.Z, b.Z, c.Z, tetraVolume)
		second[5] += tetraSecondMoment3(a.Y, b.Y, c.Y, a.Z, b.Z, c.Z, tetraVolume)
	}
	if volume == 0 {
		return
	}
	if volume < 0 {
		volume = -volume
		centroid.Mul(-1)
		for index := range second {
			second[index] = -second[index]
		}
	}
	centroid.Mul(1 / volume)
	var density T = mass / volume
	var (
		xx, yy, zz T = second[0] * density, second[1] * density, second[2] * density
		xy, xz, yz T = second[3] * density, second[4] * density, second[5] * density
	)
	tensor[0] = yy + zz - mass*(centroid.Y*centroid.Y+centroid.Z*centroid.Z)
	tensor[4] = xx + zz - mass*(centroid.X*centroid.X+centroid.Z*centroid.Z)
	tensor[8] = xx + yy - mass*(centroid.X*centroid.X+centroid.Y*centroid.Y)
	tensor[1], tensor[3] = -xy+mass*centroid.X*centroid.Y, -xy+mass*centroid.X*centroid.Y
	tensor[2], tensor[6] = -xz+mass*centroid.X*centroid.Z, -xz+mass*centroid.X*centroid.Z
	tensor[5], tensor[7] = -yz+mass*centroid.Y*centroid.Z, -yz+mass*centroid.Y*centroid.Z
	return
}

func tetraSecondMoment3[T constraints.Float](ax, bx, cx, ay, by, cy, volume T) (moment T) {
	moment = volume * (2*(ax*ay+bx*by+cx*cy) + ax*by + ay*bx + ax*cy + ay*cx + bx*cy + by*cx) / 20
	return
}

// Clone returns an independent polyhedron collider.
func (p *Polyhedron3[T]) Clone() (clone Shape3[T]) {
	clone = &Polyhedron3[T]{Polyhedron: p.Polyhedron.Copy(), Scale: p.Scale, volume: p.volume, size: p.size}
	return
}
