package physics

import (
	"math"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	// Shape2 supplies transformed bounds and mass properties for a two-dimensional body.
	Shape2[T constraints.Float] interface {
		GetAABB() *hshg.AABB2[T]
		Transform(position vector.Vec2[T], rotation T)
		Area() T
		MomentOfInertia(mass T) T
		Clone() Shape2[T]
	}

	// Circle2 is an analytic circle collider.
	Circle2[T constraints.Float] struct {
		Radius   T
		Position vector.Vec2[T]
		aabb     hshg.AABB2[T]
	}

	// Polygon2 adapts a polygon mesh for use as a rigid collider.
	Polygon2[T constraints.Float] struct {
		Polygon       *poly.Polygon[T]
		Scale         T
		area          T
		width, height T
	}
)

// NewCircle2 creates a circle collider with a non-negative radius.
func NewCircle2[T constraints.Float](radius T) (circle *Circle2[T]) {
	circle = &Circle2[T]{Radius: max(radius, 0)}
	circle.Transform(vector.Vec2[T]{}, 0)
	return
}

// Transform updates the circle's world-space center and bounds.
func (c *Circle2[T]) Transform(position vector.Vec2[T], rotation T) {
	_ = rotation
	c.Position = position
	c.aabb = hshg.AABB2[T]{X1: position.X - c.Radius, Y1: position.Y - c.Radius, X2: position.X + c.Radius, Y2: position.Y + c.Radius}
}

// GetAABB returns the circle's cached bounds.
func (c *Circle2[T]) GetAABB() (aabb *hshg.AABB2[T]) {
	aabb = &c.aabb
	return
}

// Area returns the circle's area.
func (c *Circle2[T]) Area() (area T) {
	area = T(math.Pi) * c.Radius * c.Radius
	return
}

// MomentOfInertia returns the circle's moment about its center.
func (c *Circle2[T]) MomentOfInertia(mass T) (inertia T) {
	inertia = mass * c.Radius * c.Radius / 2
	return
}

// Clone returns an independent circle collider.
func (c *Circle2[T]) Clone() (clone Shape2[T]) {
	clone = &Circle2[T]{Radius: c.Radius, Position: c.Position, aabb: c.aabb}
	return
}

// NewPolygon2 creates a polygon collider from local-space vertices and uniform scale.
func NewPolygon2[T constraints.Float](points []vector.Vec2[T], scale T) (shape *Polygon2[T]) {
	var pointers []*vector.Vec2[T] = make([]*vector.Vec2[T], len(points))
	for i := range points {
		pointers[i] = vector.NewVec2(points[i].X, points[i].Y)
	}
	shape = &Polygon2[T]{Polygon: poly.NewPolygon(pointers, vector.Vec2_0[T](), scale, 0), Scale: scale}
	shape.area = polygonAbsoluteArea(shape.Polygon.Reference) * scale * scale
	shape.width, shape.height = shape.Polygon.AABB.X2-shape.Polygon.AABB.X1, shape.Polygon.AABB.Y2-shape.Polygon.AABB.Y1
	return
}

// Transform updates the polygon collider's world-space geometry.
func (p *Polygon2[T]) Transform(position vector.Vec2[T], rotation T) {
	p.Polygon.Transform(&position, p.Scale, rotation)
}

// GetAABB returns the polygon's cached bounds.
func (p *Polygon2[T]) GetAABB() (aabb *hshg.AABB2[T]) {
	aabb = p.Polygon.AABB
	return
}

// Area returns the scaled polygon area.
func (p *Polygon2[T]) Area() (area T) {
	area = p.area
	return
}

// MomentOfInertia approximates polygon inertia from its local axis-aligned bounds.
func (p *Polygon2[T]) MomentOfInertia(mass T) (inertia T) {
	inertia = mass * (p.width*p.width + p.height*p.height) / 12
	return
}

// Clone returns an independent polygon collider with the same local geometry and scale.
func (p *Polygon2[T]) Clone() (clone Shape2[T]) {
	var points []vector.Vec2[T] = make([]vector.Vec2[T], len(p.Polygon.Reference))
	for i := range points {
		points[i] = *p.Polygon.Reference[i]
	}
	clone = NewPolygon2(points, p.Scale)
	return
}

// polygonAbsoluteArea returns the unsigned area of local polygon vertices.
func polygonAbsoluteArea[T constraints.Float](points []*vector.Vec2[T]) (area T) {
	for i := range points {
		var next int = (i + 1) % len(points)
		area += points[i].X*points[next].Y - points[next].X*points[i].Y
	}
	if area < 0 {
		area = -area
	}
	area /= 2
	return
}
