package poly

import (
	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	// PolygonXYPoint stores an internal two-dimensional point as scalar components.
	PolygonXYPoint[T constraints.Float] struct {
		x, y T
	}

	// Polygon represents a transformed 2D polygon and its cached convex decomposition.
	Polygon[T constraints.Float] struct {
		numPoints              int
		Reference, Points      []*vector.Vec2[T]
		x, y, radius, rotation T
		AABB                   *hshg.AABB2[T]
		parts                  []*Polygon[T]
		axes                   []vector.Vec2[T]
		transformed            bool
	}

	// Triangle3 identifies one consistently wound triangle in a polyhedron mesh.
	Triangle3 struct {
		A, B, C int
	}

	// PolyhedronTransform describes scale, XYZ Euler rotation in radians, and translation.
	PolyhedronTransform[T constraints.Float] struct {
		Position, Scale, Rotation vector.Vec3[T]
	}

	// PolyhedronTriangle is one convex world-space surface primitive.
	PolyhedronTriangle[T constraints.Float] struct {
		A, B, C vector.Vec3[T]
		Normal  vector.Vec3[T]
		AABB    hshg.AABB3[T]
		Area    T
	}

	// PolyhedronContact describes one point shared by two intersecting solids.
	PolyhedronContact[T constraints.Float] struct {
		Point, Normal vector.Vec3[T]
		Penetration   T
	}

	// PolyhedronManifold contains the deduplicated contacts between two solids.
	PolyhedronManifold[T constraints.Float] struct {
		Contacts []PolyhedronContact[T]
	}

	polyhedronBVHNode[T constraints.Float] struct {
		bounds       hshg.AABB3[T]
		left, right  int
		start, count int
	}

	// Polyhedron represents a closed triangle mesh and its cached world-space geometry.
	// Its topology may describe either a convex or concave solid.
	Polyhedron[T constraints.Float] struct {
		Reference []vector.Vec3[T]
		Points    []vector.Vec3[T]
		Triangles []Triangle3
		AABB      *hshg.AABB3[T]

		transform   PolyhedronTransform[T]
		transformed bool
		faces       []PolyhedronTriangle[T]
		faceOrder   []int
		bvh         []polyhedronBVHNode[T]
		bvhRoot     int
	}
)
