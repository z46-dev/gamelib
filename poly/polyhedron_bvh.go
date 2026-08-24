package poly

import (
	"math"
	"sort"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

const polyhedronBVHLeafSize = 4

// rebuildAccelerationData caches transformed triangles and builds their bounding-volume hierarchy.
func (p *Polyhedron[T]) rebuildAccelerationData() {
	p.faces = make([]PolyhedronTriangle[T], len(p.Triangles))
	p.faceOrder = make([]int, len(p.Triangles))
	for i := range p.Triangles {
		var (
			triangle Triangle3              = p.Triangles[i]
			face     *PolyhedronTriangle[T] = &p.faces[i]
			edge1    vector.Vec3[T]
			edge2    vector.Vec3[T]
		)
		face.A, face.B, face.C = p.Points[triangle.A], p.Points[triangle.B], p.Points[triangle.C]
		face.AABB = hshg.AABB3[T]{
			X1: min(face.A.X, face.B.X, face.C.X), Y1: min(face.A.Y, face.B.Y, face.C.Y), Z1: min(face.A.Z, face.B.Z, face.C.Z),
			X2: max(face.A.X, face.B.X, face.C.X), Y2: max(face.A.Y, face.B.Y, face.C.Y), Z2: max(face.A.Z, face.B.Z, face.C.Z),
		}
		edge1 = vector.Vec3[T]{X: face.B.X - face.A.X, Y: face.B.Y - face.A.Y, Z: face.B.Z - face.A.Z}
		edge2 = vector.Vec3[T]{X: face.C.X - face.A.X, Y: face.C.Y - face.A.Y, Z: face.C.Z - face.A.Z}
		edge1.Cross(&edge2)
		face.Area = edge1.Length() / 2
		if edge1.SquaredLength() != 0 {
			edge1.Normalize()
		}
		face.Normal = edge1
		p.faceOrder[i] = i
	}

	p.bvh = p.bvh[:0]
	if len(p.faceOrder) == 0 {
		p.bvhRoot = -1
		return
	}
	p.bvhRoot = p.buildBVHNode(0, len(p.faceOrder))
}

// buildBVHNode recursively partitions cached faces along their widest centroid axis.
func (p *Polyhedron[T]) buildBVHNode(start, end int) (index int) {
	var bounds hshg.AABB3[T] = hshg.AABB3[T]{X1: T(math.Inf(1)), Y1: T(math.Inf(1)), Z1: T(math.Inf(1)), X2: T(math.Inf(-1)), Y2: T(math.Inf(-1)), Z2: T(math.Inf(-1))}
	for i := start; i < end; i++ {
		var face *PolyhedronTriangle[T] = &p.faces[p.faceOrder[i]]
		bounds.X1, bounds.Y1, bounds.Z1 = min(bounds.X1, face.AABB.X1), min(bounds.Y1, face.AABB.Y1), min(bounds.Z1, face.AABB.Z1)
		bounds.X2, bounds.Y2, bounds.Z2 = max(bounds.X2, face.AABB.X2), max(bounds.Y2, face.AABB.Y2), max(bounds.Z2, face.AABB.Z2)
	}

	index = len(p.bvh)
	p.bvh = append(p.bvh, polyhedronBVHNode[T]{bounds: bounds, left: -1, right: -1, start: start, count: end - start})
	if end-start <= polyhedronBVHLeafSize {
		return
	}

	var (
		xSize T = bounds.X2 - bounds.X1
		ySize T = bounds.Y2 - bounds.Y1
		zSize T = bounds.Z2 - bounds.Z1
		axis  int
	)
	if ySize > xSize {
		axis = 1
		xSize = ySize
	}
	if zSize > xSize {
		axis = 2
	}
	sort.Slice(p.faceOrder[start:end], func(i, j int) bool {
		var (
			left  *PolyhedronTriangle[T] = &p.faces[p.faceOrder[start+i]]
			right *PolyhedronTriangle[T] = &p.faces[p.faceOrder[start+j]]
		)
		if axis == 0 {
			return left.A.X+left.B.X+left.C.X < right.A.X+right.B.X+right.C.X
		}
		if axis == 1 {
			return left.A.Y+left.B.Y+left.C.Y < right.A.Y+right.B.Y+right.C.Y
		}
		return left.A.Z+left.B.Z+left.C.Z < right.A.Z+right.B.Z+right.C.Z
	})

	var middle int = start + (end-start)/2
	p.bvh[index].left = p.buildBVHNode(start, middle)
	p.bvh[index].right = p.buildBVHNode(middle, end)
	p.bvh[index].count = 0
	return
}

// ConvexParts returns the mesh's exact decomposition into convex triangle primitives.
// Triangles are surface primitives rather than volumetric tetrahedra, so this works for arbitrary concave topology.
func (p *Polyhedron[T]) ConvexParts() (parts []PolyhedronTriangle[T]) {
	parts = append([]PolyhedronTriangle[T](nil), p.faces...)
	return
}

// BVHNodeCount returns the number of nodes in the cached triangle hierarchy.
func (p *Polyhedron[T]) BVHNodeCount() (count int) {
	count = len(p.bvh)
	return
}

// closestPointBVH searches only nodes whose bounds can improve the current result.
func (p *Polyhedron[T]) closestPointBVH(nodeIndex int, point vector.Vec3[T], closest *vector.Vec3[T], bestDistance *T) {
	if nodeIndex < 0 || squaredDistanceToAABB(point, p.bvh[nodeIndex].bounds) > *bestDistance {
		return
	}

	var node *polyhedronBVHNode[T] = &p.bvh[nodeIndex]
	if node.count != 0 {
		for i := node.start; i < node.start+node.count; i++ {
			var (
				face       *PolyhedronTriangle[T] = &p.faces[p.faceOrder[i]]
				candidate  vector.Vec3[T]         = closestPointOnTriangle(point, face.A, face.B, face.C)
				dx, dy, dz T                      = candidate.X - point.X, candidate.Y - point.Y, candidate.Z - point.Z
				distance   T                      = dx*dx + dy*dy + dz*dz
			)
			if distance < *bestDistance {
				*bestDistance = distance
				*closest = candidate
			}
		}
		return
	}

	var (
		leftDistance  T = squaredDistanceToAABB(point, p.bvh[node.left].bounds)
		rightDistance T = squaredDistanceToAABB(point, p.bvh[node.right].bounds)
	)
	if leftDistance <= rightDistance {
		p.closestPointBVH(node.left, point, closest, bestDistance)
		p.closestPointBVH(node.right, point, closest, bestDistance)
	} else {
		p.closestPointBVH(node.right, point, closest, bestDistance)
		p.closestPointBVH(node.left, point, closest, bestDistance)
	}
}

// rayIntersectsBVH searches the cached hierarchy for the nearest triangle hit.
func (p *Polyhedron[T]) rayIntersectsBVH(nodeIndex int, origin, direction vector.Vec3[T], bestDistance *T) (intersects bool) {
	if nodeIndex < 0 || !rayIntersectsAABB(origin, direction, p.bvh[nodeIndex].bounds, *bestDistance) {
		return
	}

	var node *polyhedronBVHNode[T] = &p.bvh[nodeIndex]
	if node.count != 0 {
		for i := node.start; i < node.start+node.count; i++ {
			var (
				face     *PolyhedronTriangle[T] = &p.faces[p.faceOrder[i]]
				distance T
				hit      bool
			)
			distance, hit = rayIntersectsTriangle(origin, direction, face.A, face.B, face.C)
			if hit && distance < *bestDistance {
				*bestDistance = distance
				intersects = true
			}
		}
		return
	}

	intersects = p.rayIntersectsBVH(node.left, origin, direction, bestDistance)
	intersects = p.rayIntersectsBVH(node.right, origin, direction, bestDistance) || intersects
	return
}

// squaredDistanceToAABB returns zero inside bounds and squared distance outside them.
func squaredDistanceToAABB[T constraints.Float](point vector.Vec3[T], bounds hshg.AABB3[T]) (distance T) {
	var dx, dy, dz T
	if point.X < bounds.X1 {
		dx = bounds.X1 - point.X
	} else if point.X > bounds.X2 {
		dx = point.X - bounds.X2
	}
	if point.Y < bounds.Y1 {
		dy = bounds.Y1 - point.Y
	} else if point.Y > bounds.Y2 {
		dy = point.Y - bounds.Y2
	}
	if point.Z < bounds.Z1 {
		dz = bounds.Z1 - point.Z
	} else if point.Z > bounds.Z2 {
		dz = point.Z - bounds.Z2
	}
	distance = dx*dx + dy*dy + dz*dz
	return
}

// rayIntersectsAABB applies a slab test capped by the current closest hit.
func rayIntersectsAABB[T constraints.Float](origin, direction vector.Vec3[T], bounds hshg.AABB3[T], maximum T) (intersects bool) {
	var minimum T
	if !rayAxisInterval(origin.X, direction.X, bounds.X1, bounds.X2, &minimum, &maximum) ||
		!rayAxisInterval(origin.Y, direction.Y, bounds.Y1, bounds.Y2, &minimum, &maximum) ||
		!rayAxisInterval(origin.Z, direction.Z, bounds.Z1, bounds.Z2, &minimum, &maximum) {
		return
	}
	intersects = maximum >= max(minimum, 0)
	return
}

// rayAxisInterval clips a ray interval against one AABB slab.
func rayAxisInterval[T constraints.Float](origin, direction, lower, upper T, minimum, maximum *T) (intersects bool) {
	if direction == 0 {
		intersects = origin >= lower && origin <= upper
		return
	}
	var first, second T = (lower - origin) / direction, (upper - origin) / direction
	if first > second {
		first, second = second, first
	}
	*minimum, *maximum = max(*minimum, first), min(*maximum, second)
	intersects = *minimum <= *maximum
	return
}
