package poly

import (
	"math"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

// TwoPolyhedraIntersect reports whether two convex or concave closed meshes overlap or touch.
func TwoPolyhedraIntersect[T constraints.Float](first, second *Polyhedron[T]) (intersects bool) {
	if !first.AABB.Intersects(second.AABB) {
		return
	}
	if bvhTrianglesIntersect(first, first.bvhRoot, second, second.bvhRoot, nil) {
		intersects = true
		return
	}

	intersects = first.PointIsInside(&second.Points[0]) || second.PointIsInside(&first.Points[0])
	return
}

// GetPolyhedronContactManifold returns surface crossings and contained-vertex penetration contacts.
func GetPolyhedronContactManifold[T constraints.Float](first, second *Polyhedron[T]) (manifold PolyhedronManifold[T]) {
	if !first.AABB.Intersects(second.AABB) {
		return
	}

	var points []vector.Vec3[T]
	bvhTrianglesIntersect(first, first.bvhRoot, second, second.bvhRoot, &points)
	var (
		firstCenter  vector.Vec3[T] = first.Centroid()
		secondCenter vector.Vec3[T] = second.Centroid()
		direction    vector.Vec3[T] = vector.Vec3[T]{X: secondCenter.X - firstCenter.X, Y: secondCenter.Y - firstCenter.Y, Z: secondCenter.Z - firstCenter.Z}
	)
	if direction.SquaredLength() != 0 {
		direction.Normalize()
	}
	for i := range points {
		var (
			firstNormal  vector.Vec3[T] = first.surfaceNormalAt(points[i])
			secondNormal vector.Vec3[T] = second.surfaceNormalAt(points[i])
			normal       vector.Vec3[T] = vector.Vec3[T]{X: firstNormal.X - secondNormal.X, Y: firstNormal.Y - secondNormal.Y, Z: firstNormal.Z - secondNormal.Z}
		)
		if normal.SquaredLength() == 0 {
			normal = direction
		} else {
			normal.Normalize()
			if normal.Dot(&direction) < 0 {
				normal.Mul(-1)
			}
		}
		appendPolyhedronContact(&manifold.Contacts, PolyhedronContact[T]{Point: points[i], Normal: normal})
	}

	appendContainedVertexContacts(first, second, false, &manifold.Contacts)
	appendContainedVertexContacts(second, first, true, &manifold.Contacts)
	appendInteriorPointContact(first.Centroid(), second, false, &manifold.Contacts)
	appendInteriorPointContact(second.Centroid(), first, true, &manifold.Contacts)
	return
}

// surfaceNormalAt returns the normal of the closest cached surface triangle.
func (p *Polyhedron[T]) surfaceNormalAt(point vector.Vec3[T]) (normal vector.Vec3[T]) {
	var bestDistance T = T(math.Inf(1))
	for i := range p.faces {
		var (
			closest    vector.Vec3[T] = closestPointOnTriangle(point, p.faces[i].A, p.faces[i].B, p.faces[i].C)
			dx, dy, dz T              = closest.X - point.X, closest.Y - point.Y, closest.Z - point.Z
			distance   T              = dx*dx + dy*dy + dz*dz
		)
		if distance < bestDistance {
			bestDistance = distance
			normal = p.faces[i].Normal
		}
	}
	return
}

// appendContainedVertexContacts adds penetration estimates for vertices strictly inside another mesh.
func appendContainedVertexContacts[T constraints.Float](source, container *Polyhedron[T], flip bool, contacts *[]PolyhedronContact[T]) {
	for i := range source.Points {
		var vertex *vector.Vec3[T] = &source.Points[i]
		if !container.PointIsInside(vertex) {
			continue
		}

		var (
			closest vector.Vec3[T] = container.ClosestPoint(vertex)
			normal  vector.Vec3[T] = vector.Vec3[T]{X: vertex.X - closest.X, Y: vertex.Y - closest.Y, Z: vertex.Z - closest.Z}
			depth   T              = normal.Length()
		)
		if depth <= polyhedronTolerance(container) {
			continue
		}
		normal.Mul(1 / depth)
		if flip {
			normal.Mul(-1)
		}
		appendPolyhedronContact(contacts, PolyhedronContact[T]{Point: closest, Normal: normal, Penetration: depth})
	}
	for i := range source.faces {
		appendInteriorPointContact(vector.Vec3[T]{
			X: (source.faces[i].A.X + source.faces[i].B.X + source.faces[i].C.X) / 3,
			Y: (source.faces[i].A.Y + source.faces[i].B.Y + source.faces[i].C.Y) / 3,
			Z: (source.faces[i].A.Z + source.faces[i].B.Z + source.faces[i].C.Z) / 3,
		}, container, flip, contacts)
	}
}

// appendInteriorPointContact estimates penetration when a representative interior point is enclosed.
func appendInteriorPointContact[T constraints.Float](point vector.Vec3[T], container *Polyhedron[T], flip bool, contacts *[]PolyhedronContact[T]) {
	if !container.PointIsInside(&point) {
		return
	}

	var (
		closest vector.Vec3[T] = container.ClosestPoint(&point)
		normal  vector.Vec3[T] = vector.Vec3[T]{X: point.X - closest.X, Y: point.Y - closest.Y, Z: point.Z - closest.Z}
		depth   T              = normal.Length()
	)
	if depth <= polyhedronTolerance(container) {
		return
	}
	normal.Mul(1 / depth)
	if flip {
		normal.Mul(-1)
	}
	appendPolyhedronContact(contacts, PolyhedronContact[T]{Point: closest, Normal: normal, Penetration: depth})
}

// appendPolyhedronContact adds a contact unless an equivalent point is already present.
func appendPolyhedronContact[T constraints.Float](contacts *[]PolyhedronContact[T], contact PolyhedronContact[T]) {
	var tolerance T = 1e-8
	for i := range *contacts {
		var (
			dx T = (*contacts)[i].Point.X - contact.Point.X
			dy T = (*contacts)[i].Point.Y - contact.Point.Y
			dz T = (*contacts)[i].Point.Z - contact.Point.Z
		)
		if dx*dx+dy*dy+dz*dz <= tolerance*tolerance {
			if contact.Penetration > (*contacts)[i].Penetration {
				(*contacts)[i] = contact
			}
			return
		}
	}
	*contacts = append(*contacts, contact)
}

// bvhTrianglesIntersect traverses two BVHs and optionally collects triangle contact points.
func bvhTrianglesIntersect[T constraints.Float](first *Polyhedron[T], firstNode int, second *Polyhedron[T], secondNode int, points *[]vector.Vec3[T]) (intersects bool) {
	if firstNode < 0 || secondNode < 0 || !first.bvh[firstNode].bounds.Intersects(&second.bvh[secondNode].bounds) {
		return
	}

	var (
		left  *polyhedronBVHNode[T] = &first.bvh[firstNode]
		right *polyhedronBVHNode[T] = &second.bvh[secondNode]
	)
	if left.count != 0 && right.count != 0 {
		for i := left.start; i < left.start+left.count; i++ {
			var firstFace *PolyhedronTriangle[T] = &first.faces[first.faceOrder[i]]
			for j := right.start; j < right.start+right.count; j++ {
				var secondFace *PolyhedronTriangle[T] = &second.faces[second.faceOrder[j]]
				if !firstFace.AABB.Intersects(&secondFace.AABB) {
					continue
				}
				var contacts []vector.Vec3[T] = triangleContacts(*firstFace, *secondFace)
				if len(contacts) == 0 {
					continue
				}
				intersects = true
				if points == nil {
					return
				}
				for k := range contacts {
					appendUniquePoint(points, contacts[k])
				}
			}
		}
		return
	}

	if left.count == 0 && (right.count != 0 || polyhedronBoundsVolume(left.bounds) >= polyhedronBoundsVolume(right.bounds)) {
		intersects = bvhTrianglesIntersect(first, left.left, second, secondNode, points)
		if points == nil && intersects {
			return
		}
		intersects = bvhTrianglesIntersect(first, left.right, second, secondNode, points) || intersects
	} else {
		intersects = bvhTrianglesIntersect(first, firstNode, second, right.left, points)
		if points == nil && intersects {
			return
		}
		intersects = bvhTrianglesIntersect(first, firstNode, second, right.right, points) || intersects
	}
	return
}

// polyhedronBoundsVolume returns the volume used to choose which BVH node to split.
func polyhedronBoundsVolume[T constraints.Float](bounds hshg.AABB3[T]) (volume float64) {
	volume = float64((bounds.X2 - bounds.X1) * (bounds.Y2 - bounds.Y1) * (bounds.Z2 - bounds.Z1))
	return
}

// triangleContacts returns all unique edge and coplanar contacts between two triangles.
func triangleContacts[T constraints.Float](first, second PolyhedronTriangle[T]) (contacts []vector.Vec3[T]) {
	var (
		firstVertices  [3]vector.Vec3[T] = [3]vector.Vec3[T]{first.A, first.B, first.C}
		secondVertices [3]vector.Vec3[T] = [3]vector.Vec3[T]{second.A, second.B, second.C}
	)
	for i := range firstVertices {
		appendSegmentTriangleContact(&contacts, firstVertices[i], firstVertices[(i+1)%3], second)
		appendSegmentTriangleContact(&contacts, secondVertices[i], secondVertices[(i+1)%3], first)
	}

	if len(contacts) == 0 && trianglesCoplanar(first, second) {
		for i := range firstVertices {
			if pointInTriangle3(firstVertices[i], second, 1e-9) {
				appendUniquePoint(&contacts, firstVertices[i])
			}
			if pointInTriangle3(secondVertices[i], first, 1e-9) {
				appendUniquePoint(&contacts, secondVertices[i])
			}
			for j := range secondVertices {
				var (
					contact vector.Vec3[T]
					hit     bool
				)
				contact, hit = coplanarSegmentsIntersect(firstVertices[i], firstVertices[(i+1)%3], secondVertices[j], secondVertices[(j+1)%3], first.Normal)
				if hit {
					appendUniquePoint(&contacts, contact)
				}
			}
		}
	}
	return
}

// coplanarSegmentsIntersect projects onto the strongest plane and intersects two segments.
func coplanarSegmentsIntersect[T constraints.Float](a, b, c, d, normal vector.Vec3[T]) (point vector.Vec3[T], intersects bool) {
	var (
		nx, ny, nz     float64 = math.Abs(float64(normal.X)), math.Abs(float64(normal.Y)), math.Abs(float64(normal.Z))
		a1, a2, b1, b2 T
		c1, c2, d1, d2 T
	)
	if nx >= ny && nx >= nz {
		a1, a2, b1, b2, c1, c2, d1, d2 = a.Y, a.Z, b.Y, b.Z, c.Y, c.Z, d.Y, d.Z
	} else if ny >= nz {
		a1, a2, b1, b2, c1, c2, d1, d2 = a.X, a.Z, b.X, b.Z, c.X, c.Z, d.X, d.Z
	} else {
		a1, a2, b1, b2, c1, c2, d1, d2 = a.X, a.Y, b.X, b.Y, c.X, c.Y, d.X, d.Y
	}

	var denominator T = (b1-a1)*(d2-c2) - (b2-a2)*(d1-c1)
	if math.Abs(float64(denominator)) <= 1e-12 {
		return
	}
	var (
		t T = ((c1-a1)*(d2-c2) - (c2-a2)*(d1-c1)) / denominator
		u T = ((c1-a1)*(b2-a2) - (c2-a2)*(b1-a1)) / denominator
	)
	if t < 0 || t > 1 || u < 0 || u > 1 {
		return
	}
	point = vector.Vec3[T]{X: a.X + (b.X-a.X)*t, Y: a.Y + (b.Y-a.Y)*t, Z: a.Z + (b.Z-a.Z)*t}
	intersects = true
	return
}

// appendSegmentTriangleContact adds a segment's contact with a triangle.
func appendSegmentTriangleContact[T constraints.Float](contacts *[]vector.Vec3[T], start, end vector.Vec3[T], triangle PolyhedronTriangle[T]) {
	var (
		direction vector.Vec3[T] = vector.Vec3[T]{X: end.X - start.X, Y: end.Y - start.Y, Z: end.Z - start.Z}
		parameter T
		hit       bool
	)
	parameter, hit = rayIntersectsTriangle(start, direction, triangle.A, triangle.B, triangle.C)
	if hit && parameter <= 1 {
		appendUniquePoint(contacts, vector.Vec3[T]{X: start.X + direction.X*parameter, Y: start.Y + direction.Y*parameter, Z: start.Z + direction.Z*parameter})
	}
}

// trianglesCoplanar reports whether two triangles lie on the same plane.
func trianglesCoplanar[T constraints.Float](first, second PolyhedronTriangle[T]) (coplanar bool) {
	var (
		dx, dy, dz T       = second.A.X - first.A.X, second.A.Y - first.A.Y, second.A.Z - first.A.Z
		distance   float64 = math.Abs(float64(dx*first.Normal.X + dy*first.Normal.Y + dz*first.Normal.Z))
		alignment  float64 = math.Abs(float64(first.Normal.X*second.Normal.X + first.Normal.Y*second.Normal.Y + first.Normal.Z*second.Normal.Z))
	)
	coplanar = distance <= 1e-9 && alignment >= 1-1e-9
	return
}

// pointInTriangle3 reports whether a coplanar point lies in a triangle using barycentric coordinates.
func pointInTriangle3[T constraints.Float](point vector.Vec3[T], triangle PolyhedronTriangle[T], tolerance T) (inside bool) {
	var (
		v0                  vector.Vec3[T] = vector.Vec3[T]{X: triangle.C.X - triangle.A.X, Y: triangle.C.Y - triangle.A.Y, Z: triangle.C.Z - triangle.A.Z}
		v1                  vector.Vec3[T] = vector.Vec3[T]{X: triangle.B.X - triangle.A.X, Y: triangle.B.Y - triangle.A.Y, Z: triangle.B.Z - triangle.A.Z}
		v2                  vector.Vec3[T] = vector.Vec3[T]{X: point.X - triangle.A.X, Y: point.Y - triangle.A.Y, Z: point.Z - triangle.A.Z}
		dot00, dot01, dot02 T              = v0.Dot(&v0), v0.Dot(&v1), v0.Dot(&v2)
		dot11, dot12        T              = v1.Dot(&v1), v1.Dot(&v2)
		denominator         T              = dot00*dot11 - dot01*dot01
	)
	if denominator == 0 {
		return
	}
	var (
		u T = (dot11*dot02 - dot01*dot12) / denominator
		v T = (dot00*dot12 - dot01*dot02) / denominator
	)
	inside = u >= -tolerance && v >= -tolerance && u+v <= 1+tolerance
	return
}

// appendUniquePoint deduplicates contact points shared by adjacent triangles.
func appendUniquePoint[T constraints.Float](points *[]vector.Vec3[T], point vector.Vec3[T]) {
	const tolerance float64 = 1e-8
	for i := range *points {
		var (
			dx float64 = float64((*points)[i].X - point.X)
			dy float64 = float64((*points)[i].Y - point.Y)
			dz float64 = float64((*points)[i].Z - point.Z)
		)
		if dx*dx+dy*dy+dz*dz <= tolerance*tolerance {
			return
		}
	}
	*points = append(*points, point)
}

// polyhedronTolerance returns a scale-aware tolerance for surface classification.
func polyhedronTolerance[T constraints.Float](polyhedron *Polyhedron[T]) (tolerance T) {
	var (
		dx T = polyhedron.AABB.X2 - polyhedron.AABB.X1
		dy T = polyhedron.AABB.Y2 - polyhedron.AABB.Y1
		dz T = polyhedron.AABB.Z2 - polyhedron.AABB.Z1
	)
	tolerance = T(math.Sqrt(float64(dx*dx+dy*dy+dz*dz))) * 1e-9
	if tolerance < 1e-12 {
		tolerance = 1e-12
	}
	return
}
