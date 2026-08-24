package poly

import (
	"fmt"
	"math"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	polyhedronEdge struct {
		low, high int
	}

	polyhedronEdgeUse struct {
		count, direction int
	}
)

// IdentityPolyhedronTransform returns a transform with unit scale and no rotation or translation.
func IdentityPolyhedronTransform[T constraints.Float]() (transform PolyhedronTransform[T]) {
	transform.Scale = vector.Vec3[T]{X: 1, Y: 1, Z: 1}
	return
}

// NewPolyhedron validates and copies a closed, consistently wound triangle mesh.
func NewPolyhedron[T constraints.Float](vertices []vector.Vec3[T], triangles []Triangle3) (polyhedron *Polyhedron[T], err error) {
	if err = validatePolyhedron(vertices, triangles); err != nil {
		return
	}

	polyhedron = &Polyhedron[T]{
		Reference: append([]vector.Vec3[T](nil), vertices...),
		Points:    make([]vector.Vec3[T], len(vertices)),
		Triangles: append([]Triangle3(nil), triangles...),
		AABB:      &hshg.AABB3[T]{},
	}
	polyhedron.Transform(IdentityPolyhedronTransform[T]())
	return
}

// validatePolyhedron verifies indices, triangle area, and closed oriented manifold topology.
func validatePolyhedron[T constraints.Float](vertices []vector.Vec3[T], triangles []Triangle3) (err error) {
	if len(vertices) < 4 {
		err = fmt.Errorf("poly: polyhedron requires at least four vertices")
		return
	}
	if len(triangles) < 4 {
		err = fmt.Errorf("poly: polyhedron requires at least four triangles")
		return
	}

	var edges map[polyhedronEdge]polyhedronEdgeUse = make(map[polyhedronEdge]polyhedronEdgeUse, len(triangles)*3/2)
	for i := range triangles {
		var triangle Triangle3 = triangles[i]
		if triangle.A < 0 || triangle.A >= len(vertices) || triangle.B < 0 || triangle.B >= len(vertices) || triangle.C < 0 || triangle.C >= len(vertices) {
			err = fmt.Errorf("poly: triangle %d contains an out-of-range vertex index", i)
			return
		}
		if triangle.A == triangle.B || triangle.B == triangle.C || triangle.C == triangle.A {
			err = fmt.Errorf("poly: triangle %d repeats a vertex", i)
			return
		}

		var (
			a  vector.Vec3[T] = vertices[triangle.A]
			b  vector.Vec3[T] = vertices[triangle.B]
			c  vector.Vec3[T] = vertices[triangle.C]
			ab vector.Vec3[T] = vector.Vec3[T]{X: b.X - a.X, Y: b.Y - a.Y, Z: b.Z - a.Z}
			ac vector.Vec3[T] = vector.Vec3[T]{X: c.X - a.X, Y: c.Y - a.Y, Z: c.Z - a.Z}
		)
		ab.Cross(&ac)
		if ab.SquaredLength() == 0 {
			err = fmt.Errorf("poly: triangle %d is degenerate", i)
			return
		}

		addPolyhedronEdge(edges, triangle.A, triangle.B)
		addPolyhedronEdge(edges, triangle.B, triangle.C)
		addPolyhedronEdge(edges, triangle.C, triangle.A)
	}

	for edge, use := range edges {
		if use.count != 2 || use.direction != 0 {
			err = fmt.Errorf("poly: edge %d-%d is not shared by two oppositely wound triangles", edge.low, edge.high)
			return
		}
	}

	return
}

// addPolyhedronEdge records an oriented edge for closed-manifold validation.
func addPolyhedronEdge(edges map[polyhedronEdge]polyhedronEdgeUse, start, end int) {
	var (
		edge      polyhedronEdge
		use       polyhedronEdgeUse
		direction int = 1
	)
	if start < end {
		edge = polyhedronEdge{low: start, high: end}
	} else {
		edge = polyhedronEdge{low: end, high: start}
		direction = -1
	}

	use = edges[edge]
	use.count++
	use.direction += direction
	edges[edge] = use
}

// Transform updates world-space vertices and bounds using scale, then X/Y/Z rotation, then translation.
func (p *Polyhedron[T]) Transform(transform PolyhedronTransform[T]) {
	if p.transformed && p.transform == transform {
		return
	}

	var (
		rotationX  float64 = float64(transform.Rotation.X)
		rotationY  float64 = float64(transform.Rotation.Y)
		rotationZ  float64 = float64(transform.Rotation.Z)
		cosX, sinX T       = T(math.Cos(rotationX)), T(math.Sin(rotationX))
		cosY, sinY T       = T(math.Cos(rotationY)), T(math.Sin(rotationY))
		cosZ, sinZ T       = T(math.Cos(rotationZ)), T(math.Sin(rotationZ))
	)
	for i := range p.Reference {
		var (
			x T = p.Reference[i].X * transform.Scale.X
			y T = p.Reference[i].Y * transform.Scale.Y
			z T = p.Reference[i].Z * transform.Scale.Z
		)
		y, z = y*cosX-z*sinX, y*sinX+z*cosX
		x, z = x*cosY+z*sinY, -x*sinY+z*cosY
		x, y = x*cosZ-y*sinZ, x*sinZ+y*cosZ
		p.Points[i] = vector.Vec3[T]{X: x + transform.Position.X, Y: y + transform.Position.Y, Z: z + transform.Position.Z}
	}

	p.transform = transform
	p.transformed = true
	p.updateAABB()
	p.rebuildAccelerationData()
}

// updateAABB refreshes the polyhedron's world-space bounds.
func (p *Polyhedron[T]) updateAABB() {
	var (
		minimum vector.Vec3[T] = vector.Vec3[T]{X: T(math.Inf(1)), Y: T(math.Inf(1)), Z: T(math.Inf(1))}
		maximum vector.Vec3[T] = vector.Vec3[T]{X: T(math.Inf(-1)), Y: T(math.Inf(-1)), Z: T(math.Inf(-1))}
	)
	for i := range p.Points {
		minimum.X, minimum.Y, minimum.Z = min(minimum.X, p.Points[i].X), min(minimum.Y, p.Points[i].Y), min(minimum.Z, p.Points[i].Z)
		maximum.X, maximum.Y, maximum.Z = max(maximum.X, p.Points[i].X), max(maximum.Y, p.Points[i].Y), max(maximum.Z, p.Points[i].Z)
	}

	p.AABB.X1, p.AABB.Y1, p.AABB.Z1 = minimum.X, minimum.Y, minimum.Z
	p.AABB.X2, p.AABB.Y2, p.AABB.Z2 = maximum.X, maximum.Y, maximum.Z
}

// GetAABB returns the cached bounds used by SpatialHash3.
func (p *Polyhedron[T]) GetAABB() (aabb *hshg.AABB3[T]) {
	aabb = p.AABB
	return
}

// GetTransform returns the transform most recently applied to the polyhedron.
func (p *Polyhedron[T]) GetTransform() (transform PolyhedronTransform[T]) {
	transform = p.transform
	return
}

// VertexCount returns the number of unique mesh vertices.
func (p *Polyhedron[T]) VertexCount() (count int) {
	count = len(p.Reference)
	return
}

// TriangleCount returns the number of indexed mesh triangles.
func (p *Polyhedron[T]) TriangleCount() (count int) {
	count = len(p.Triangles)
	return
}

// ProjectOnto projects all world-space vertices onto an axis.
func (p *Polyhedron[T]) ProjectOnto(axis *vector.Vec3[T]) (minimum, maximum T) {
	minimum, maximum = T(math.Inf(1)), T(math.Inf(-1))
	for i := range p.Points {
		var dot T = p.Points[i].X*axis.X + p.Points[i].Y*axis.Y + p.Points[i].Z*axis.Z
		minimum, maximum = min(minimum, dot), max(maximum, dot)
	}

	return
}

// FaceNormal returns the normalized world-space normal of one triangle.
func (p *Polyhedron[T]) FaceNormal(index int) (normal vector.Vec3[T], ok bool) {
	if index < 0 || index >= len(p.faces) || p.faces[index].Normal.SquaredLength() == 0 {
		return
	}
	normal, ok = p.faces[index].Normal, true
	return
}

// SurfaceArea returns the total world-space area of the triangle mesh.
func (p *Polyhedron[T]) SurfaceArea() (area T) {
	for i := range p.faces {
		area += p.faces[i].Area
	}

	return
}

// SignedVolume returns the oriented world-space volume of the closed mesh.
func (p *Polyhedron[T]) SignedVolume() (volume T) {
	for i := range p.Triangles {
		var (
			triangle Triangle3      = p.Triangles[i]
			a        vector.Vec3[T] = p.Points[triangle.A]
			b        vector.Vec3[T] = p.Points[triangle.B]
			c        vector.Vec3[T] = p.Points[triangle.C]
		)
		volume += (a.X*(b.Y*c.Z-b.Z*c.Y) + a.Y*(b.Z*c.X-b.X*c.Z) + a.Z*(b.X*c.Y-b.Y*c.X)) / 6
	}

	return
}

// Volume returns the non-negative world-space volume of the closed mesh.
func (p *Polyhedron[T]) Volume() (volume T) {
	volume = p.SignedVolume()
	if volume < 0 {
		volume = -volume
	}

	return
}

// Centroid returns the volume-weighted center of the closed mesh.
func (p *Polyhedron[T]) Centroid() (centroid vector.Vec3[T]) {
	var volume6 T
	for i := range p.Triangles {
		var (
			triangle    Triangle3      = p.Triangles[i]
			a           vector.Vec3[T] = p.Points[triangle.A]
			b           vector.Vec3[T] = p.Points[triangle.B]
			c           vector.Vec3[T] = p.Points[triangle.C]
			determinant T              = a.X*(b.Y*c.Z-b.Z*c.Y) + a.Y*(b.Z*c.X-b.X*c.Z) + a.Z*(b.X*c.Y-b.Y*c.X)
		)
		volume6 += determinant
		centroid.X += (a.X + b.X + c.X) * determinant
		centroid.Y += (a.Y + b.Y + c.Y) * determinant
		centroid.Z += (a.Z + b.Z + c.Z) * determinant
	}

	if volume6 != 0 {
		centroid.X /= volume6 * 4
		centroid.Y /= volume6 * 4
		centroid.Z /= volume6 * 4
	}
	return
}

// ClosestPoint returns the nearest point on the polyhedron surface.
func (p *Polyhedron[T]) ClosestPoint(point *vector.Vec3[T]) (closest vector.Vec3[T]) {
	var closestDistance T = T(math.Inf(1))
	p.closestPointBVH(p.bvhRoot, *point, &closest, &closestDistance)
	return
}

// PointIsInside reports whether a point lies inside or on a closed mesh.
func (p *Polyhedron[T]) PointIsInside(point *vector.Vec3[T]) (inside bool) {
	if point.X < p.AABB.X1 || point.X > p.AABB.X2 || point.Y < p.AABB.Y1 || point.Y > p.AABB.Y2 || point.Z < p.AABB.Z1 || point.Z > p.AABB.Z2 {
		return
	}

	var (
		dx, dy, dz float64        = float64(p.AABB.X2 - p.AABB.X1), float64(p.AABB.Y2 - p.AABB.Y1), float64(p.AABB.Z2 - p.AABB.Z1)
		tolerance  float64        = math.Max((dx*dx+dy*dy+dz*dz)*1e-12, 1e-18)
		closest    vector.Vec3[T] = p.ClosestPoint(point)
		cx, cy, cz float64        = float64(closest.X - point.X), float64(closest.Y - point.Y), float64(closest.Z - point.Z)
	)
	if cx*cx+cy*cy+cz*cz <= tolerance {
		inside = true
		return
	}

	var solidAngle float64
	for i := range p.Triangles {
		var (
			triangle    Triangle3 = p.Triangles[i]
			aX, aY, aZ  float64   = float64(p.Points[triangle.A].X - point.X), float64(p.Points[triangle.A].Y - point.Y), float64(p.Points[triangle.A].Z - point.Z)
			bX, bY, bZ  float64   = float64(p.Points[triangle.B].X - point.X), float64(p.Points[triangle.B].Y - point.Y), float64(p.Points[triangle.B].Z - point.Z)
			cX, cY, cZ  float64   = float64(p.Points[triangle.C].X - point.X), float64(p.Points[triangle.C].Y - point.Y), float64(p.Points[triangle.C].Z - point.Z)
			aLength     float64   = math.Sqrt(aX*aX + aY*aY + aZ*aZ)
			bLength     float64   = math.Sqrt(bX*bX + bY*bY + bZ*bZ)
			cLength     float64   = math.Sqrt(cX*cX + cY*cY + cZ*cZ)
			numerator   float64   = aX*(bY*cZ-bZ*cY) + aY*(bZ*cX-bX*cZ) + aZ*(bX*cY-bY*cX)
			denominator float64   = aLength*bLength*cLength + (aX*bX+aY*bY+aZ*bZ)*cLength + (bX*cX+bY*cY+bZ*cZ)*aLength + (cX*aX+cY*aY+cZ*aZ)*bLength
		)
		solidAngle += 2 * math.Atan2(numerator, denominator)
	}

	inside = math.Abs(solidAngle) > 2*math.Pi
	return
}

// RayIntersects returns the closest non-negative intersection along a ray.
func (p *Polyhedron[T]) RayIntersects(origin, direction *vector.Vec3[T]) (point vector.Vec3[T], distance T, intersects bool) {
	var rayDirection vector.Vec3[T] = *direction
	if rayDirection.SquaredLength() == 0 {
		return
	}
	rayDirection.Normalize()

	distance = T(math.Inf(1))
	intersects = p.rayIntersectsBVH(p.bvhRoot, *origin, rayDirection, &distance)

	if intersects {
		point = vector.Vec3[T]{X: origin.X + rayDirection.X*distance, Y: origin.Y + rayDirection.Y*distance, Z: origin.Z + rayDirection.Z*distance}
	} else {
		distance = 0
	}
	return
}

// LineIntersects reports whether a finite line segment touches the polyhedron surface or interior.
func (p *Polyhedron[T]) LineIntersects(start, end *vector.Vec3[T]) (intersects bool) {
	if p.PointIsInside(start) || p.PointIsInside(end) {
		intersects = true
		return
	}

	var (
		direction vector.Vec3[T] = vector.Vec3[T]{X: end.X - start.X, Y: end.Y - start.Y, Z: end.Z - start.Z}
		length    T              = direction.Length()
		distance  T
	)
	_, distance, intersects = p.RayIntersects(start, &direction)
	intersects = intersects && distance <= length
	return
}

// SphereIntersects reports whether a sphere overlaps the polyhedron interior or surface.
func (p *Polyhedron[T]) SphereIntersects(center *vector.Vec3[T], radius T) (intersects bool) {
	if radius < 0 {
		return
	}
	if p.PointIsInside(center) {
		intersects = true
		return
	}

	var (
		closest    vector.Vec3[T] = p.ClosestPoint(center)
		dx, dy, dz T              = closest.X - center.X, closest.Y - center.Y, closest.Z - center.Z
	)
	intersects = dx*dx+dy*dy+dz*dz <= radius*radius
	return
}

// closestPointOnTriangle returns the closest point on a triangle to a query point.
func closestPointOnTriangle[T constraints.Float](point, a, b, c vector.Vec3[T]) (closest vector.Vec3[T]) {
	var (
		ab     vector.Vec3[T] = vector.Vec3[T]{X: b.X - a.X, Y: b.Y - a.Y, Z: b.Z - a.Z}
		ac     vector.Vec3[T] = vector.Vec3[T]{X: c.X - a.X, Y: c.Y - a.Y, Z: c.Z - a.Z}
		ap     vector.Vec3[T] = vector.Vec3[T]{X: point.X - a.X, Y: point.Y - a.Y, Z: point.Z - a.Z}
		d1, d2 T              = ab.Dot(&ap), ac.Dot(&ap)
	)
	if d1 <= 0 && d2 <= 0 {
		return a
	}

	var (
		bp     vector.Vec3[T] = vector.Vec3[T]{X: point.X - b.X, Y: point.Y - b.Y, Z: point.Z - b.Z}
		d3, d4 T              = ab.Dot(&bp), ac.Dot(&bp)
	)
	if d3 >= 0 && d4 <= d3 {
		return b
	}
	if d1*d4-d3*d2 <= 0 && d1 >= 0 && d3 <= 0 {
		var ratio T = d1 / (d1 - d3)
		return vector.Vec3[T]{X: a.X + ab.X*ratio, Y: a.Y + ab.Y*ratio, Z: a.Z + ab.Z*ratio}
	}

	var (
		cp     vector.Vec3[T] = vector.Vec3[T]{X: point.X - c.X, Y: point.Y - c.Y, Z: point.Z - c.Z}
		d5, d6 T              = ab.Dot(&cp), ac.Dot(&cp)
	)
	if d6 >= 0 && d5 <= d6 {
		return c
	}
	if d5*d2-d1*d6 <= 0 && d2 >= 0 && d6 <= 0 {
		var ratio T = d2 / (d2 - d6)
		return vector.Vec3[T]{X: a.X + ac.X*ratio, Y: a.Y + ac.Y*ratio, Z: a.Z + ac.Z*ratio}
	}
	if (d3*d6-d5*d4) <= 0 && (d4-d3) >= 0 && (d5-d6) >= 0 {
		var (
			bc    vector.Vec3[T] = vector.Vec3[T]{X: c.X - b.X, Y: c.Y - b.Y, Z: c.Z - b.Z}
			ratio T              = (d4 - d3) / ((d4 - d3) + (d5 - d6))
		)
		return vector.Vec3[T]{X: b.X + bc.X*ratio, Y: b.Y + bc.Y*ratio, Z: b.Z + bc.Z*ratio}
	}

	var (
		vc      T = d1*d4 - d3*d2
		vb      T = d5*d2 - d1*d6
		va      T = d3*d6 - d5*d4
		inverse T = 1 / (va + vb + vc)
		v       T = vb * inverse
		w       T = vc * inverse
	)
	closest = vector.Vec3[T]{X: a.X + ab.X*v + ac.X*w, Y: a.Y + ab.Y*v + ac.Y*w, Z: a.Z + ab.Z*v + ac.Z*w}
	return
}

// rayIntersectsTriangle applies the two-sided Moller-Trumbore ray test.
func rayIntersectsTriangle[T constraints.Float](origin, direction, a, b, c vector.Vec3[T]) (distance T, intersects bool) {
	var (
		edge1 vector.Vec3[T] = vector.Vec3[T]{X: b.X - a.X, Y: b.Y - a.Y, Z: b.Z - a.Z}
		edge2 vector.Vec3[T] = vector.Vec3[T]{X: c.X - a.X, Y: c.Y - a.Y, Z: c.Z - a.Z}
		cross vector.Vec3[T] = direction
	)
	cross.Cross(&edge2)
	var determinant T = edge1.Dot(&cross)
	if math.Abs(float64(determinant)) <= 1e-12 {
		return
	}

	var (
		inverse T              = 1 / determinant
		fromA   vector.Vec3[T] = vector.Vec3[T]{X: origin.X - a.X, Y: origin.Y - a.Y, Z: origin.Z - a.Z}
		u       T              = fromA.Dot(&cross) * inverse
	)
	if u < 0 || u > 1 {
		return
	}

	fromA.Cross(&edge1)
	var v T = direction.Dot(&fromA) * inverse
	if v < 0 || u+v > 1 {
		return
	}

	distance = edge2.Dot(&fromA) * inverse
	intersects = distance >= 0
	return
}

// Copy returns a deep copy whose geometry and bounds can be modified independently.
func (p *Polyhedron[T]) Copy() (copy *Polyhedron[T]) {
	copy = &Polyhedron[T]{
		Reference:   append([]vector.Vec3[T](nil), p.Reference...),
		Points:      append([]vector.Vec3[T](nil), p.Points...),
		Triangles:   append([]Triangle3(nil), p.Triangles...),
		AABB:        p.AABB.Copy(),
		transform:   p.transform,
		transformed: p.transformed,
		faces:       append([]PolyhedronTriangle[T](nil), p.faces...),
		faceOrder:   append([]int(nil), p.faceOrder...),
		bvh:         append([]polyhedronBVHNode[T](nil), p.bvh...),
		bvhRoot:     p.bvhRoot,
	}
	return
}
