package poly

import (
	"math"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

// NewPolygon creates a polygon from local-space points and applies its initial transform.
func NewPolygon[T constraints.Float](points []*vector.Vec2[T], position *vector.Vec2[T], radius, rotation T) (p *Polygon[T]) {
	if len(points) < 3 {
		panic("poly: polygon requires at least three points")
	}

	p = &Polygon[T]{
		numPoints: len(points),
		Reference: make([]*vector.Vec2[T], len(points)),
		Points:    make([]*vector.Vec2[T], len(points)),
		x:         0,
		y:         0,
		radius:    0,
		rotation:  0,
		AABB:      &hshg.AABB2[T]{},
		axes:      make([]vector.Vec2[T], len(points)),
	}

	for i := range points {
		p.Reference[i] = vector.NewVec2(points[i].X, points[i].Y)
		p.Points[i] = vector.NewVec2[T](0, 0)
	}

	if !isConvexPolygon(points) {
		p.parts = buildConvexParts(points)
		if len(p.parts) == 0 {
			panic("poly: polygon could not be decomposed into convex parts")
		}
	}

	p.Transform(position, radius, rotation)
	return
}

// Transform updates the polygon's world-space points, convex parts, and bounds.
func (p *Polygon[T]) Transform(position *vector.Vec2[T], radius, rotation T) {
	if p.transformed && p.x == position.X && p.y == position.Y && p.radius == radius && p.rotation == rotation {
		return
	}

	var (
		rotation64 float64 = float64(rotation)
		cos, sin   T       = T(math.Cos(rotation64)), T(math.Sin(rotation64))
	)

	for i := range p.numPoints {
		p.Points[i].X = position.X + (p.Reference[i].X*cos-p.Reference[i].Y*sin)*radius
		p.Points[i].Y = position.Y + (p.Reference[i].X*sin+p.Reference[i].Y*cos)*radius
	}

	for _, part := range p.parts {
		part.Transform(position, radius, rotation)
	}

	p.x, p.y, p.radius, p.rotation = position.X, position.Y, radius, rotation
	p.transformed = true
	p.updateAABB()
}

// updateAABB refreshes the polygon's bounds and normalized separating axes.
func (p *Polygon[T]) updateAABB() {
	var minX, minY, maxX, maxY T = T(math.Inf(1)), T(math.Inf(1)), T(math.Inf(-1)), T(math.Inf(-1))

	for i := range p.numPoints {
		var (
			point  *vector.Vec2[T] = p.Points[i]
			next   *vector.Vec2[T] = p.Points[(i+1)%p.numPoints]
			axisX  T               = -(next.Y - point.Y)
			axisY  T               = next.X - point.X
			length T               = T(math.Hypot(float64(axisX), float64(axisY)))
		)

		minX = min(minX, point.X)
		minY = min(minY, point.Y)
		maxX = max(maxX, point.X)
		maxY = max(maxY, point.Y)
		if length == 0 {
			p.axes[i].X, p.axes[i].Y = 0, 0
		} else {
			p.axes[i].X, p.axes[i].Y = axisX/length, axisY/length
		}
	}

	p.AABB.X1, p.AABB.Y1, p.AABB.X2, p.AABB.Y2 = minX, minY, maxX, maxY
}

// makePolygonFromPoints builds an internal convex polygon from local-space points.
func makePolygonFromPoints[T constraints.Float](points []*vector.Vec2[T]) (p *Polygon[T]) {
	p = &Polygon[T]{
		numPoints: len(points),
		Reference: make([]*vector.Vec2[T], len(points)),
		Points:    make([]*vector.Vec2[T], len(points)),
		AABB:      &hshg.AABB2[T]{},
		axes:      make([]vector.Vec2[T], len(points)),
	}

	for i := range points {
		p.Reference[i] = vector.NewVec2(points[i].X, points[i].Y)
		p.Points[i] = vector.NewVec2(points[i].X, points[i].Y)
	}

	p.updateAABB()
	return
}

// polygonArea returns the signed area used to determine vertex winding.
func polygonArea[T constraints.Float](points []*vector.Vec2[T]) (area T) {
	for i := range points {
		var j int = (i + 1) % len(points)
		area += points[i].X*points[j].Y - points[j].X*points[i].Y
	}

	area *= .5
	return
}

// isConvexPolygon reports whether all nondegenerate turns use the same winding.
func isConvexPolygon[T constraints.Float](points []*vector.Vec2[T]) (convex bool) {
	if len(points) < 4 {
		convex = true
		return
	}

	const eps float64 = 1e-9
	var sign float64
	for i := range points {
		var (
			a     *vector.Vec2[T] = points[i]
			b     *vector.Vec2[T] = points[(i+1)%len(points)]
			c     *vector.Vec2[T] = points[(i+2)%len(points)]
			cross float64         = float64((b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X))
		)

		if math.Abs(cross) <= eps {
			continue
		}

		if sign == 0 {
			sign = math.Copysign(1, cross)
			continue
		}

		if sign*cross < 0 {
			convex = false
			return
		}
	}

	convex = true
	return
}

// pointInTriangle reports whether a point lies inside or on a consistently wound triangle.
func pointInTriangle[T constraints.Float](point, a, b, c *vector.Vec2[T], ccw bool) (inside bool) {
	var (
		eps T = 1e-9
		ab  T = (b.X-a.X)*(point.Y-a.Y) - (b.Y-a.Y)*(point.X-a.X)
		bc  T = (c.X-b.X)*(point.Y-b.Y) - (c.Y-b.Y)*(point.X-b.X)
		ca  T = (a.X-c.X)*(point.Y-c.Y) - (a.Y-c.Y)*(point.X-c.X)
	)

	if ccw {
		inside = ab >= -eps && bc >= -eps && ca >= -eps
	} else {
		inside = ab <= eps && bc <= eps && ca <= eps
	}

	return
}

// buildConvexParts decomposes a simple concave polygon into triangles using ear clipping.
func buildConvexParts[T constraints.Float](points []*vector.Vec2[T]) (parts []*Polygon[T]) {
	if len(points) < 4 || isConvexPolygon(points) {
		return []*Polygon[T]{makePolygonFromPoints(points)}
	}

	var (
		indices []int = make([]int, len(points))
		ccw     bool  = polygonArea(points) >= 0
	)

	for i := range points {
		indices[i] = i
	}

	const eps float64 = 1e-9
	var guard int
	for len(indices) > 3 && guard < len(points)*len(points) {
		guard++
		var earFound bool
		for i := range indices {
			var (
				prev int = indices[(i-1+len(indices))%len(indices)]
				curr int = indices[i]
				next int = indices[(i+1)%len(indices)]

				a     *vector.Vec2[T] = points[prev]
				b     *vector.Vec2[T] = points[curr]
				c     *vector.Vec2[T] = points[next]
				cross float64         = float64((b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X))
			)

			if math.Abs(cross) <= eps {
				continue
			}

			if ccw && cross <= 0 {
				continue
			}
			if !ccw && cross >= 0 {
				continue
			}

			var anyInside bool
			for _, idx := range indices {
				if idx == prev || idx == curr || idx == next {
					continue
				}
				if pointInTriangle(points[idx], a, b, c, ccw) {
					anyInside = true
					break
				}
			}

			if anyInside {
				continue
			}

			parts = append(parts, makePolygonFromPoints([]*vector.Vec2[T]{a, b, c}))
			indices = append(indices[:i], indices[i+1:]...)
			earFound = true
			break
		}

		if !earFound {
			break
		}
	}

	if len(indices) == 3 {
		parts = append(parts, makePolygonFromPoints([]*vector.Vec2[T]{
			points[indices[0]], points[indices[1]], points[indices[2]],
		}))
	}

	return parts
}

// PointIsInside reports whether a point lies inside the polygon using an even-odd crossing test.
func (p *Polygon[T]) PointIsInside(point *vector.Vec2[T]) (inside bool) {
	var x1, y1 T = p.Points[p.numPoints-1].X, p.Points[p.numPoints-1].Y

	for i := range p.numPoints {
		var x2, y2 T = p.Points[i].X, p.Points[i].Y

		if (point.Y < y1) != (point.Y < y2) && (point.X < (x2-x1)*(point.Y-y1)/(y2-y1)+x1) {
			inside = !inside
		}

		x1, y1 = x2, y2
	}

	return
}

// CircleIntersectsEdge reports whether a circle overlaps a line segment from the polygon.
func (p *Polygon[T]) CircleIntersectsEdge(p1, p2 *vector.Vec2[T], circlePoint *vector.Vec2[T], circleRadius T) (intersects bool) {
	var (
		ABx, ABy      T = p2.X - p1.X, p2.Y - p1.Y
		ACx, ACy      T = circlePoint.X - p1.X, circlePoint.Y - p1.Y
		squaredLength T = ABx*ABx + ABy*ABy
		t             T
	)

	if squaredLength != 0 {
		t = max(0, min(1, (ABx*ACx+ABy*ACy)/squaredLength))
	}

	var dx, dy T = (p1.X + ABx*t) - circlePoint.X, (p1.Y + ABy*t) - circlePoint.Y

	intersects = (dx*dx + dy*dy) < (circleRadius * circleRadius)
	return
}

// CircleIntersects reports whether a circle overlaps the polygon interior or any edge.
func (p *Polygon[T]) CircleIntersects(circlePoint *vector.Vec2[T], radius T) (intersects bool) {
	if intersects = p.PointIsInside(circlePoint); intersects {
		return
	}

	for i := range p.numPoints {
		if intersects = p.CircleIntersectsEdge(p.Points[i], p.Points[(i+1)%p.numPoints], circlePoint, radius); intersects {
			return
		}
	}

	return
}

// GetClosestPointOnEdge returns the closest point on segment p1-p2 to point p3.
func (p *Polygon[T]) GetClosestPointOnEdge(p1, p2, p3 *vector.Vec2[T]) (point *vector.Vec2[T]) {
	var (
		ABx, ABy      T = p2.X - p1.X, p2.Y - p1.Y
		ACx, ACy      T = p3.X - p1.X, p3.Y - p1.Y
		squaredLength T = ABx*ABx + ABy*ABy
		t             T
	)

	if squaredLength != 0 {
		t = max(0, min(1, (ABx*ACx+ABy*ACy)/squaredLength))
	}

	point = vector.NewVec2(p1.X+ABx*t, p1.Y+ABy*t)
	return
}

// ProjectOnto projects all world-space vertices onto an axis.
func (p *Polygon[T]) ProjectOnto(axis *vector.Vec2[T]) (minimum, maximum T) {
	minimum, maximum = T(math.Inf(1)), T(math.Inf(-1))

	for i := range p.numPoints {
		var dotProduct T = p.Points[i].Dot(axis)
		minimum, maximum = min(minimum, dotProduct), max(maximum, dotProduct)
	}

	return
}

// projectOntoAxis projects all world-space vertices onto scalar axis components.
func (p *Polygon[T]) projectOntoAxis(axisX, axisY T) (minimum, maximum T) {
	minimum, maximum = T(math.Inf(1)), T(math.Inf(-1))

	for i := range p.numPoints {
		var dotProduct T = p.Points[i].X*axisX + p.Points[i].Y*axisY
		minimum, maximum = min(minimum, dotProduct), max(maximum, dotProduct)
	}

	return
}

// ResolveCirclePolygon returns a boundary-relative circle position and its outward direction.
func ResolveCirclePolygon[T constraints.Float](circlePoint *vector.Vec2[T], circleRadius T, polygon *Polygon[T]) (point *vector.Vec2[T], angle T) {
	var (
		closestDistance    T = T(math.Inf(1))
		closestX, closestY T
		found              bool
	)

	for i := range polygon.numPoints {
		var (
			start         *vector.Vec2[T] = polygon.Points[i]
			end           *vector.Vec2[T] = polygon.Points[(i+1)%polygon.numPoints]
			edgeX, edgeY  T               = end.X - start.X, end.Y - start.Y
			toX, toY      T               = circlePoint.X - start.X, circlePoint.Y - start.Y
			squaredLength T               = edgeX*edgeX + edgeY*edgeY
			t             T
		)

		if squaredLength != 0 {
			t = max(0, min(1, (edgeX*toX+edgeY*toY)/squaredLength))
		}

		var (
			candidateX T = start.X + edgeX*t
			candidateY T = start.Y + edgeY*t
			dx, dy     T = candidateX - circlePoint.X, candidateY - circlePoint.Y
			distance   T = dx*dx + dy*dy
		)

		if distance < closestDistance {
			closestDistance = distance
			closestX, closestY = candidateX, candidateY
			found = true
		}
	}

	if !found {
		return nil, 0
	}

	var normal *vector.Vec2[T] = vector.NewVec2(circlePoint.X-closestX, circlePoint.Y-closestY)
	if normal.SquaredLength() == 0 {
		normal = vector.NewVec2[T](1, 0)
	} else {
		normal.Normalize()
	}

	if polygon.PointIsInside(circlePoint) {
		normal.Mul(-1)
	}

	point = vector.NewVec2(closestX, closestY).Add(normal.Mul(circleRadius))
	angle = T(math.Atan2(float64(point.Y-closestY), float64(point.X-closestX)))
	return
}

// TwoPolygonsIntersect reports whether two convex or decomposed concave polygons overlap.
func TwoPolygonsIntersect[T constraints.Float](p1, p2 *Polygon[T]) (intersects bool) {
	if !p1.AABB.Intersects(p2.AABB) {
		return
	}

	var (
		p1single [1]*Polygon[T]
		p2single [1]*Polygon[T]
		parts1   []*Polygon[T] = p1.parts
		parts2   []*Polygon[T] = p2.parts
	)

	if len(parts1) == 0 {
		p1single[0] = p1
		parts1 = p1single[:]
	}

	if len(parts2) == 0 {
		p2single[0] = p2
		parts2 = p2single[:]
	}

	for _, a := range parts1 {
		for _, b := range parts2 {
			if a.AABB.Intersects(b.AABB) && twoPolygonsIntersectConvex(a, b) {
				return true
			}
		}
	}

	return false
}

// twoPolygonsIntersectConvex applies SAT to a pair of convex polygons.
func twoPolygonsIntersectConvex[T constraints.Float](p1, p2 *Polygon[T]) (intersects bool) {
	return !polygonsSeparateOnAxes(p1, p2) && !polygonsSeparateOnAxes(p2, p1)
}

// ResolveTwoPolygons returns a separating translation for overlapping polygon parts.
func ResolveTwoPolygons[T constraints.Float](p1, p2 *Polygon[T]) (resolution *vector.Vec2[T]) {
	if !p1.AABB.Intersects(p2.AABB) {
		return
	}

	var (
		p1single  [1]*Polygon[T]
		p2single  [1]*Polygon[T]
		parts1    []*Polygon[T]   = p1.parts
		parts2    []*Polygon[T]   = p2.parts
		bestMTV   *vector.Vec2[T] = nil
		bestScore T               = T(math.Inf(-1))
		deltaX    T               = (p2.AABB.X1+p2.AABB.X2)/2 - (p1.AABB.X1+p1.AABB.X2)/2
		deltaY    T               = (p2.AABB.Y1+p2.AABB.Y2)/2 - (p1.AABB.Y1+p1.AABB.Y2)/2
	)

	if len(parts1) == 0 {
		p1single[0] = p1
		parts1 = p1single[:]
	}

	if len(parts2) == 0 {
		p2single[0] = p2
		parts2 = p2single[:]
	}

	for _, a := range parts1 {
		for _, b := range parts2 {
			if !a.AABB.Intersects(b.AABB) {
				continue
			}

			var mtv *vector.Vec2[T] = resolveTwoPolygonsConvex(a, b)
			if mtv == nil {
				continue
			}

			if (deltaX != 0 || deltaY != 0) && mtv.X*deltaX+mtv.Y*deltaY > 0 {
				mtv = mtv.Copy().Mul(-1)
			}

			var score T = mtv.SquaredLength()
			if score > bestScore {
				bestScore = score
				bestMTV = mtv
			}
		}
	}

	return bestMTV
}

// resolveTwoPolygonsConvex returns the minimum SAT translation for two convex polygons.
func resolveTwoPolygonsConvex[T constraints.Float](p1, p2 *Polygon[T]) (resolution *vector.Vec2[T]) {
	var (
		mtv        *vector.Vec2[T] = nil
		minOverlap T               = T(math.Inf(1))
		deltaX     T               = (p2.AABB.X1+p2.AABB.X2)/2 - (p1.AABB.X1+p1.AABB.X2)/2
		deltaY     T               = (p2.AABB.Y1+p2.AABB.Y2)/2 - (p1.AABB.Y1+p1.AABB.Y2)/2
		eps        T               = 1e-6
	)

	if !updateMTVFromAxes(p1, p2, deltaX, deltaY, eps, &minOverlap, &mtv) {
		return nil
	}
	if !updateMTVFromAxes(p2, p1, deltaX, deltaY, eps, &minOverlap, &mtv) {
		return nil
	}

	return mtv
}

// polygonsSeparateOnAxes reports whether any cached source axis separates the polygons.
func polygonsSeparateOnAxes[T constraints.Float](source, target *Polygon[T]) bool {
	for i := range source.axes {
		var axis *vector.Vec2[T] = &source.axes[i]
		if axis.X == 0 && axis.Y == 0 {
			continue
		}

		var (
			min1, max1 T = source.projectOntoAxis(axis.X, axis.Y)
			min2, max2 T = target.projectOntoAxis(axis.X, axis.Y)
		)

		if max1 < min2 || max2 < min1 {
			return true
		}
	}

	return false
}

// updateMTVFromAxes tests cached axes and updates the smallest separating translation.
func updateMTVFromAxes[T constraints.Float](source, target *Polygon[T], deltaX, deltaY, eps T, minOverlap *T, mtv **vector.Vec2[T]) bool {
	for i := range source.axes {
		var axis *vector.Vec2[T] = &source.axes[i]
		if axis.X == 0 && axis.Y == 0 {
			continue
		}

		var (
			min1, max1 T = source.projectOntoAxis(axis.X, axis.Y)
			min2, max2 T = target.projectOntoAxis(axis.X, axis.Y)
			overlap    T = min(max1, max2) - max(min1, min2)
		)

		if overlap < -eps {
			return false
		}

		if overlap < eps {
			overlap = eps
		}

		if overlap < *minOverlap {
			*minOverlap = overlap
			var flip T = 1
			if deltaX*axis.X+deltaY*axis.Y > 0 {
				flip = -1
			}

			if *mtv == nil {
				*mtv = &vector.Vec2[T]{}
			}

			(*mtv).X = axis.X * overlap * flip
			(*mtv).Y = axis.Y * overlap * flip
		}
	}

	return true
}

// LineIntersects reports whether a line segment crosses any polygon edge.
func (p *Polygon[T]) LineIntersects(p1, p2 *vector.Vec2[T]) (intersects bool) {
	for i := range p.numPoints {
		var (
			q1X, q1Y T = p.Points[i].X, p.Points[i].Y
			q2X, q2Y T = p.Points[(i+1)%p.numPoints].X, p.Points[(i+1)%p.numPoints].Y
			denom    T = (p2.Y-p1.Y)*(q2X-q1X) - (p2.X-p1.X)*(q2Y-q1Y)
		)

		if denom == 0 {
			continue
		}

		var (
			uA T = ((p2.X-p1.X)*(q1Y-p1.Y) - (p2.Y-p1.Y)*(q1X-p1.X)) / denom
			uB T = ((q2X-q1X)*(q1Y-p1.Y) - (q2Y-q1Y)*(q1X-p1.X)) / denom
		)

		if uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1 {
			intersects = true
			return
		}
	}

	return
}
