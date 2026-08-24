package gamelib

import (
	"math"

	"golang.org/x/exp/constraints"
)

type PolygonXYPoint[T constraints.Float] struct {
	x, y T
}

type Polygon[T constraints.Float] struct {
	numPoints              int
	Reference, Points      []*Vec2[T]
	x, y, radius, rotation T
	AABB                   *AABB[T]
	parts                  []*Polygon[T]
}

func NewPolygon[T constraints.Float](points []*Vec2[T], position *Vec2[T], radius, rotation T) (p *Polygon[T]) {
	p = &Polygon[T]{
		numPoints: len(points),
		Reference: make([]*Vec2[T], len(points)),
		Points:    make([]*Vec2[T], len(points)),
		x:         0,
		y:         0,
		radius:    0,
		rotation:  0,
		AABB:      &AABB[T]{},
	}

	for i := range points {
		p.Reference[i] = NewVec2(points[i].X, points[i].Y)
		p.Points[i] = NewVec2[T](0, 0)
	}

	if !isConvexPolygon(points) {
		p.parts = buildConvexParts(points)
	}

	p.Transform(position, radius, rotation)
	return
}

func (p *Polygon[T]) Transform(position *Vec2[T], radius, rotation T) {
	if p.x == position.X && p.y == position.Y && p.radius == radius && p.rotation == rotation {
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
	p.updateAABB()
}

func (p *Polygon[T]) updateAABB() {
	var minX, minY, maxX, maxY T = T(math.Inf(1)), T(math.Inf(1)), T(math.Inf(-1)), T(math.Inf(-1))

	for i := range p.numPoints {
		minX = min(minX, p.Points[i].X)
		minY = min(minY, p.Points[i].Y)
		maxX = max(maxX, p.Points[i].X)
		maxY = max(maxY, p.Points[i].Y)
	}

	p.AABB.X1, p.AABB.Y1, p.AABB.X2, p.AABB.Y2 = minX, minY, maxX, maxY
}

func makePolygonFromPoints[T constraints.Float](points []*Vec2[T]) (p *Polygon[T]) {
	p = &Polygon[T]{
		numPoints: len(points),
		Reference: make([]*Vec2[T], len(points)),
		Points:    make([]*Vec2[T], len(points)),
		AABB:      &AABB[T]{},
	}

	for i := range points {
		p.Reference[i] = NewVec2(points[i].X, points[i].Y)
		p.Points[i] = NewVec2(points[i].X, points[i].Y)
	}

	p.updateAABB()
	return
}

func polygonArea[T constraints.Float](points []*Vec2[T]) (area T) {
	for i := range points {
		var j int = (i + 1) % len(points)
		area += points[i].X*points[j].Y - points[j].X*points[i].Y
	}

	area *= .5
	return
}

func isConvexPolygon[T constraints.Float](points []*Vec2[T]) (convex bool) {
	if len(points) < 4 {
		convex = true
		return
	}

	const eps float64 = 1e-9
	var sign float64
	for i := range points {
		var (
			a     *Vec2[T] = points[i]
			b     *Vec2[T] = points[(i+1)%len(points)]
			c     *Vec2[T] = points[(i+2)%len(points)]
			cross float64  = float64((b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X))
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

func pointInTriangle[T constraints.Float](point, a, b, c *Vec2[T], ccw bool) (inside bool) {
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

func buildConvexParts[T constraints.Float](points []*Vec2[T]) (parts []*Polygon[T]) {
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

				a     *Vec2[T] = points[prev]
				b     *Vec2[T] = points[curr]
				c     *Vec2[T] = points[next]
				cross float64  = float64((b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X))
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

			parts = append(parts, makePolygonFromPoints([]*Vec2[T]{a, b, c}))
			indices = append(indices[:i], indices[i+1:]...)
			earFound = true
			break
		}

		if !earFound {
			break
		}
	}

	if len(indices) == 3 {
		parts = append(parts, makePolygonFromPoints([]*Vec2[T]{
			points[indices[0]], points[indices[1]], points[indices[2]],
		}))
	}

	if len(parts) == 0 {
		return []*Polygon[T]{makePolygonFromPoints(points)}
	}

	return parts
}

func (p *Polygon[T]) PointIsInside(point *Vec2[T]) (inside bool) {
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

func (p *Polygon[T]) CircleIntersectsEdge(p1, p2 *Vec2[T], circlePoint *Vec2[T], circleRadius T) (intersects bool) {
	var (
		ABx, ABy T = p2.X - p1.X, p2.Y - p1.Y
		ACx, ACy T = circlePoint.X - p1.X, circlePoint.Y - p1.Y
		t        T = max(0, min(1, (ABx*ACx+ABy*ACy)/(ABx*ABx+ABy*ABy)))
		dx, dy   T = (p1.X + ABx*t) - circlePoint.X, (p1.Y + ABy*t) - circlePoint.Y
	)

	intersects = (dx*dx + dy*dy) < (circleRadius * circleRadius)
	return
}

func (p *Polygon[T]) CircleIntersects(circlePoint *Vec2[T], radius T) (intersects bool) {
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

func (p *Polygon[T]) GetClosestPointOnEdge(p1, p2, p3 *Vec2[T]) (point *Vec2[T]) {
	var (
		ABx, ABy T = p2.X - p1.X, p2.Y - p1.Y
		ACx, ACy T = p3.X - p1.X, p3.Y - p1.Y
		t        T = max(0, min(1, (ABx*ACx+ABy*ACy)/(ABx*ABx+ABy*ABy)))
	)

	point = NewVec2(p1.X+ABx*t, p1.Y+ABy*t)
	return
}

func (p *Polygon[T]) ProjectOnto(axis *Vec2[T]) (minimum, maximum T) {
	minimum, maximum = T(math.Inf(1)), T(math.Inf(-1))

	for i := range p.numPoints {
		var dotProduct T = p.Points[i].Dot(axis)
		minimum, maximum = min(minimum, dotProduct), max(maximum, dotProduct)
	}

	return
}

func (p *Polygon[T]) projectOntoAxis(axisX, axisY T) (minimum, maximum T) {
	minimum, maximum = T(math.Inf(1)), T(math.Inf(-1))

	for i := range p.numPoints {
		var dotProduct T = p.Points[i].X*axisX + p.Points[i].Y*axisY
		minimum, maximum = min(minimum, dotProduct), max(maximum, dotProduct)
	}

	return
}

func ResolveCirclePolygon[T constraints.Float](circlePoint *Vec2[T], circleRadius T, polygon *Polygon[T]) (point *Vec2[T], angle T) {
	var (
		closestDistance T        = T(math.Inf(1))
		closestPoint    *Vec2[T] = nil
	)

	for i := range polygon.numPoints {
		var (
			point *Vec2[T] = polygon.GetClosestPointOnEdge(polygon.Points[i], polygon.Points[(i+1)%polygon.numPoints], circlePoint)
			dist  T        = point.DistSquared(circlePoint)
		)

		if dist < closestDistance {
			closestDistance = dist
			closestPoint = point
		}
	}

	if closestPoint == nil {
		return nil, 0
	}

	var normal *Vec2[T] = circlePoint.Copy().Sub(closestPoint)
	if normal.SquaredLength() == 0 {
		normal = NewVec2[T](1, 0)
	} else {
		normal.Normalize()
	}

	if polygon.PointIsInside(circlePoint) {
		normal.Mul(-1)
	}

	point = closestPoint.Copy().Add(normal.Mul(circleRadius))
	angle = closestPoint.AngleTo(point)
	return
}

func TwoPolygonsIntersect[T constraints.Float](p1, p2 *Polygon[T]) (intersects bool) {
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
			if twoPolygonsIntersectConvex(a, b) {
				return true
			}
		}
	}

	return false
}

func twoPolygonsIntersectConvex[T constraints.Float](p1, p2 *Polygon[T]) (intersects bool) {
	return !polygonsSeparateOnAxes(p1, p2) && !polygonsSeparateOnAxes(p2, p1)
}

func ResolveTwoPolygons[T constraints.Float](p1, p2 *Polygon[T]) (resolution *Vec2[T]) {
	var (
		p1single  [1]*Polygon[T]
		p2single  [1]*Polygon[T]
		parts1    []*Polygon[T] = p1.parts
		parts2    []*Polygon[T] = p2.parts
		bestMTV   *Vec2[T]      = nil
		bestScore T             = T(math.Inf(-1))
		deltaX    T             = (p2.AABB.X1+p2.AABB.X2)/2 - (p1.AABB.X1+p1.AABB.X2)/2
		deltaY    T             = (p2.AABB.Y1+p2.AABB.Y2)/2 - (p1.AABB.Y1+p1.AABB.Y2)/2
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
			var mtv *Vec2[T] = resolveTwoPolygonsConvex(a, b)
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

func resolveTwoPolygonsConvex[T constraints.Float](p1, p2 *Polygon[T]) (resolution *Vec2[T]) {
	var (
		mtv        *Vec2[T] = nil
		minOverlap T        = T(math.Inf(1))
		deltaX     T        = (p2.AABB.X1+p2.AABB.X2)/2 - (p1.AABB.X1+p1.AABB.X2)/2
		deltaY     T        = (p2.AABB.Y1+p2.AABB.Y2)/2 - (p1.AABB.Y1+p1.AABB.Y2)/2
		eps        T        = 1e-6
	)

	if !updateMTVFromAxes(p1, p2, deltaX, deltaY, eps, &minOverlap, &mtv) {
		return nil
	}
	if !updateMTVFromAxes(p2, p1, deltaX, deltaY, eps, &minOverlap, &mtv) {
		return nil
	}

	return mtv
}

func polygonsSeparateOnAxes[T constraints.Float](source, target *Polygon[T]) bool {
	for i := range source.numPoints {
		var (
			x1, y1 T = source.Points[i].X, source.Points[i].Y
			x2, y2 T = source.Points[(i+1)%source.numPoints].X, source.Points[(i+1)%source.numPoints].Y
			axisX  T = -(y2 - y1)
			axisY  T = x2 - x1
			length T = T(math.Hypot(float64(axisX), float64(axisY)))
		)

		if length == 0 {
			continue
		}

		axisX /= length
		axisY /= length

		var (
			min1, max1 T = source.projectOntoAxis(axisX, axisY)
			min2, max2 T = target.projectOntoAxis(axisX, axisY)
		)

		if max1 < min2 || max2 < min1 {
			return true
		}
	}

	return false
}

func updateMTVFromAxes[T constraints.Float](source, target *Polygon[T], deltaX, deltaY, eps T, minOverlap *T, mtv **Vec2[T]) bool {
	for i := range source.numPoints {
		var (
			x1, y1 T = source.Points[i].X, source.Points[i].Y
			x2, y2 T = source.Points[(i+1)%source.numPoints].X, source.Points[(i+1)%source.numPoints].Y
			axisX  T = -(y2 - y1)
			axisY  T = x2 - x1
			length T = T(math.Hypot(float64(axisX), float64(axisY)))
		)

		if length == 0 {
			continue
		}

		axisX /= length
		axisY /= length

		var (
			min1, max1 T = source.projectOntoAxis(axisX, axisY)
			min2, max2 T = target.projectOntoAxis(axisX, axisY)
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
			if deltaX*axisX+deltaY*axisY > 0 {
				flip = -1
			}

			if *mtv == nil {
				*mtv = &Vec2[T]{}
			}

			(*mtv).X = axisX * overlap * flip
			(*mtv).Y = axisY * overlap * flip
		}
	}

	return true
}

func (p *Polygon[T]) LineIntersects(p1, p2 *Vec2[T]) (intersects bool) {
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
