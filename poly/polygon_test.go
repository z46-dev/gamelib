package poly_test

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/z46-dev/gamelib/gmath"
	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
)

func newPolygon(points [][2]float64) (polygon *poly.Polygon[float64]) {
	var vertices []*vector.Vec2[float64] = make([]*vector.Vec2[float64], len(points))
	for i := range points {
		vertices[i] = vector.NewVec2(points[i][0], points[i][1])
	}

	polygon = poly.NewPolygon(vertices, vector.Vec2_0[float64](), 1, 0)
	return
}

func TestPolygonTransformAndQueries(t *testing.T) {
	var polygon *poly.Polygon[float64] = newPolygon([][2]float64{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}})

	assert.True(t, polygon.PointIsInside(vector.NewVec2(0.0, 0.0)))
	assert.False(t, polygon.PointIsInside(vector.NewVec2(2.0, 0.0)))
	assert.True(t, polygon.CircleIntersects(vector.NewVec2(1.5, 0.0), 0.6))
	assert.False(t, polygon.CircleIntersects(vector.NewVec2(2.0, 0.0), 0.5))
	assert.True(t, polygon.LineIntersects(vector.NewVec2(-2.0, 0.0), vector.NewVec2(2.0, 0.0)))
	assert.False(t, polygon.LineIntersects(vector.NewVec2(-2.0, 2.0), vector.NewVec2(2.0, 2.0)))

	polygon.Transform(vector.NewVec2(5.0, -2.0), 2, math.Pi/2)
	assert.InDelta(t, 3, polygon.AABB.X1, gmath.EPSILON)
	assert.InDelta(t, -4, polygon.AABB.Y1, gmath.EPSILON)
	assert.InDelta(t, 7, polygon.AABB.X2, gmath.EPSILON)
	assert.InDelta(t, 0, polygon.AABB.Y2, gmath.EPSILON)
	assert.True(t, polygon.PointIsInside(vector.NewVec2(5.0, -2.0)))
}

func TestPolygonConstructionValidationAndZeroScale(t *testing.T) {
	var polygon *poly.Polygon[float64]

	assert.Panics(t, func() {
		poly.NewPolygon([]*vector.Vec2[float64]{vector.NewVec2(0.0, 0.0), vector.NewVec2(1.0, 0.0)}, vector.Vec2_0[float64](), 1, 0)
	})

	polygon = poly.NewPolygon([]*vector.Vec2[float64]{
		vector.NewVec2(0.0, 0.0),
		vector.NewVec2(1.0, 0.0),
		vector.NewVec2(0.0, 1.0),
	}, vector.NewVec2(5.0, 6.0), 0, 0)
	assert.Equal(t, &hshg.AABB2[float64]{X1: 5, Y1: 6, X2: 5, Y2: 6}, polygon.AABB)
}

func TestConvexPolygonIntersectionAndResolution(t *testing.T) {
	var (
		left  *poly.Polygon[float64] = newPolygon([][2]float64{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}})
		right *poly.Polygon[float64] = newPolygon([][2]float64{{0.5, -1}, {2.5, -1}, {2.5, 1}, {0.5, 1}})
		far   *poly.Polygon[float64] = newPolygon([][2]float64{{3, -1}, {5, -1}, {5, 1}, {3, 1}})
	)

	assert.True(t, poly.TwoPolygonsIntersect(left, right))
	assert.False(t, poly.TwoPolygonsIntersect(left, far))
	assert.NotNil(t, poly.ResolveTwoPolygons(left, right))
	assert.Nil(t, poly.ResolveTwoPolygons(left, far))
}

func TestConcavePolygonIntersection(t *testing.T) {
	var (
		concave *poly.Polygon[float64] = newPolygon([][2]float64{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}})
		inArm   *poly.Polygon[float64] = newPolygon([][2]float64{{0.25, 1.5}, {0.75, 1.5}, {0.75, 2}, {0.25, 2}})
		inNotch *poly.Polygon[float64] = newPolygon([][2]float64{{1.5, 1.5}, {2.5, 1.5}, {2.5, 2.5}, {1.5, 2.5}})
	)

	assert.True(t, poly.TwoPolygonsIntersect(concave, inArm))
	assert.False(t, poly.TwoPolygonsIntersect(concave, inNotch), "overlapping AABBs must not fill a concavity")
	assert.True(t, concave.PointIsInside(vector.NewVec2(0.5, 2.0)))
	assert.False(t, concave.PointIsInside(vector.NewVec2(2.0, 2.0)))
}

func TestCirclePolygonResolution(t *testing.T) {
	var (
		polygon *poly.Polygon[float64] = newPolygon([][2]float64{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}})
		point   *vector.Vec2[float64]
		angle   float64
	)

	point, angle = poly.ResolveCirclePolygon(vector.NewVec2(1.25, 0.0), 0.5, polygon)
	assert.NotNil(t, point)
	assert.InDelta(t, 1.5, point.X, gmath.EPSILON)
	assert.InDelta(t, 0, point.Y, gmath.EPSILON)
	assert.InDelta(t, 0, angle, gmath.EPSILON)
}
