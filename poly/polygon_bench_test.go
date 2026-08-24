package poly_test

import (
	"math"
	"testing"

	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
)

var (
	benchmarkIntersection bool
	benchmarkResolution   *vector.Vec2[float64]
)

func newRegularPolygon(vertexCount int, centerX float64) (polygon *poly.Polygon[float64]) {
	var points []*vector.Vec2[float64] = make([]*vector.Vec2[float64], vertexCount)
	for i := range vertexCount {
		var angle float64 = float64(i) * 2 * math.Pi / float64(vertexCount)
		points[i] = vector.NewVec2(math.Cos(angle)+centerX, math.Sin(angle))
	}

	polygon = poly.NewPolygon(points, vector.Vec2_0[float64](), 1, 0)
	return
}

func BenchmarkConvexPolygonIntersection(b *testing.B) {
	var (
		first  *poly.Polygon[float64] = newRegularPolygon(16, 0)
		second *poly.Polygon[float64] = newRegularPolygon(16, 1.5)
	)

	b.ReportAllocs()
	for b.Loop() {
		benchmarkIntersection = poly.TwoPolygonsIntersect(first, second)
	}
}

func BenchmarkConcavePolygonIntersection(b *testing.B) {
	var (
		concave *poly.Polygon[float64] = newPolygon([][2]float64{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}})
		box     *poly.Polygon[float64] = newPolygon([][2]float64{{0.25, 1.5}, {0.75, 1.5}, {0.75, 2}, {0.25, 2}})
	)

	b.ReportAllocs()
	for b.Loop() {
		benchmarkIntersection = poly.TwoPolygonsIntersect(concave, box)
	}
}

func BenchmarkPolygonResolution(b *testing.B) {
	var (
		first  *poly.Polygon[float64] = newRegularPolygon(16, 0)
		second *poly.Polygon[float64] = newRegularPolygon(16, 1.5)
	)

	b.ReportAllocs()
	for b.Loop() {
		benchmarkResolution = poly.ResolveTwoPolygons(first, second)
	}
}
