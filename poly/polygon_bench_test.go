package poly_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
)

var (
	benchmarkIntersection                          bool
	benchmarkResolution                            *vector.Vec2[float64]
	benchmarkProjectionMin, benchmarkProjectionMax float64
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

func BenchmarkPolygonIntersection(b *testing.B) {
	for _, vertexCount := range []int{4, 16, 64} {
		b.Run(fmt.Sprintf("ConvexHit/%dVertices", vertexCount), func(b *testing.B) {
			benchmarkPolygonPairIntersection(b, newRegularPolygon(vertexCount, 0), newRegularPolygon(vertexCount, 1.5))
		})
		b.Run(fmt.Sprintf("ConvexMiss/%dVertices", vertexCount), func(b *testing.B) {
			benchmarkPolygonPairIntersection(b, newRegularPolygon(vertexCount, 0), newRegularPolygon(vertexCount, 3))
		})
	}

	var concave *poly.Polygon[float64] = newPolygon([][2]float64{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}})
	b.Run("ConcaveHit", func(b *testing.B) {
		benchmarkPolygonPairIntersection(b, concave, newPolygon([][2]float64{{0.25, 1.5}, {0.75, 1.5}, {0.75, 2}, {0.25, 2}}))
	})
	b.Run("ConcaveVoid", func(b *testing.B) {
		benchmarkPolygonPairIntersection(b, concave, newPolygon([][2]float64{{1.5, 1.5}, {2.5, 1.5}, {2.5, 2.5}, {1.5, 2.5}}))
	})
}

func benchmarkPolygonPairIntersection(b *testing.B, first, second *poly.Polygon[float64]) {
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		benchmarkIntersection = poly.TwoPolygonsIntersect(first, second)
	}
}

func BenchmarkPolygonResolution(b *testing.B) {
	for _, vertexCount := range []int{4, 16, 64} {
		b.Run(fmt.Sprintf("Overlap/%dVertices", vertexCount), func(b *testing.B) {
			var (
				first  *poly.Polygon[float64] = newRegularPolygon(vertexCount, 0)
				second *poly.Polygon[float64] = newRegularPolygon(vertexCount, 1.5)
			)
			b.ReportAllocs()
			b.ResetTimer()
			for b.Loop() {
				benchmarkResolution = poly.ResolveTwoPolygons(first, second)
			}
		})
	}
}

func BenchmarkPolygonQueries(b *testing.B) {
	var polygon *poly.Polygon[float64] = newRegularPolygon(64, 0)
	b.Run("Projection/64Vertices", func(b *testing.B) {
		var axis *vector.Vec2[float64] = vector.NewVec2(0.6, 0.8)
		b.ReportAllocs()
		for b.Loop() {
			benchmarkProjectionMin, benchmarkProjectionMax = polygon.ProjectOnto(axis)
		}
	})
	b.Run("Transform/64Vertices", func(b *testing.B) {
		var (
			position *vector.Vec2[float64] = vector.NewVec2(10.0, -4.0)
			rotation float64
		)
		b.ReportAllocs()
		for b.Loop() {
			rotation += 0.001
			polygon.Transform(position, 2, rotation)
		}
	})
}
