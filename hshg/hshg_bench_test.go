package hshg_test

import (
	"testing"

	"github.com/z46-dev/gamelib/hshg"
)

type (
	benchmarkCollidable2 struct {
		Bounds hshg.AABB2[float64]
	}

	benchmarkCollidable3 struct {
		Bounds hshg.AABB3[float64]
	}
)

var (
	benchmarkResults2 []*benchmarkCollidable2
	benchmarkResults3 []*benchmarkCollidable3
	benchmarkVisited2 *benchmarkCollidable2
	benchmarkVisited3 *benchmarkCollidable3
)

func (item *benchmarkCollidable2) GetAABB() (aabb *hshg.AABB2[float64]) {
	aabb = &item.Bounds
	return
}

func (item *benchmarkCollidable3) GetAABB() (aabb *hshg.AABB3[float64]) {
	aabb = &item.Bounds
	return
}

func buildBenchmarkSpatialHash2(spacing float64) (hash *hshg.SpatialHash2[*benchmarkCollidable2, float64]) {
	hash = hshg.NewSpatialHash2[*benchmarkCollidable2]()
	for y := range 100 {
		for x := range 100 {
			hash.Insert(&benchmarkCollidable2{Bounds: hshg.AABB2[float64]{
				X1: float64(x) * spacing,
				Y1: float64(y) * spacing,
				X2: float64(x)*spacing + 8,
				Y2: float64(y)*spacing + 8,
			}})
		}
	}

	return
}

func buildBenchmarkSpatialHash3(spacing float64, options ...hshg.SpatialHash3Option) (hash *hshg.SpatialHash3[*benchmarkCollidable3, float64]) {
	hash = hshg.NewSpatialHash3[*benchmarkCollidable3](options...)
	for z := range 25 {
		for y := range 25 {
			for x := range 25 {
				hash.Insert(&benchmarkCollidable3{Bounds: hshg.AABB3[float64]{
					X1: float64(x) * spacing,
					Y1: float64(y) * spacing,
					Z1: float64(z) * spacing,
					X2: float64(x)*spacing + 8,
					Y2: float64(y)*spacing + 8,
					Z2: float64(z)*spacing + 8,
				}})
			}
		}
	}

	return
}

func visitBenchmarkCollidable2(item *benchmarkCollidable2) (keepGoing bool) {
	benchmarkVisited2 = item
	keepGoing = true
	return
}

func visitBenchmarkCollidable3(item *benchmarkCollidable3) (keepGoing bool) {
	benchmarkVisited3 = item
	keepGoing = true
	return
}

func benchmarkSpatialHash2Scenario(b *testing.B, spacing float64, query hshg.AABB2[float64]) {
	var (
		hash    *hshg.SpatialHash2[*benchmarkCollidable2, float64] = buildBenchmarkSpatialHash2(spacing)
		results []*benchmarkCollidable2                            = hash.Retrieve(&query)
	)

	b.ReportAllocs()
	for b.Loop() {
		results = hash.RetrieveInto(results, &query)
	}

	benchmarkResults2 = results
}

func benchmarkSpatialHash3Scenario(b *testing.B, spacing float64, query hshg.AABB3[float64]) {
	var (
		hash    *hshg.SpatialHash3[*benchmarkCollidable3, float64] = buildBenchmarkSpatialHash3(spacing)
		results []*benchmarkCollidable3                            = hash.Retrieve(&query)
	)

	b.ReportAllocs()
	for b.Loop() {
		results = hash.RetrieveInto(results, &query)
	}

	benchmarkResults3 = results
}

func benchmarkSpatialHash3ConfiguredScenario(b *testing.B, option hshg.SpatialHash3Option) {
	var (
		hash    *hshg.SpatialHash3[*benchmarkCollidable3, float64] = buildBenchmarkSpatialHash3(16, option)
		query   hshg.AABB3[float64]                                = hshg.AABB3[float64]{X1: 250, Y1: 250, Z1: 250, X2: 390, Y2: 390, Z2: 390}
		results []*benchmarkCollidable3                            = hash.Retrieve(&query)
	)

	b.ReportAllocs()
	for b.Loop() {
		results = hash.RetrieveInto(results, &query)
	}

	benchmarkResults3 = results
}

func BenchmarkSpatialHash2Retrieve(b *testing.B) {
	var (
		hash  *hshg.SpatialHash2[*benchmarkCollidable2, float64] = buildBenchmarkSpatialHash2(16)
		query hshg.AABB2[float64]                                = hshg.AABB2[float64]{X1: 700, Y1: 700, X2: 900, Y2: 900}
	)

	b.ReportAllocs()

	for b.Loop() {
		benchmarkResults2 = hash.Retrieve(&query)
	}
}

func BenchmarkSpatialHash2RetrieveInto(b *testing.B) {
	benchmarkSpatialHash2Scenario(b, 16, hshg.AABB2[float64]{X1: 700, Y1: 700, X2: 900, Y2: 900})
}

func BenchmarkSpatialHash2Visit(b *testing.B) {
	var (
		hash  *hshg.SpatialHash2[*benchmarkCollidable2, float64] = buildBenchmarkSpatialHash2(16)
		query hshg.AABB2[float64]                                = hshg.AABB2[float64]{X1: 700, Y1: 700, X2: 900, Y2: 900}
	)

	b.ReportAllocs()
	for b.Loop() {
		hash.Visit(&query, visitBenchmarkCollidable2)
	}
}

func BenchmarkSpatialHash3Retrieve(b *testing.B) {
	var (
		hash  *hshg.SpatialHash3[*benchmarkCollidable3, float64] = buildBenchmarkSpatialHash3(16)
		query hshg.AABB3[float64]                                = hshg.AABB3[float64]{X1: 250, Y1: 250, Z1: 250, X2: 390, Y2: 390, Z2: 390}
	)

	b.ReportAllocs()

	for b.Loop() {
		benchmarkResults3 = hash.Retrieve(&query)
	}
}

func BenchmarkSpatialHash3RetrieveInto(b *testing.B) {
	benchmarkSpatialHash3Scenario(b, 16, hshg.AABB3[float64]{X1: 250, Y1: 250, Z1: 250, X2: 390, Y2: 390, Z2: 390})
}

func BenchmarkSpatialHash3Visit(b *testing.B) {
	var (
		hash  *hshg.SpatialHash3[*benchmarkCollidable3, float64] = buildBenchmarkSpatialHash3(16)
		query hshg.AABB3[float64]                                = hshg.AABB3[float64]{X1: 250, Y1: 250, Z1: 250, X2: 390, Y2: 390, Z2: 390}
	)

	b.ReportAllocs()
	for b.Loop() {
		hash.Visit(&query, visitBenchmarkCollidable3)
	}
}

func BenchmarkSpatialHash2Scenarios(b *testing.B) {
	b.Run("DenseSmall", func(b *testing.B) {
		benchmarkSpatialHash2Scenario(b, 16, hshg.AABB2[float64]{X1: 700, Y1: 700, X2: 900, Y2: 900})
	})
	b.Run("SparseSmall", func(b *testing.B) {
		benchmarkSpatialHash2Scenario(b, 256, hshg.AABB2[float64]{X1: 11200, Y1: 11200, X2: 11400, Y2: 11400})
	})
	b.Run("DenseLarge", func(b *testing.B) {
		benchmarkSpatialHash2Scenario(b, 16, hshg.AABB2[float64]{X1: 0, Y1: 0, X2: 1600, Y2: 1600})
	})
}

func BenchmarkSpatialHash3Scenarios(b *testing.B) {
	b.Run("DenseSmall", func(b *testing.B) {
		benchmarkSpatialHash3Scenario(b, 16, hshg.AABB3[float64]{X1: 250, Y1: 250, Z1: 250, X2: 390, Y2: 390, Z2: 390})
	})
	b.Run("SparseSmall", func(b *testing.B) {
		benchmarkSpatialHash3Scenario(b, 256, hshg.AABB3[float64]{X1: 4000, Y1: 4000, Z1: 4000, X2: 4200, Y2: 4200, Z2: 4200})
	})
	b.Run("DenseLarge", func(b *testing.B) {
		benchmarkSpatialHash3Scenario(b, 16, hshg.AABB3[float64]{X1: 0, Y1: 0, Z1: 0, X2: 400, Y2: 400, Z2: 400})
	})
}

func BenchmarkSpatialHash3LevelTuning(b *testing.B) {
	b.Run("Shifts_4_6_8", func(b *testing.B) {
		var (
			hash *hshg.SpatialHash3[*benchmarkCollidable3, float64] = buildBenchmarkSpatialHash3(16, hshg.WithSpatialHash3Levels(
				hshg.SpatialHashLevelConfig{Shift: 4, MaxCellsPerObject: 8},
				hshg.SpatialHashLevelConfig{Shift: 6, MaxCellsPerObject: 8},
				hshg.SpatialHashLevelConfig{Shift: 8, MaxCellsPerObject: 64},
			))
			query   hshg.AABB3[float64]     = hshg.AABB3[float64]{X1: 250, Y1: 250, Z1: 250, X2: 390, Y2: 390, Z2: 390}
			results []*benchmarkCollidable3 = hash.Retrieve(&query)
		)

		b.ReportAllocs()
		for b.Loop() {
			results = hash.RetrieveInto(results, &query)
		}

		benchmarkResults3 = results
	})
	b.Run("Shifts_5_7_9", func(b *testing.B) {
		benchmarkSpatialHash3Scenario(b, 16, hshg.AABB3[float64]{X1: 250, Y1: 250, Z1: 250, X2: 390, Y2: 390, Z2: 390})
	})
	b.Run("Shifts_6_8_10", func(b *testing.B) {
		var (
			hash *hshg.SpatialHash3[*benchmarkCollidable3, float64] = buildBenchmarkSpatialHash3(16, hshg.WithSpatialHash3Levels(
				hshg.SpatialHashLevelConfig{Shift: 6, MaxCellsPerObject: 8},
				hshg.SpatialHashLevelConfig{Shift: 8, MaxCellsPerObject: 8},
				hshg.SpatialHashLevelConfig{Shift: 10, MaxCellsPerObject: 64},
			))
			query   hshg.AABB3[float64]     = hshg.AABB3[float64]{X1: 250, Y1: 250, Z1: 250, X2: 390, Y2: 390, Z2: 390}
			results []*benchmarkCollidable3 = hash.Retrieve(&query)
		)

		b.ReportAllocs()
		for b.Loop() {
			results = hash.RetrieveInto(results, &query)
		}

		benchmarkResults3 = results
	})
	b.Run("Shifts_7_9_11", func(b *testing.B) {
		benchmarkSpatialHash3ConfiguredScenario(b, hshg.WithSpatialHash3Levels(
			hshg.SpatialHashLevelConfig{Shift: 7, MaxCellsPerObject: 8},
			hshg.SpatialHashLevelConfig{Shift: 9, MaxCellsPerObject: 8},
			hshg.SpatialHashLevelConfig{Shift: 11, MaxCellsPerObject: 64},
		))
	})
	b.Run("Shifts_8_10_12", func(b *testing.B) {
		benchmarkSpatialHash3ConfiguredScenario(b, hshg.WithSpatialHash3Levels(
			hshg.SpatialHashLevelConfig{Shift: 8, MaxCellsPerObject: 8},
			hshg.SpatialHashLevelConfig{Shift: 10, MaxCellsPerObject: 8},
			hshg.SpatialHashLevelConfig{Shift: 12, MaxCellsPerObject: 64},
		))
	})
}
