package hshg_test

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/z46-dev/gamelib/hshg"
	"golang.org/x/exp/constraints"
)

type (
	testCollidable2 struct {
		ID     int
		Bounds hshg.AABB2[float64]
	}

	testCollidable3 struct {
		ID     int
		Bounds hshg.AABB3[float64]
	}
)

func (item *testCollidable2) GetAABB() (aabb *hshg.AABB2[float64]) {
	aabb = &item.Bounds
	return
}

func (item *testCollidable3) GetAABB() (aabb *hshg.AABB3[float64]) {
	aabb = &item.Bounds
	return
}

func copyAABB[T constraints.Float, V hshg.AABBValue[T], P hshg.AABBPtr[T, V]](bounds P) (copy *V) {
	copy = bounds.Copy()
	return
}

func rebuildSpatialHash[T any, U constraints.Float, V hshg.SpatialHashValue[T, U], P hshg.SpatialHashPtr[T, U, V]](hash P, items []T) (all []T) {
	hash.Clear()
	for _, item := range items {
		hash.Insert(item)
	}

	all = hash.All()
	return
}

func TestGenericAABBAndSpatialHashAPIs(t *testing.T) {
	var (
		bounds2 *hshg.AABB2[float64] = &hshg.AABB2[float64]{X1: 1, Y1: 2, X2: 3, Y2: 4}
		bounds3 *hshg.AABB3[float64] = &hshg.AABB3[float64]{X1: 1, Y1: 2, Z1: 3, X2: 4, Y2: 5, Z2: 6}
		copy2   *hshg.AABB2[float64] = copyAABB[float64, hshg.AABB2[float64]](bounds2)
		copy3   *hshg.AABB3[float64] = copyAABB[float64, hshg.AABB3[float64]](bounds3)
		item2   *testCollidable2     = &testCollidable2{ID: 2, Bounds: *bounds2}
		item3   *testCollidable3     = &testCollidable3{ID: 3, Bounds: *bounds3}
	)

	assert.Equal(t, bounds2, copy2)
	assert.NotSame(t, bounds2, copy2)
	assert.Equal(t, bounds3, copy3)
	assert.NotSame(t, bounds3, copy3)
	assert.Equal(t, []*testCollidable2{item2}, rebuildSpatialHash[
		*testCollidable2,
		float64,
		hshg.SpatialHash2[*testCollidable2, float64],
	](hshg.NewSpatialHash2[*testCollidable2](), []*testCollidable2{item2}))
	assert.Equal(t, []*testCollidable3{item3}, rebuildSpatialHash[
		*testCollidable3,
		float64,
		hshg.SpatialHash3[*testCollidable3, float64],
	](hshg.NewSpatialHash3[*testCollidable3](), []*testCollidable3{item3}))
}

func TestAABB2Geometry(t *testing.T) {
	var bounds hshg.AABB2[float64] = hshg.AABB2[float64]{X1: -2, Y1: -1, X2: 4, Y2: 3}

	assert.True(t, bounds.Contains(-2, -1), "boundaries should be inclusive")
	assert.True(t, bounds.Contains(0, 0))
	assert.False(t, bounds.Contains(4.01, 0))
	assert.True(t, bounds.Intersects(&hshg.AABB2[float64]{X1: 4, Y1: 0, X2: 5, Y2: 1}), "touching bounds should intersect")
	assert.False(t, bounds.Intersects(&hshg.AABB2[float64]{X1: 5, Y1: 0, X2: 6, Y2: 1}))
	assert.InDelta(t, 1, bounds.GetCenter().X, 1e-9)
	assert.InDelta(t, 1, bounds.GetCenter().Y, 1e-9)
}

func TestSpatialHash2Configuration(t *testing.T) {
	var (
		defaults hshg.SpatialHashConfig        = hshg.DefaultSpatialHash2Config()
		levels   []hshg.SpatialHashLevelConfig = []hshg.SpatialHashLevelConfig{
			{Shift: 2, MaxCellsPerObject: 2},
			{Shift: 5, MaxCellsPerObject: 8},
		}
		hash *hshg.SpatialHash2[*testCollidable2, float64]
	)

	assert.Len(t, defaults.Levels, 3)
	defaults.Levels[0].Shift = 99
	assert.NotEqual(t, defaults.Levels[0].Shift, hshg.DefaultSpatialHash2Config().Levels[0].Shift, "default configurations should not share slices")

	hash = hshg.NewSpatialHash2[*testCollidable2](hshg.WithSpatialHash2Levels(levels...))
	levels[0].Shift = 99
	assert.Equal(t, uint(2), hash.Config().Levels[0].Shift, "constructor options should detach caller-owned configuration")

	assert.Panics(t, func() {
		hshg.NewSpatialHash2[*testCollidable2](hshg.WithSpatialHash2Levels())
	})
	assert.Panics(t, func() {
		hshg.NewSpatialHash2[*testCollidable2](hshg.WithSpatialHash2Levels(hshg.SpatialHashLevelConfig{Shift: 2, MaxCellsPerObject: 0}))
	})
	assert.Panics(t, func() {
		hshg.NewSpatialHash2[*testCollidable2](hshg.WithSpatialHash2Levels(
			hshg.SpatialHashLevelConfig{Shift: 5, MaxCellsPerObject: 4},
			hshg.SpatialHashLevelConfig{Shift: 2, MaxCellsPerObject: 4},
		))
	})
}

func TestSpatialHash2RetrievalAndDeduplication(t *testing.T) {
	var (
		hash *hshg.SpatialHash2[*testCollidable2, float64] = hshg.NewSpatialHash2[*testCollidable2](
			hshg.WithSpatialHash2Levels(hshg.SpatialHashLevelConfig{Shift: 0, MaxCellsPerObject: 4}),
		)
		spanning  *testCollidable2 = &testCollidable2{ID: 1, Bounds: hshg.AABB2[float64]{X1: -0.5, Y1: -0.5, X2: 0.5, Y2: 0.5}}
		negative  *testCollidable2 = &testCollidable2{ID: 2, Bounds: hshg.AABB2[float64]{X1: -4, Y1: -4, X2: -3, Y2: -3}}
		results   []*testCollidable2
		visited   []*testCollidable2 = make([]*testCollidable2, 0, 1)
		completed bool
	)

	hash.Insert(spanning)
	hash.Insert(negative)
	for i := 0; i < 20; i++ {
		hash.Insert(&testCollidable2{
			ID:     i + 3,
			Bounds: hshg.AABB2[float64]{X1: float64(i + 10), Y1: 10, X2: float64(i + 10), Y2: 10},
		})
	}

	results = hash.Retrieve(&hshg.AABB2[float64]{X1: -1, Y1: -1, X2: 1, Y2: 1})
	assert.Equal(t, []*testCollidable2{spanning}, results, "an object stored in multiple cells should only be returned once")
	results = hash.RetrieveInto(results, &hshg.AABB2[float64]{X1: -4, Y1: -4, X2: -3, Y2: -3})
	assert.Equal(t, []*testCollidable2{negative}, results)

	completed = hash.Visit(&hshg.AABB2[float64]{X1: -1, Y1: -1, X2: 1, Y2: 1}, func(item *testCollidable2) bool {
		visited = append(visited, item)
		return true
	})
	assert.True(t, completed)
	assert.Equal(t, []*testCollidable2{spanning}, visited)
	assert.False(t, hash.Visit(&hshg.AABB2[float64]{X1: -100, Y1: -100, X2: 100, Y2: 100}, func(item *testCollidable2) bool {
		return false
	}), "Visit should stop when the visitor returns false")

	assert.Equal(t, []*testCollidable2{negative}, hash.RetrieveAround(-3.5, -3.5, 0.25))
	assert.Len(t, hash.All(), 22)
	assert.Len(t, hash.AllInto(make([]*testCollidable2, 0, 22)), 22)
}

func TestSpatialHash2OversizedEntriesAndClear(t *testing.T) {
	var (
		hash *hshg.SpatialHash2[*testCollidable2, float64] = hshg.NewSpatialHash2[*testCollidable2](
			hshg.WithSpatialHash2Levels(hshg.SpatialHashLevelConfig{Shift: 0, MaxCellsPerObject: 1}),
		)
		large *testCollidable2 = &testCollidable2{ID: 1, Bounds: hshg.AABB2[float64]{X1: -10, Y1: -10, X2: 10, Y2: 10}}
		moved *testCollidable2 = &testCollidable2{ID: 2, Bounds: hshg.AABB2[float64]{X1: 1, Y1: 1, X2: 1, Y2: 1}}
	)

	hash.Insert(large)
	hash.Insert(moved)
	assert.Equal(t, []*testCollidable2{large}, hash.RetrieveAround(0, 0, 0), "oversized entries should participate in hashed queries")
	assert.ElementsMatch(t, []*testCollidable2{large, moved}, hash.RetrieveAround(0, 0, 2))

	moved.Bounds = hshg.AABB2[float64]{X1: 100, Y1: 100, X2: 100, Y2: 100}
	assert.Contains(t, hash.RetrieveAround(1, 1, 0.1), moved, "insertion should snapshot mutable bounds")

	hash.Clear()
	hash.Insert(moved)
	assert.Empty(t, hash.RetrieveAround(0, 0, 20))
	assert.Equal(t, []*testCollidable2{moved}, hash.RetrieveAround(100, 100, 0.1))
	assert.Equal(t, []*testCollidable2{moved}, hash.All())
}

func TestAABB3Geometry(t *testing.T) {
	var bounds hshg.AABB3[float64] = hshg.AABB3[float64]{X1: -2, Y1: -1, Z1: -3, X2: 4, Y2: 3, Z2: 5}

	assert.True(t, bounds.Contains(-2, -1, -3), "boundaries should be inclusive")
	assert.True(t, bounds.Contains(0, 0, 0))
	assert.False(t, bounds.Contains(0, 0, 5.01))
	assert.True(t, bounds.Intersects(&hshg.AABB3[float64]{X1: 4, Y1: 0, Z1: 0, X2: 5, Y2: 1, Z2: 1}), "touching bounds should intersect")
	assert.False(t, bounds.Intersects(&hshg.AABB3[float64]{X1: 5, Y1: 0, Z1: 0, X2: 6, Y2: 1, Z2: 1}))
	assert.InDelta(t, 1, bounds.GetCenter().X, 1e-9)
	assert.InDelta(t, 1, bounds.GetCenter().Y, 1e-9)
	assert.InDelta(t, 1, bounds.GetCenter().Z, 1e-9)
}

func TestSpatialHash3Configuration(t *testing.T) {
	var (
		defaults hshg.SpatialHashConfig        = hshg.DefaultSpatialHash3Config()
		levels   []hshg.SpatialHashLevelConfig = []hshg.SpatialHashLevelConfig{
			{Shift: 2, MaxCellsPerObject: 8},
			{Shift: 5, MaxCellsPerObject: 64},
		}
		hash *hshg.SpatialHash3[*testCollidable3, float64]
	)

	assert.Equal(t, int64(8), defaults.Levels[0].MaxCellsPerObject)
	assert.Equal(t, int64(64), defaults.Levels[2].MaxCellsPerObject)
	defaults.Levels[0].Shift = 99
	assert.NotEqual(t, defaults.Levels[0].Shift, hshg.DefaultSpatialHash3Config().Levels[0].Shift, "default configurations should not share slices")

	hash = hshg.NewSpatialHash3[*testCollidable3](hshg.WithSpatialHash3Levels(levels...))
	levels[0].Shift = 99
	assert.Equal(t, uint(2), hash.Config().Levels[0].Shift, "constructor options should detach caller-owned configuration")

	assert.Panics(t, func() {
		hshg.NewSpatialHash3[*testCollidable3](hshg.WithSpatialHash3Levels())
	})
}

func TestSpatialHash3RetrievalAndDeduplication(t *testing.T) {
	var (
		hash *hshg.SpatialHash3[*testCollidable3, float64] = hshg.NewSpatialHash3[*testCollidable3](
			hshg.WithSpatialHash3Levels(hshg.SpatialHashLevelConfig{Shift: 0, MaxCellsPerObject: 8}),
		)
		spanning  *testCollidable3 = &testCollidable3{ID: 1, Bounds: hshg.AABB3[float64]{X1: -0.5, Y1: -0.5, Z1: -0.5, X2: 0.5, Y2: 0.5, Z2: 0.5}}
		negative  *testCollidable3 = &testCollidable3{ID: 2, Bounds: hshg.AABB3[float64]{X1: -4, Y1: -4, Z1: -4, X2: -3, Y2: -3, Z2: -3}}
		results   []*testCollidable3
		visited   []*testCollidable3 = make([]*testCollidable3, 0, 1)
		completed bool
	)

	hash.Insert(spanning)
	hash.Insert(negative)
	for i := 0; i < 100; i++ {
		hash.Insert(&testCollidable3{
			ID: i + 3,
			Bounds: hshg.AABB3[float64]{
				X1: float64(i + 10),
				Y1: 10,
				Z1: 10,
				X2: float64(i + 10),
				Y2: 10,
				Z2: 10,
			},
		})
	}

	results = hash.Retrieve(&hshg.AABB3[float64]{X1: -1, Y1: -1, Z1: -1, X2: 1, Y2: 1, Z2: 1})
	assert.Equal(t, []*testCollidable3{spanning}, results, "an object stored in multiple cells should only be returned once")
	results = hash.RetrieveInto(results, &hshg.AABB3[float64]{X1: -4, Y1: -4, Z1: -4, X2: -3, Y2: -3, Z2: -3})
	assert.Equal(t, []*testCollidable3{negative}, results)

	completed = hash.Visit(&hshg.AABB3[float64]{X1: -1, Y1: -1, Z1: -1, X2: 1, Y2: 1, Z2: 1}, func(item *testCollidable3) bool {
		visited = append(visited, item)
		return true
	})
	assert.True(t, completed)
	assert.Equal(t, []*testCollidable3{spanning}, visited)
	assert.False(t, hash.Visit(&hshg.AABB3[float64]{X1: -100, Y1: -100, Z1: -100, X2: 100, Y2: 100, Z2: 100}, func(item *testCollidable3) bool {
		return false
	}), "Visit should stop when the visitor returns false")

	assert.Equal(t, []*testCollidable3{negative}, hash.RetrieveAround(-3.5, -3.5, -3.5, 0.25))
	assert.Len(t, hash.All(), 102)
	assert.Len(t, hash.AllInto(make([]*testCollidable3, 0, 102)), 102)
}

func TestSpatialHash3SeparatesDepthAndClears(t *testing.T) {
	var (
		hash *hshg.SpatialHash3[*testCollidable3, float64] = hshg.NewSpatialHash3[*testCollidable3](
			hshg.WithSpatialHash3Levels(hshg.SpatialHashLevelConfig{Shift: 0, MaxCellsPerObject: 1}),
		)
		front *testCollidable3 = &testCollidable3{ID: 1, Bounds: hshg.AABB3[float64]{X1: 1, Y1: 1, Z1: 1, X2: 1, Y2: 1, Z2: 1}}
		back  *testCollidable3 = &testCollidable3{ID: 2, Bounds: hshg.AABB3[float64]{X1: 1, Y1: 1, Z1: 100, X2: 1, Y2: 1, Z2: 100}}
		large *testCollidable3 = &testCollidable3{ID: 3, Bounds: hshg.AABB3[float64]{X1: -10, Y1: -10, Z1: -10, X2: 10, Y2: 10, Z2: 10}}
	)

	hash.Insert(front)
	hash.Insert(back)
	hash.Insert(large)
	assert.ElementsMatch(t, []*testCollidable3{front, large}, hash.RetrieveAround(1, 1, 1, 0))
	assert.Equal(t, []*testCollidable3{back}, hash.RetrieveAround(1, 1, 100, 0))

	back.Bounds = hshg.AABB3[float64]{X1: 200, Y1: 200, Z1: 200, X2: 200, Y2: 200, Z2: 200}
	assert.Contains(t, hash.RetrieveAround(1, 1, 100, 0), back, "insertion should snapshot mutable bounds")

	hash.Clear()
	hash.Insert(back)
	assert.Empty(t, hash.RetrieveAround(1, 1, 100, 0))
	assert.Equal(t, []*testCollidable3{back}, hash.RetrieveAround(200, 200, 200, 0))
	assert.Equal(t, []*testCollidable3{back}, hash.All())
}
