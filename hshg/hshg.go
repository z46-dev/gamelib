package hshg

import "golang.org/x/exp/constraints"

func NewSpatialHash[T Collidable[U], U constraints.Float]() (sh *SpatialHash[T, U]) {
	sh = &SpatialHash[T, U]{
		generation: 1,
		levels: [hashLevels]spatialHashLevel{{
			shift:  fineShift,
			lookup: make(map[uint64]uint32, 1024),
		}, {
			shift:  mediumShift,
			lookup: make(map[uint64]uint32, 1024),
		}, {
			shift:  coarseShift,
			lookup: make(map[uint64]uint32, 1024),
		}},
	}

	return
}

// Performs mathematical floor while avoiding math.Floor (Go's math.Floor would return floor(-0.5) = 0, but we want -1).
func floorInt[T constraints.Float](v T) (i int) {
	if i = int(v); v < T(i) {
		i--
	}

	return
}

// Converts a world coordinate into a cell coordinate.
func cellCoord[T constraints.Float](v T, shift uint) (coord int) {
	coord = floorInt(v) >> shift
	return
}

// cellKey losslessly packs two signed 32-bit cell coordinates into one uint64. Casting a negative coordinate to uint32 preserves its bit pattern.
func cellKey(x, y int) (key uint64) {
	key = uint64(uint32(x))<<32 | uint64(uint32(y))
	return
}

func cellBounds[T constraints.Float](aabb *AABB[T], shift uint) (x1, y1, x2, y2 int) {
	x1, y1 = cellCoord(aabb.X1, shift), cellCoord(aabb.Y1, shift)
	x2, y2 = cellCoord(aabb.X2, shift), cellCoord(aabb.Y2, shift)
	return
}

func cellCount(x1, y1, x2, y2 int) (count int64) {
	var w, h int64 = int64(x2-x1) + 1, int64(y2-y1) + 1
	if w <= 0 || h <= 0 {
		count = 0
	} else {
		count = w * h
	}

	return
}

func intersects[T constraints.Float](a *AABB[T], x1, y1, x2, y2 T) (intersects bool) {
	intersects = a.X1 <= x2 && a.X2 >= x1 && a.Y1 <= y2 && a.Y2 >= y1
	return
}

// Clear starts a new spatial-hash build. Importantly, we do NOT delete every map entry and we do NOT destroy every
// bucket's backing array. Buckets are lazily reset when they are first reused in the new generation.
func (sh *SpatialHash[T, u]) Clear() {
	for i := range sh.levels {
		var (
			level  *spatialHashLevel = &sh.levels[i]
			active int               = sh.activeBuckets[i]
		)

		if len(level.lookup) > 4096 && len(level.lookup) > active*4 {
			var capacity int = max(active*2, 1024)
			level.lookup = make(map[uint64]uint32, capacity)
			level.buckets = level.buckets[:0]
		}

		sh.activeBuckets[i] = 0
	}

	sh.entries, sh.oversized = sh.entries[:0], sh.oversized[:0]
	sh.generation++

	if sh.generation == 0 {
		sh.generation = 1

		for i := range sh.levels {
			var level *spatialHashLevel = &sh.levels[i]
			clear(level.lookup)
			level.buckets = level.buckets[:0]
		}
	}
}

// Returns an active bucket for a cell, creating or lazily resetting it if necessary.
func (sh *SpatialHash[T, u]) getBucket(levelIndex int, key uint64) (bucket *spatialHashBucket) {
	var (
		level    *spatialHashLevel = &sh.levels[levelIndex]
		rawIndex uint32
		found    bool
	)

	if rawIndex, found = level.lookup[key]; !found {
		var index int = len(level.buckets)
		level.buckets = append(level.buckets, spatialHashBucket{generation: sh.generation})
		level.lookup[key] = uint32(index + 1)
		sh.activeBuckets[levelIndex]++
		bucket = &level.buckets[index]
		return
	}

	var index int = int(rawIndex - 1)

	// Existing cell from an older frame.
	if bucket = &level.buckets[index]; bucket.generation != sh.generation {
		bucket.generation = sh.generation
		bucket.entries = bucket.entries[:0]

		sh.activeBuckets[levelIndex]++
	}

	return
}

// chooseLevel selects the finest grid that doesn't cause excessive bucket duplication. Fine and medium objects may
// touch at most 4 cells. Coarse objects may touch up to 16 cells. Anything larger is put into the oversized list.
func (sh *SpatialHash[T, U]) chooseLevel(aabb *AABB[U]) (levelIndex int, x1, y1, x2, y2 int, ok bool) {
	for i := range sh.levels {
		var shift uint = sh.levels[i].shift

		x1, y1, x2, y2 = cellBounds(aabb, shift)

		var (
			count    int64 = cellCount(x1, y1, x2, y2)
			maxCells int64 = 4
		)

		// Coarse objects can occupy a few more cells before falling back to the oversized list.
		if i == hashLevels-1 {
			maxCells = 16
		}

		if count <= maxCells {
			levelIndex = i
			ok = true
			return
		}
	}

	ok = false
	return
}

func (sh *SpatialHash[T, U]) Insert(item T) {
	var (
		aabbPtr *AABB[U] = item.GetAABB()
		aabb    AABB[U]  = *aabbPtr
		index   uint32   = uint32(len(sh.entries))
	)

	sh.entries = append(sh.entries, spatialHashEntry[T, U]{item: item, aabb: aabb})
	var (
		levelIndex, x1, y1, x2, y2 int
		ok                         bool
	)

	if levelIndex, x1, y1, x2, y2, ok = sh.chooseLevel(&aabb); !ok {
		sh.oversized = append(sh.oversized, index)
		return
	}

	for y := y1; y <= y2; y++ {
		for x := x1; x <= x2; x++ {
			var bucket *spatialHashBucket = sh.getBucket(levelIndex, cellKey(x, y))
			bucket.entries = append(bucket.entries, index)
		}
	}
}

// queryCellCount determines roughly how many hash lookups a Retrieve would need. If a query covers a huge
// area, walking thousands of mostly-empty cells can be slower than simply scanning all cached AABBs.
func (sh *SpatialHash[T, U]) queryCellCount(x1, y1, x2, y2 U) (total int64) {
	var aabb AABB[U] = AABB[U]{X1: x1, Y1: y1, X2: x2, Y2: y2}

	for i := range sh.levels {
		var cx1, cy1, cx2, cy2 int = cellBounds(&aabb, sh.levels[i].shift)
		total += cellCount(cx1, cy1, cx2, cy2)
	}

	return
}

func (sh *SpatialHash[T, U]) retrieveLinear(x1, y1, x2, y2 U) (results []T) {
	for i := range sh.entries {
		var entry *spatialHashEntry[T, U] = &sh.entries[i]
		if intersects(&entry.aabb, x1, y1, x2, y2) {
			results = append(results, entry.item)
		}
	}

	return
}

func (sh *SpatialHash[T, U]) retrieve(x1, y1, x2, y2 U) (results []T) {
	if len(sh.entries) == 0 {
		return
	}

	var cellQueries int64 = sh.queryCellCount(x1, y1, x2, y2)
	if cellQueries > int64(len(sh.entries))/2 {
		results = sh.retrieveLinear(x1, y1, x2, y2)
		return
	}

	sh.queryGeneration++

	if sh.queryGeneration == 0 {
		for i := range sh.entries {
			sh.entries[i].seen = 0
		}

		sh.queryGeneration = 1
	}

	var (
		queryGeneration uint32  = sh.queryGeneration
		queryAABB       AABB[U] = AABB[U]{X1: x1, Y1: y1, X2: x2, Y2: y2}
	)

	for levelIndex := range sh.levels {
		var level *spatialHashLevel = &sh.levels[levelIndex]

		var cx1, cy1, cx2, cy2 int = cellBounds(&queryAABB, level.shift)
		for cy := cy1; cy <= cy2; cy++ {
			for cx := cx1; cx <= cx2; cx++ {
				var (
					rawIndex uint32
					found    bool
				)

				if rawIndex, found = level.lookup[cellKey(cx, cy)]; !found {
					continue
				}

				var bucket *spatialHashBucket = &level.buckets[int(rawIndex-1)]
				if bucket.generation != sh.generation {
					continue
				}

				for _, entryIndex := range bucket.entries {
					var entry *spatialHashEntry[T, U] = &sh.entries[entryIndex]
					if entry.seen == queryGeneration {
						continue
					}

					entry.seen = queryGeneration
					if intersects(&entry.aabb, x1, y1, x2, y2) {
						results = append(results, entry.item)
					}
				}
			}
		}
	}

	for _, entryIndex := range sh.oversized {
		var entry *spatialHashEntry[T, U] = &sh.entries[entryIndex]
		if intersects(&entry.aabb, x1, y1, x2, y2) {
			results = append(results, entry.item)
		}
	}

	return
}

func (sh *SpatialHash[T, U]) Retrieve(aabb *AABB[U]) (results []T) {
	results = sh.retrieve(aabb.X1, aabb.Y1, aabb.X2, aabb.Y2)
	return
}

func (sh *SpatialHash[T, U]) RetrieveAround(x, y, radius U) (results []T) {
	results = sh.retrieve(x-radius, y-radius, x+radius, y+radius)
	return
}

func (sh *SpatialHash[T, U]) All() (results []T) {
	if len(sh.entries) == 0 {
		return
	}

	results = make([]T, len(sh.entries))
	for i := range sh.entries {
		results[i] = sh.entries[i].item
	}

	return
}
