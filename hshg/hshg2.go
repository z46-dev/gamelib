package hshg

import (
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

// Contains reports whether a point lies inside or on the boundary of the AABB2.
func (a *AABB2[T]) Contains(x, y T) (contains bool) {
	contains = x >= a.X1 && x <= a.X2 && y >= a.Y1 && y <= a.Y2
	return
}

// Copy returns a detached copy of the AABB2.
func (a *AABB2[T]) Copy() (copy *AABB2[T]) {
	copy = &AABB2[T]{X1: a.X1, Y1: a.Y1, X2: a.X2, Y2: a.Y2}
	return
}

// CopyInto copies the AABB2 into destination and returns the source.
func (a *AABB2[T]) CopyInto(destination *AABB2[T]) (self *AABB2[T]) {
	destination.X1, destination.Y1, destination.X2, destination.Y2 = a.X1, a.Y1, a.X2, a.Y2
	self = a
	return
}

// Intersects reports whether two AABB2 values overlap or touch.
func (a *AABB2[T]) Intersects(b *AABB2[T]) (intersects bool) {
	intersects = a.X1 <= b.X2 && a.X2 >= b.X1 && a.Y1 <= b.Y2 && a.Y2 >= b.Y1
	return
}

// GetCenter returns the center point of the AABB2.
func (a *AABB2[T]) GetCenter() (center *vector.Vec2[T]) {
	center = vector.NewVec2((a.X1+a.X2)/2, (a.Y1+a.Y2)/2)
	return
}

// DefaultSpatialHash2Config returns a fresh copy of the default two-dimensional level configuration.
func DefaultSpatialHash2Config() (config SpatialHashConfig) {
	config.Levels = []SpatialHashLevelConfig{
		{Shift: defaultBaseShift - 3, MaxCellsPerObject: 4},
		{Shift: defaultBaseShift - 1, MaxCellsPerObject: 4},
		{Shift: defaultBaseShift + 1, MaxCellsPerObject: 16},
	}

	return
}

// WithSpatialHash2Config replaces the default constructor configuration.
func WithSpatialHash2Config(config SpatialHashConfig) (option SpatialHash2Option) {
	var levels []SpatialHashLevelConfig = append([]SpatialHashLevelConfig(nil), config.Levels...)

	option = func(destination *SpatialHashConfig) {
		destination.Levels = append(destination.Levels[:0], levels...)
	}

	return
}

// WithSpatialHash2Levels replaces the default grid levels.
func WithSpatialHash2Levels(levels ...SpatialHashLevelConfig) (option SpatialHash2Option) {
	option = WithSpatialHash2Config(SpatialHashConfig{Levels: levels})
	return
}

// validateSpatialHashConfig rejects configurations that cannot produce a valid hierarchy.
func validateSpatialHashConfig(config *SpatialHashConfig, spatialHashName string) {
	if len(config.Levels) == 0 {
		panic("hshg: " + spatialHashName + " requires at least one level")
	}

	for i := range config.Levels {
		var level SpatialHashLevelConfig = config.Levels[i]
		if level.MaxCellsPerObject <= 0 {
			panic("hshg: MaxCellsPerObject must be positive")
		}

		if i > 0 && level.Shift <= config.Levels[i-1].Shift {
			panic("hshg: level shifts must be strictly increasing")
		}
	}
}

// NewSpatialHash2 creates a two-dimensional spatial hash with optional level configuration.
func NewSpatialHash2[T Collidable2[U], U constraints.Float](options ...SpatialHash2Option) (sh *SpatialHash2[T, U]) {
	var config SpatialHashConfig = DefaultSpatialHash2Config()
	for _, option := range options {
		option(&config)
	}

	config.Levels = append([]SpatialHashLevelConfig(nil), config.Levels...)
	validateSpatialHashConfig(&config, "SpatialHash2")

	sh = &SpatialHash2[T, U]{
		generation:      1,
		levels:          make([]spatialHash2Level, len(config.Levels)),
		activeBuckets:   make([]int, len(config.Levels)),
		entryReferences: make([]int64, len(config.Levels)),
		getAABB: func(item T) (aabb *AABB2[U]) {
			aabb = item.GetAABB()
			return
		},
	}

	for i := range config.Levels {
		sh.levels[i] = spatialHash2Level{
			shift:             config.Levels[i].Shift,
			maxCellsPerObject: config.Levels[i].MaxCellsPerObject,
			lookup:            make(map[uint64]uint32, 1024),
		}
	}

	return
}

// Config returns a detached copy of the spatial hash's active configuration.
func (sh *SpatialHash2[T, U]) Config() (config SpatialHashConfig) {
	config.Levels = make([]SpatialHashLevelConfig, len(sh.levels))
	for i := range sh.levels {
		config.Levels[i] = SpatialHashLevelConfig{
			Shift:             sh.levels[i].shift,
			MaxCellsPerObject: sh.levels[i].maxCellsPerObject,
		}
	}

	return
}

// floorInt converts a float to its mathematical floor without an intermediate math.Floor call.
func floorInt[T constraints.Float](v T) (i int) {
	if i = int(v); v < T(i) {
		i--
	}

	return
}

// cellCoord converts a world coordinate into a cell coordinate at a grid shift.
func cellCoord[T constraints.Float](v T, shift uint) (coord int) {
	coord = floorInt(v) >> shift
	return
}

// cellKey losslessly packs two signed 32-bit cell coordinates into one uint64. Casting a negative coordinate to uint32 preserves its bit pattern.
func cellKey(x, y int) (key uint64) {
	key = uint64(uint32(x))<<32 | uint64(uint32(y)) // #nosec G115 -- the low signed-coordinate bits form the intentionally compact hash key.
	return
}

// cellBounds returns the inclusive 2D cell range touched by an AABB.
func cellBounds[T constraints.Float](aabb *AABB2[T], shift uint) (x1, y1, x2, y2 int) {
	x1, y1 = cellCoord(aabb.X1, shift), cellCoord(aabb.Y1, shift)
	x2, y2 = cellCoord(aabb.X2, shift), cellCoord(aabb.Y2, shift)
	return
}

// cellCount returns the number of cells in an inclusive 2D cell range.
func cellCount(x1, y1, x2, y2 int) (count int64) {
	var w, h int64 = int64(x2-x1) + 1, int64(y2-y1) + 1
	if w <= 0 || h <= 0 {
		count = 0
	} else {
		count = w * h
	}

	return
}

// intersects reports whether an AABB overlaps raw 2D bounds.
func intersects[T constraints.Float](a *AABB2[T], x1, y1, x2, y2 T) (intersects bool) {
	intersects = a.X1 <= x2 && a.X2 >= x1 && a.Y1 <= y2 && a.Y2 >= y1
	return
}

// Clear starts a new spatial-hash build. Importantly, we do NOT delete every map entry and we do NOT destroy every
// bucket's backing array. Buckets are lazily reset when they are first reused in the new generation.
func (sh *SpatialHash2[T, U]) Clear() {
	for i := range sh.levels {
		var (
			level  *spatialHash2Level = &sh.levels[i]
			active int                = sh.activeBuckets[i]
		)

		if len(level.lookup) > 4096 && len(level.lookup) > active*4 {
			var capacity int = max(active*2, 1024)
			level.lookup = make(map[uint64]uint32, capacity)
			level.buckets = level.buckets[:0]
		}

		sh.activeBuckets[i] = 0
		sh.entryReferences[i] = 0
	}

	sh.entries, sh.oversized = sh.entries[:0], sh.oversized[:0]
	sh.generation++

	if sh.generation == 0 {
		sh.generation = 1

		for i := range sh.levels {
			var level *spatialHash2Level = &sh.levels[i]
			clear(level.lookup)
			level.buckets = level.buckets[:0]
		}
	}
}

// getBucket returns an active bucket, creating or lazily resetting it when necessary.
func (sh *SpatialHash2[T, U]) getBucket(levelIndex int, key uint64) (bucket *spatialHash2Bucket) {
	var (
		level    *spatialHash2Level = &sh.levels[levelIndex]
		rawIndex uint32
		found    bool
	)

	if rawIndex, found = level.lookup[key]; !found {
		var index int = len(level.buckets)
		level.buckets = append(level.buckets, spatialHash2Bucket{generation: sh.generation})
		level.lookup[key] = uint32(index + 1) // #nosec G115 -- a slice cannot approach uint32 capacity within supported process memory.
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

// chooseLevel selects the finest configured grid that does not exceed its bucket duplication limit.
func (sh *SpatialHash2[T, U]) chooseLevel(aabb *AABB2[U]) (levelIndex int, x1, y1, x2, y2 int, ok bool) {
	for i := range sh.levels {
		var shift uint = sh.levels[i].shift

		x1, y1, x2, y2 = cellBounds(aabb, shift)
		if cellCount(x1, y1, x2, y2) <= sh.levels[i].maxCellsPerObject {
			levelIndex = i
			ok = true
			return
		}
	}

	ok = false
	return
}

// Insert adds an item and snapshots its current bounds for this spatial-hash build.
func (sh *SpatialHash2[T, U]) Insert(item T) {
	var (
		aabbPtr *AABB2[U] = sh.getAABB(item)
		aabb    AABB2[U]  = *aabbPtr
		index   uint32    = uint32(len(sh.entries)) // #nosec G115 -- uint32 indexes halve broad-phase reference storage; process memory bounds the slice first.
	)

	sh.entries = append(sh.entries, spatialHash2Entry[T, U]{item: item, aabb: aabb})
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
			var bucket *spatialHash2Bucket = sh.getBucket(levelIndex, cellKey(x, y))
			bucket.entries = append(bucket.entries, index)
			sh.entryReferences[levelIndex]++
		}
	}
}

// estimatedHashCost estimates cell lookups and candidate checks for query-path selection.
func (sh *SpatialHash2[T, U]) estimatedHashCost(x1, y1, x2, y2 U) (total int64) {
	var aabb AABB2[U] = AABB2[U]{X1: x1, Y1: y1, X2: x2, Y2: y2}

	for i := range sh.levels {
		var (
			cx1, cy1, cx2, cy2 int   = cellBounds(&aabb, sh.levels[i].shift)
			cells              int64 = cellCount(cx1, cy1, cx2, cy2)
		)

		total += cells
		if sh.activeBuckets[i] > 0 {
			var averageEntries int64 = (sh.entryReferences[i] + int64(sh.activeBuckets[i]) - 1) / int64(sh.activeBuckets[i])
			total += cells * averageEntries
		}
	}

	total += int64(len(sh.oversized))

	return
}

// retrieveLinear scans every cached AABB and appends intersections to results.
func (sh *SpatialHash2[T, U]) retrieveLinear(results []T, x1, y1, x2, y2 U) (retrieved []T) {
	retrieved = results
	for i := range sh.entries {
		var entry *spatialHash2Entry[T, U] = &sh.entries[i]
		if intersects(&entry.aabb, x1, y1, x2, y2) {
			if cap(retrieved) == 0 {
				retrieved = make([]T, 0, min(64, len(sh.entries)))
			}

			retrieved = append(retrieved, entry.item)
		}
	}

	return
}

// retrieve selects a hashed or linear query and replaces results with matching items.
func (sh *SpatialHash2[T, U]) retrieve(results []T, x1, y1, x2, y2 U) (retrieved []T) {
	retrieved = results[:0]
	if len(sh.entries) == 0 {
		return
	}

	if sh.estimatedHashCost(x1, y1, x2, y2) >= int64(len(sh.entries)) {
		retrieved = sh.retrieveLinear(retrieved, x1, y1, x2, y2)
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
		queryGeneration uint32   = sh.queryGeneration
		queryAABB       AABB2[U] = AABB2[U]{X1: x1, Y1: y1, X2: x2, Y2: y2}
	)

	for levelIndex := range sh.levels {
		var level *spatialHash2Level = &sh.levels[levelIndex]

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

				var bucket *spatialHash2Bucket = &level.buckets[int(rawIndex-1)]
				if bucket.generation != sh.generation {
					continue
				}

				for _, entryIndex := range bucket.entries {
					var entry *spatialHash2Entry[T, U] = &sh.entries[entryIndex]
					if entry.seen == queryGeneration {
						continue
					}

					entry.seen = queryGeneration
					if intersects(&entry.aabb, x1, y1, x2, y2) {
						if cap(retrieved) == 0 {
							retrieved = make([]T, 0, min(64, len(sh.entries)))
						}

						retrieved = append(retrieved, entry.item)
					}
				}
			}
		}
	}

	for _, entryIndex := range sh.oversized {
		var entry *spatialHash2Entry[T, U] = &sh.entries[entryIndex]
		if intersects(&entry.aabb, x1, y1, x2, y2) {
			if cap(retrieved) == 0 {
				retrieved = make([]T, 0, min(64, len(sh.entries)))
			}

			retrieved = append(retrieved, entry.item)
		}
	}

	return
}

// Retrieve returns every inserted item whose bounds intersect the supplied bounds.
func (sh *SpatialHash2[T, U]) Retrieve(aabb *AABB2[U]) (results []T) {
	results = sh.retrieve(nil, aabb.X1, aabb.Y1, aabb.X2, aabb.Y2)
	return
}

// RetrieveInto replaces results with every inserted item whose bounds intersect the supplied bounds.
func (sh *SpatialHash2[T, U]) RetrieveInto(results []T, aabb *AABB2[U]) (retrieved []T) {
	retrieved = sh.retrieve(results, aabb.X1, aabb.Y1, aabb.X2, aabb.Y2)
	return
}

// RetrieveAround returns every inserted item intersecting a square around a point.
func (sh *SpatialHash2[T, U]) RetrieveAround(x, y, radius U) (results []T) {
	results = sh.retrieve(nil, x-radius, y-radius, x+radius, y+radius)
	return
}

// RetrieveAroundInto replaces results with every inserted item intersecting a square around a point.
func (sh *SpatialHash2[T, U]) RetrieveAroundInto(results []T, x, y, radius U) (retrieved []T) {
	retrieved = sh.retrieve(results, x-radius, y-radius, x+radius, y+radius)
	return
}

// visit traverses matching items without building a result slice.
func (sh *SpatialHash2[T, U]) visit(x1, y1, x2, y2 U, visitor func(T) bool) (completed bool) {
	completed = true
	if len(sh.entries) == 0 {
		return
	}

	if sh.estimatedHashCost(x1, y1, x2, y2) >= int64(len(sh.entries)) {
		for i := range sh.entries {
			var entry *spatialHash2Entry[T, U] = &sh.entries[i]
			if intersects(&entry.aabb, x1, y1, x2, y2) && !visitor(entry.item) {
				completed = false
				return
			}
		}

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
		queryGeneration uint32   = sh.queryGeneration
		queryAABB       AABB2[U] = AABB2[U]{X1: x1, Y1: y1, X2: x2, Y2: y2}
	)

	for levelIndex := range sh.levels {
		var level *spatialHash2Level = &sh.levels[levelIndex]
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

				var bucket *spatialHash2Bucket = &level.buckets[int(rawIndex-1)]
				if bucket.generation != sh.generation {
					continue
				}

				for _, entryIndex := range bucket.entries {
					var entry *spatialHash2Entry[T, U] = &sh.entries[entryIndex]
					if entry.seen == queryGeneration {
						continue
					}

					entry.seen = queryGeneration
					if intersects(&entry.aabb, x1, y1, x2, y2) && !visitor(entry.item) {
						completed = false
						return
					}
				}
			}
		}
	}

	for _, entryIndex := range sh.oversized {
		var entry *spatialHash2Entry[T, U] = &sh.entries[entryIndex]
		if intersects(&entry.aabb, x1, y1, x2, y2) && !visitor(entry.item) {
			completed = false
			return
		}
	}

	return
}

// Visit calls visitor for every inserted item intersecting the supplied bounds and stops when it returns false.
func (sh *SpatialHash2[T, U]) Visit(aabb *AABB2[U], visitor func(T) bool) (completed bool) {
	completed = sh.visit(aabb.X1, aabb.Y1, aabb.X2, aabb.Y2, visitor)
	return
}

// VisitAround calls visitor for every inserted item intersecting a square around a point.
func (sh *SpatialHash2[T, U]) VisitAround(x, y, radius U, visitor func(T) bool) (completed bool) {
	completed = sh.visit(x-radius, y-radius, x+radius, y+radius, visitor)
	return
}

// All returns every item in insertion order.
func (sh *SpatialHash2[T, U]) All() (results []T) {
	results = sh.AllInto(nil)
	return
}

// AllInto replaces results with every item in insertion order.
func (sh *SpatialHash2[T, U]) AllInto(results []T) (all []T) {
	if cap(results) < len(sh.entries) {
		all = make([]T, len(sh.entries))
	} else {
		all = results[:len(sh.entries)]
	}

	for i := range sh.entries {
		all[i] = sh.entries[i].item
	}

	return
}
