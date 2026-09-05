package hshg

import (
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

// newSpatialHash3Lookup creates a power-of-two open-addressed cell table.
func newSpatialHash3Lookup(capacity int) (lookup spatialHash3Lookup) {
	var size int = 16
	for size < capacity*2 {
		size <<= 1
	}
	lookup.keys = make([]spatialHash3Cell, size)
	lookup.values = make([]uint32, size)
	return
}

// hashSpatialHash3Cell mixes three signed coordinates into one probe seed.
func hashSpatialHash3Cell(cell spatialHash3Cell) (hash uint64) {
	hash = uint64(int64(cell.X))*0x9e3779b185ebca87 ^ uint64(int64(cell.Y))*0xc2b2ae3d27d4eb4f ^ uint64(int64(cell.Z))*0x165667b19e3779f9 // #nosec G115 -- signed coordinate bit patterns are intentionally mixed as unsigned hash input.
	hash ^= hash >> 33
	hash *= 0xff51afd7ed558ccd
	hash ^= hash >> 33
	return
}

// get retrieves a cell's stable bucket index.
func (l *spatialHash3Lookup) get(key spatialHash3Cell) (value uint32, found bool) {
	if len(l.values) == 0 {
		return
	}
	var index int = int(hashSpatialHash3Cell(key) & uint64(len(l.values)-1)) // #nosec G115 -- the mask bounds the result to this allocated slice.
	for l.values[index] != 0 {
		if l.keys[index] == key {
			value, found = l.values[index], true
			return
		}
		index = (index + 1) & (len(l.values) - 1)
	}
	return
}

// set inserts a cell and grows the table before probe chains become dense.
func (l *spatialHash3Lookup) set(key spatialHash3Cell, value uint32) {
	if len(l.values) == 0 {
		*l = newSpatialHash3Lookup(16)
	}
	if (l.used+1)*10 >= len(l.values)*7 {
		l.grow()
	}
	var index int = int(hashSpatialHash3Cell(key) & uint64(len(l.values)-1)) // #nosec G115 -- the mask bounds the result to this allocated slice.
	for l.values[index] != 0 {
		if l.keys[index] == key {
			l.values[index] = value
			return
		}
		index = (index + 1) & (len(l.values) - 1)
	}
	l.keys[index], l.values[index] = key, value
	l.used++
}

// grow doubles the lookup while preserving stable bucket values.
func (l *spatialHash3Lookup) grow() {
	var grown spatialHash3Lookup = newSpatialHash3Lookup(len(l.values))
	for i := range l.values {
		if l.values[i] != 0 {
			grown.set(l.keys[i], l.values[i])
		}
	}
	*l = grown
}

// clear removes every key while retaining table storage.
func (l *spatialHash3Lookup) clear() {
	clear(l.values)
	l.used = 0
}

// Contains reports whether a point lies inside or on the boundary of the AABB3.
func (a *AABB3[T]) Contains(x, y, z T) (contains bool) {
	contains = x >= a.X1 && x <= a.X2 && y >= a.Y1 && y <= a.Y2 && z >= a.Z1 && z <= a.Z2
	return
}

// Copy returns a detached copy of the AABB3.
func (a *AABB3[T]) Copy() (copy *AABB3[T]) {
	copy = &AABB3[T]{X1: a.X1, Y1: a.Y1, Z1: a.Z1, X2: a.X2, Y2: a.Y2, Z2: a.Z2}
	return
}

// CopyInto copies the AABB3 into destination and returns the source.
func (a *AABB3[T]) CopyInto(destination *AABB3[T]) (self *AABB3[T]) {
	destination.X1, destination.Y1, destination.Z1 = a.X1, a.Y1, a.Z1
	destination.X2, destination.Y2, destination.Z2 = a.X2, a.Y2, a.Z2
	self = a
	return
}

// Intersects reports whether two AABB3 values overlap or touch.
func (a *AABB3[T]) Intersects(b *AABB3[T]) (intersects bool) {
	intersects = a.X1 <= b.X2 && a.X2 >= b.X1 && a.Y1 <= b.Y2 && a.Y2 >= b.Y1 && a.Z1 <= b.Z2 && a.Z2 >= b.Z1
	return
}

// GetCenter returns the center point of the AABB3.
func (a *AABB3[T]) GetCenter() (center *vector.Vec3[T]) {
	center = vector.NewVec3((a.X1+a.X2)/2, (a.Y1+a.Y2)/2, (a.Z1+a.Z2)/2)
	return
}

// DefaultSpatialHash3Config returns a fresh copy of the default three-dimensional level configuration.
func DefaultSpatialHash3Config() (config SpatialHashConfig) {
	config.Levels = []SpatialHashLevelConfig{
		{Shift: defaultBaseShift - 4, MaxCellsPerObject: 8},
		{Shift: defaultBaseShift - 2, MaxCellsPerObject: 8},
		{Shift: defaultBaseShift, MaxCellsPerObject: 64},
	}

	return
}

// WithSpatialHash3Config replaces the default constructor configuration.
func WithSpatialHash3Config(config SpatialHashConfig) (option SpatialHash3Option) {
	var levels []SpatialHashLevelConfig = append([]SpatialHashLevelConfig(nil), config.Levels...)

	option = func(destination *SpatialHashConfig) {
		destination.Levels = append(destination.Levels[:0], levels...)
	}

	return
}

// WithSpatialHash3Levels replaces the default grid levels.
func WithSpatialHash3Levels(levels ...SpatialHashLevelConfig) (option SpatialHash3Option) {
	option = WithSpatialHash3Config(SpatialHashConfig{Levels: levels})
	return
}

// NewSpatialHash3 creates a three-dimensional spatial hash with optional level configuration.
func NewSpatialHash3[T Collidable3[U], U constraints.Float](options ...SpatialHash3Option) (sh *SpatialHash3[T, U]) {
	var config SpatialHashConfig = DefaultSpatialHash3Config()
	for _, option := range options {
		option(&config)
	}

	config.Levels = append([]SpatialHashLevelConfig(nil), config.Levels...)
	validateSpatialHashConfig(&config, "SpatialHash3")

	sh = &SpatialHash3[T, U]{
		generation:      1,
		levels:          make([]spatialHash3Level, len(config.Levels)),
		activeBuckets:   make([]int, len(config.Levels)),
		entryReferences: make([]int64, len(config.Levels)),
		getAABB: func(item T) (aabb *AABB3[U]) {
			aabb = item.GetAABB()
			return
		},
	}

	for i := range config.Levels {
		sh.levels[i] = spatialHash3Level{
			shift:             config.Levels[i].Shift,
			maxCellsPerObject: config.Levels[i].MaxCellsPerObject,
			lookup:            newSpatialHash3Lookup(1024),
		}
	}

	return
}

// Config returns a detached copy of the spatial hash's active configuration.
func (sh *SpatialHash3[T, U]) Config() (config SpatialHashConfig) {
	config.Levels = make([]SpatialHashLevelConfig, len(sh.levels))
	for i := range sh.levels {
		config.Levels[i] = SpatialHashLevelConfig{
			Shift:             sh.levels[i].shift,
			MaxCellsPerObject: sh.levels[i].maxCellsPerObject,
		}
	}

	return
}

// cellBounds3 returns the inclusive 3D cell range touched by an AABB.
func cellBounds3[T constraints.Float](aabb *AABB3[T], shift uint) (x1, y1, z1, x2, y2, z2 int) {
	x1, y1, z1 = cellCoord(aabb.X1, shift), cellCoord(aabb.Y1, shift), cellCoord(aabb.Z1, shift)
	x2, y2, z2 = cellCoord(aabb.X2, shift), cellCoord(aabb.Y2, shift), cellCoord(aabb.Z2, shift)
	return
}

// cellCount3 returns the number of cells in an inclusive 3D cell range.
func cellCount3(x1, y1, z1, x2, y2, z2 int) (count int64) {
	var (
		width  int64 = int64(x2-x1) + 1
		height int64 = int64(y2-y1) + 1
		depth  int64 = int64(z2-z1) + 1
	)

	if width <= 0 || height <= 0 || depth <= 0 {
		count = 0
	} else {
		count = width * height * depth
	}

	return
}

// intersects3 reports whether an AABB overlaps raw 3D bounds.
func intersects3[T constraints.Float](a *AABB3[T], x1, y1, z1, x2, y2, z2 T) (intersects bool) {
	intersects = a.X1 <= x2 && a.X2 >= x1 && a.Y1 <= y2 && a.Y2 >= y1 && a.Z1 <= z2 && a.Z2 >= z1
	return
}

// Clear starts a new spatial-hash build while retaining reusable bucket allocations.
func (sh *SpatialHash3[T, U]) Clear() {
	for i := range sh.levels {
		var (
			level  *spatialHash3Level = &sh.levels[i]
			active int                = sh.activeBuckets[i]
		)

		if level.lookup.used > 4096 && level.lookup.used > active*4 {
			var capacity int = max(active*2, 1024)
			level.lookup = newSpatialHash3Lookup(capacity)
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
			var level *spatialHash3Level = &sh.levels[i]
			level.lookup.clear()
			level.buckets = level.buckets[:0]
		}
	}
}

// getBucket returns an active bucket for a cell, creating or lazily resetting it when necessary.
func (sh *SpatialHash3[T, U]) getBucket(levelIndex int, key spatialHash3Cell) (bucket *spatialHash3Bucket) {
	var (
		level    *spatialHash3Level = &sh.levels[levelIndex]
		rawIndex uint32
		found    bool
	)

	if rawIndex, found = level.lookup.get(key); !found {
		var index int = len(level.buckets)
		level.buckets = append(level.buckets, spatialHash3Bucket{generation: sh.generation})
		level.lookup.set(key, uint32(index+1)) // #nosec G115 -- a slice cannot approach uint32 capacity within supported process memory.
		sh.activeBuckets[levelIndex]++
		bucket = &level.buckets[index]
		return
	}

	var index int = int(rawIndex - 1)
	if bucket = &level.buckets[index]; bucket.generation != sh.generation {
		bucket.generation = sh.generation
		bucket.entries = bucket.entries[:0]
		sh.activeBuckets[levelIndex]++
	}

	return
}

// chooseLevel selects the finest configured grid that does not exceed its bucket duplication limit.
func (sh *SpatialHash3[T, U]) chooseLevel(aabb *AABB3[U]) (levelIndex int, x1, y1, z1, x2, y2, z2 int, ok bool) {
	for i := range sh.levels {
		var shift uint = sh.levels[i].shift

		x1, y1, z1, x2, y2, z2 = cellBounds3(aabb, shift)
		if cellCount3(x1, y1, z1, x2, y2, z2) <= sh.levels[i].maxCellsPerObject {
			levelIndex = i
			ok = true
			return
		}
	}

	ok = false
	return
}

// Insert adds an item and snapshots its current bounds for this spatial-hash build.
func (sh *SpatialHash3[T, U]) Insert(item T) {
	var (
		aabbPtr *AABB3[U] = sh.getAABB(item)
		aabb    AABB3[U]  = *aabbPtr
		index   uint32    = uint32(len(sh.entries)) // #nosec G115 -- uint32 indexes halve broad-phase reference storage; process memory bounds the slice first.
	)

	sh.entries = append(sh.entries, spatialHash3Entry[T, U]{item: item, aabb: aabb})
	var (
		levelIndex, x1, y1, z1, x2, y2, z2 int
		ok                                 bool
	)

	if levelIndex, x1, y1, z1, x2, y2, z2, ok = sh.chooseLevel(&aabb); !ok {
		sh.oversized = append(sh.oversized, index)
		return
	}

	for z := z1; z <= z2; z++ {
		for y := y1; y <= y2; y++ {
			for x := x1; x <= x2; x++ {
				var bucket *spatialHash3Bucket = sh.getBucket(levelIndex, spatialHash3Cell{X: x, Y: y, Z: z})
				bucket.entries = append(bucket.entries, index)
				sh.entryReferences[levelIndex]++
			}
		}
	}
}

// estimatedHashCost estimates cell lookups and candidate checks for query-path selection.
func (sh *SpatialHash3[T, U]) estimatedHashCost(x1, y1, z1, x2, y2, z2 U) (total int64) {
	var aabb AABB3[U] = AABB3[U]{X1: x1, Y1: y1, Z1: z1, X2: x2, Y2: y2, Z2: z2}

	for i := range sh.levels {
		var (
			cx1, cy1, cz1, cx2, cy2, cz2 int   = cellBounds3(&aabb, sh.levels[i].shift)
			cells                        int64 = cellCount3(cx1, cy1, cz1, cx2, cy2, cz2)
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
func (sh *SpatialHash3[T, U]) retrieveLinear(results []T, x1, y1, z1, x2, y2, z2 U) (retrieved []T) {
	retrieved = results
	for i := range sh.entries {
		var entry *spatialHash3Entry[T, U] = &sh.entries[i]
		if intersects3(&entry.aabb, x1, y1, z1, x2, y2, z2) {
			if cap(retrieved) == 0 {
				retrieved = make([]T, 0, min(64, len(sh.entries)))
			}

			retrieved = append(retrieved, entry.item)
		}
	}

	return
}

// retrieve selects a hashed or linear query and replaces results with matching items.
func (sh *SpatialHash3[T, U]) retrieve(results []T, x1, y1, z1, x2, y2, z2 U) (retrieved []T) {
	retrieved = results[:0]
	if len(sh.entries) == 0 {
		return
	}

	if sh.estimatedHashCost(x1, y1, z1, x2, y2, z2) >= int64(len(sh.entries)) {
		retrieved = sh.retrieveLinear(retrieved, x1, y1, z1, x2, y2, z2)
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
		queryAABB       AABB3[U] = AABB3[U]{X1: x1, Y1: y1, Z1: z1, X2: x2, Y2: y2, Z2: z2}
	)

	for levelIndex := range sh.levels {
		var (
			level                        *spatialHash3Level = &sh.levels[levelIndex]
			cx1, cy1, cz1, cx2, cy2, cz2 int
		)

		cx1, cy1, cz1, cx2, cy2, cz2 = cellBounds3(&queryAABB, level.shift)
		for cz := cz1; cz <= cz2; cz++ {
			for cy := cy1; cy <= cy2; cy++ {
				for cx := cx1; cx <= cx2; cx++ {
					var (
						rawIndex uint32
						found    bool
					)

					if rawIndex, found = level.lookup.get(spatialHash3Cell{X: cx, Y: cy, Z: cz}); !found {
						continue
					}

					var bucket *spatialHash3Bucket = &level.buckets[int(rawIndex-1)]
					if bucket.generation != sh.generation {
						continue
					}

					for _, entryIndex := range bucket.entries {
						var entry *spatialHash3Entry[T, U] = &sh.entries[entryIndex]
						if entry.seen == queryGeneration {
							continue
						}

						entry.seen = queryGeneration
						if intersects3(&entry.aabb, x1, y1, z1, x2, y2, z2) {
							if cap(retrieved) == 0 {
								retrieved = make([]T, 0, min(64, len(sh.entries)))
							}

							retrieved = append(retrieved, entry.item)
						}
					}
				}
			}
		}
	}

	for _, entryIndex := range sh.oversized {
		var entry *spatialHash3Entry[T, U] = &sh.entries[entryIndex]
		if intersects3(&entry.aabb, x1, y1, z1, x2, y2, z2) {
			if cap(retrieved) == 0 {
				retrieved = make([]T, 0, min(64, len(sh.entries)))
			}

			retrieved = append(retrieved, entry.item)
		}
	}

	return
}

// Retrieve returns every inserted item whose bounds intersect the supplied bounds.
func (sh *SpatialHash3[T, U]) Retrieve(aabb *AABB3[U]) (results []T) {
	results = sh.retrieve(nil, aabb.X1, aabb.Y1, aabb.Z1, aabb.X2, aabb.Y2, aabb.Z2)
	return
}

// RetrieveInto replaces results with every inserted item whose bounds intersect the supplied bounds.
func (sh *SpatialHash3[T, U]) RetrieveInto(results []T, aabb *AABB3[U]) (retrieved []T) {
	retrieved = sh.retrieve(results, aabb.X1, aabb.Y1, aabb.Z1, aabb.X2, aabb.Y2, aabb.Z2)
	return
}

// RetrieveAround returns every inserted item intersecting a cube around a point.
func (sh *SpatialHash3[T, U]) RetrieveAround(x, y, z, radius U) (results []T) {
	results = sh.retrieve(nil, x-radius, y-radius, z-radius, x+radius, y+radius, z+radius)
	return
}

// RetrieveAroundInto replaces results with every inserted item intersecting a cube around a point.
func (sh *SpatialHash3[T, U]) RetrieveAroundInto(results []T, x, y, z, radius U) (retrieved []T) {
	retrieved = sh.retrieve(results, x-radius, y-radius, z-radius, x+radius, y+radius, z+radius)
	return
}

// visit traverses matching items without building a result slice.
func (sh *SpatialHash3[T, U]) visit(x1, y1, z1, x2, y2, z2 U, visitor func(T) bool) (completed bool) {
	completed = true
	if len(sh.entries) == 0 {
		return
	}

	if sh.estimatedHashCost(x1, y1, z1, x2, y2, z2) >= int64(len(sh.entries)) {
		for i := range sh.entries {
			var entry *spatialHash3Entry[T, U] = &sh.entries[i]
			if intersects3(&entry.aabb, x1, y1, z1, x2, y2, z2) && !visitor(entry.item) {
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
		queryAABB       AABB3[U] = AABB3[U]{X1: x1, Y1: y1, Z1: z1, X2: x2, Y2: y2, Z2: z2}
	)

	for levelIndex := range sh.levels {
		var (
			level                        *spatialHash3Level = &sh.levels[levelIndex]
			cx1, cy1, cz1, cx2, cy2, cz2 int
		)

		cx1, cy1, cz1, cx2, cy2, cz2 = cellBounds3(&queryAABB, level.shift)
		for cz := cz1; cz <= cz2; cz++ {
			for cy := cy1; cy <= cy2; cy++ {
				for cx := cx1; cx <= cx2; cx++ {
					var (
						rawIndex uint32
						found    bool
					)

					if rawIndex, found = level.lookup.get(spatialHash3Cell{X: cx, Y: cy, Z: cz}); !found {
						continue
					}

					var bucket *spatialHash3Bucket = &level.buckets[int(rawIndex-1)]
					if bucket.generation != sh.generation {
						continue
					}

					for _, entryIndex := range bucket.entries {
						var entry *spatialHash3Entry[T, U] = &sh.entries[entryIndex]
						if entry.seen == queryGeneration {
							continue
						}

						entry.seen = queryGeneration
						if intersects3(&entry.aabb, x1, y1, z1, x2, y2, z2) && !visitor(entry.item) {
							completed = false
							return
						}
					}
				}
			}
		}
	}

	for _, entryIndex := range sh.oversized {
		var entry *spatialHash3Entry[T, U] = &sh.entries[entryIndex]
		if intersects3(&entry.aabb, x1, y1, z1, x2, y2, z2) && !visitor(entry.item) {
			completed = false
			return
		}
	}

	return
}

// Visit calls visitor for every inserted item intersecting the supplied bounds and stops when it returns false.
func (sh *SpatialHash3[T, U]) Visit(aabb *AABB3[U], visitor func(T) bool) (completed bool) {
	completed = sh.visit(aabb.X1, aabb.Y1, aabb.Z1, aabb.X2, aabb.Y2, aabb.Z2, visitor)
	return
}

// VisitAround calls visitor for every inserted item intersecting a cube around a point.
func (sh *SpatialHash3[T, U]) VisitAround(x, y, z, radius U, visitor func(T) bool) (completed bool) {
	completed = sh.visit(x-radius, y-radius, z-radius, x+radius, y+radius, z+radius, visitor)
	return
}

// All returns every item in insertion order.
func (sh *SpatialHash3[T, U]) All() (results []T) {
	results = sh.AllInto(nil)
	return
}

// AllInto replaces results with every item in insertion order.
func (sh *SpatialHash3[T, U]) AllInto(results []T) (all []T) {
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
