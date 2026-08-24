package hshg

import "golang.org/x/exp/constraints"

const defaultBaseShift = 10

type (
	// SpatialHashLevelConfig controls one grid resolution and its per-object bucket duplication limit.
	SpatialHashLevelConfig struct {
		Shift             uint
		MaxCellsPerObject int64
	}

	// SpatialHashConfig contains the ordered grid levels used by a spatial hash.
	SpatialHashConfig struct {
		Levels []SpatialHashLevelConfig
	}

	// SpatialHash2Option customizes a SpatialHash2 during construction.
	SpatialHash2Option func(*SpatialHashConfig)

	// SpatialHash3Option customizes a SpatialHash3 during construction.
	SpatialHash3Option func(*SpatialHashConfig)

	// AABB2 represents a two-dimensional axis-aligned bounding box.
	AABB2[T constraints.Float] struct {
		X1, Y1, X2, Y2 T
	}

	// AABB3 represents a three-dimensional axis-aligned bounding box.
	AABB3[T constraints.Float] struct {
		X1, Y1, Z1, X2, Y2, Z2 T
	}

	// AABBValue represents a supported axis-aligned bounding-box value.
	AABBValue[T constraints.Float] interface {
		AABB2[T] | AABB3[T]
	}

	// AABBPtr represents a pointer to a supported axis-aligned bounding-box value.
	AABBPtr[T constraints.Float, V AABBValue[T]] interface {
		*V
		Copy() *V
		CopyInto(*V) *V
		Intersects(*V) bool
	}

	// Collidable2 supplies the two-dimensional bounds required by SpatialHash2.
	Collidable2[T constraints.Float] interface {
		GetAABB() *AABB2[T]
	}

	// Collidable3 supplies the three-dimensional bounds required by SpatialHash3.
	Collidable3[T constraints.Float] interface {
		GetAABB() *AABB3[T]
	}

	// spatialHash2Entry is the canonical representation of an item for one spatial-hash build.
	spatialHash2Entry[T any, U constraints.Float] struct {
		item T
		aabb AABB2[U]
		seen uint32
	}

	// spatialHash2Bucket stores entry indexes for one active 2D grid cell.
	spatialHash2Bucket struct {
		generation uint32
		entries    []uint32
	}

	// spatialHash2Level stores the cells and occupancy policy for one 2D resolution.
	spatialHash2Level struct {
		shift             uint
		maxCellsPerObject int64
		lookup            map[uint64]uint32
		buckets           []spatialHash2Bucket
	}

	// SpatialHash2 indexes two-dimensional collidable objects across configurable grid levels.
	SpatialHash2[T any, U constraints.Float] struct {
		levels          []spatialHash2Level
		entries         []spatialHash2Entry[T, U]
		oversized       []uint32
		generation      uint32
		queryGeneration uint32
		activeBuckets   []int
		entryReferences []int64
		getAABB         func(T) *AABB2[U]
	}

	// spatialHash3Cell is a collision-free comparable key for a 3D grid cell.
	spatialHash3Cell struct {
		X, Y, Z int
	}

	// spatialHash3Entry is the canonical representation of an item for one spatial-hash build.
	spatialHash3Entry[T any, U constraints.Float] struct {
		item T
		aabb AABB3[U]
		seen uint32
	}

	// spatialHash3Bucket stores entry indexes for one active 3D grid cell.
	spatialHash3Bucket struct {
		generation uint32
		entries    []uint32
	}

	// spatialHash3Level stores the cells and occupancy policy for one 3D resolution.
	spatialHash3Level struct {
		shift             uint
		maxCellsPerObject int64
		lookup            map[spatialHash3Cell]uint32
		buckets           []spatialHash3Bucket
	}

	// SpatialHash3 indexes three-dimensional collidable objects across configurable grid levels.
	SpatialHash3[T any, U constraints.Float] struct {
		levels          []spatialHash3Level
		entries         []spatialHash3Entry[T, U]
		oversized       []uint32
		generation      uint32
		queryGeneration uint32
		activeBuckets   []int
		entryReferences []int64
		getAABB         func(T) *AABB3[U]
	}

	// SpatialHashValue represents a supported spatial-hash value.
	SpatialHashValue[T any, U constraints.Float] interface {
		SpatialHash2[T, U] | SpatialHash3[T, U]
	}

	// SpatialHashPtr represents the dimension-independent API shared by supported spatial hashes.
	SpatialHashPtr[T any, U constraints.Float, V SpatialHashValue[T, U]] interface {
		*V
		Config() SpatialHashConfig
		Clear()
		Insert(T)
		All() []T
		AllInto([]T) []T
	}
)
