package hshg

import "golang.org/x/exp/constraints"

const (
	SHIFT_A = 10

	// Internal hash resolutions.
	fineShift   = SHIFT_A - 3 // 128 x 128
	mediumShift = SHIFT_A - 1 // 512 x 512
	coarseShift = SHIFT_A + 1 // 2048 x 2048

	hashLevels = 3
)

type (
	AABB[T constraints.Float] struct {
		X1, Y1, X2, Y2 T
	}

	Collidable[T constraints.Float] interface {
		GetAABB() *AABB[T]
	}

	// spatialHashEntry is the canonical representation of an item for the duration of one spatial-hash build. Buckets only
	// store indexes into SpatialHash.entries. This means a T is stored exactly once even if its AABB touches several cells.
	spatialHashEntry[T Collidable[U], U constraints.Float] struct {
		item T
		aabb AABB[U]

		// Query generation in which this entry was last visited. This avoids allocating a map[ID]struct{} for every Retrieve call.
		seen uint32
	}

	spatialHashBucket struct {
		// Frame/build generation in which this bucket is active.
		generation uint32

		// Indexes into SpatialHash.entries.
		entries []uint32
	}

	spatialHashLevel struct {
		shift uint

		// key -> bucket index + 1. Zero therefore means "not present". Using an integer index rather than *spatialHashBucket
		// avoids one heap allocation per cell and keeps bucket metadata contiguous.
		lookup map[uint64]uint32

		buckets []spatialHashBucket
	}

	SpatialHash[T Collidable[U], U constraints.Float] struct {
		levels [hashLevels]spatialHashLevel

		// Every inserted item occurs exactly once here.
		entries []spatialHashEntry[T, U]

		// Items which are so large that inserting them into even the coarse grid would be more expensive than simply testing them directly.
		oversized []uint32

		// Build/frame generation.
		generation uint32

		// Retrieve generation.
		queryGeneration uint32

		// Number of buckets actually used in the current/previous build. Used to determine when the historical cell lookup
		// maps have become  excessively bloated.
		activeBuckets [hashLevels]int
	}
)
