package vector

import "golang.org/x/exp/constraints"

type (
	// Represents a generic vector-like value, which can be a scalar, direction, 2D vector, or 3D vector.
	// This interface is used to define the types that can be used in the vector lerping system.
	VectorValue[T constraints.Float] interface {
		Scalar[T] | Direction[T] | Vec2[T] | Vec3[T]
	}

	// Represents a pointer to a vector-like value, which can be a scalar, direction, 2D vector, or 3D vector.
	// This interface is used to define the types that can be used in the vector lerping system.
	VectorPtr[T constraints.Float, V VectorValue[T]] interface {
		*V
		Copy() *V
		CopyInto(*V) *V
		Lerp(*V, *V, T) *V
	}

	// Represents a generic vector lerper, which can be used to interpolate between two vector-like values over time.
	vectorLerper[T constraints.Float, V VectorValue[T], P VectorPtr[T, V]] struct {
		value, previous, current P
		progress                 T
	}

	// Represents a scalar value, which is a single float value. This can be used for 1D vectors or single values in calculations.
	Scalar[T constraints.Float] struct {
		X T
	}

	// Represents a direction vector, which is a single float value indicating direction. This can be used for directional calculations.
	// Its lerp method is different from the Scalar type's as it wraps around and accounts for the shortest path between two angles.
	Direction[T constraints.Float] struct {
		X T
	}

	// Represents a 2D vector with X and Y components.
	Vec2[T constraints.Float] struct {
		X, Y T
	}

	// Represents a 3D vector with X, Y, and Z components.
	Vec3[T constraints.Float] struct {
		X, Y, Z T
	}

	// Alias to simplify public APIs in structs
	scalar[T constraints.Float] = Scalar[T]

	// Alias to simplify public APIs in structs
	direction[T constraints.Float] = Direction[T]

	// Alias to simplify public APIs in structs
	vec2[T constraints.Float] = Vec2[T]

	// Alias to simplify public APIs in structs
	vec3[T constraints.Float] = Vec3[T]

	// Represents a lerper for scalar values, which can be used to interpolate between two scalar values over time.
	LerpScalar[T constraints.Float] struct {
		*scalar[T]
		*vectorLerper[T, Scalar[T], *Scalar[T]]
	}

	// Represents a lerper for direction values, which can be used to interpolate between two direction values over time.
	LerpDirection[T constraints.Float] struct {
		*direction[T]
		*vectorLerper[T, Direction[T], *Direction[T]]
	}

	// Represents a lerper for 2D vector values, which can be used to interpolate between two 2D vector values over time.
	LerpV2[T constraints.Float] struct {
		*vec2[T]
		*vectorLerper[T, Vec2[T], *Vec2[T]]
	}

	// Represents a lerper for 3D vector values, which can be used to interpolate between two 3D vector values over time.
	LerpV3[T constraints.Float] struct {
		*vec3[T]
		*vectorLerper[T, Vec3[T], *Vec3[T]]
	}
)
