package vector

import "golang.org/x/exp/constraints"

type (
	// Represents a generic vector type that can be either Vec2 or Vec3, constrained to float types.
	VectorValue[T constraints.Float] interface {
		Scalar[T] | Vec2[T] | Vec3[T]
	}

	VectorPtr[T constraints.Float, V VectorValue[T]] interface {
		*V
		Copy() *V
		CopyInto(*V) *V
		Lerp(*V, *V, T) *V
	}

	vectorLerper[T constraints.Float, V VectorValue[T], P VectorPtr[T, V]] struct {
		value, previous, current P
		progress                 T
	}

	// Represents a scalar value, which is a single float value. This can be used for 1D vectors or single values in calculations.
	Scalar[T constraints.Float] struct {
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

	scalar[T constraints.Float] = Scalar[T]
	vec2[T constraints.Float]   = Vec2[T]
	vec3[T constraints.Float]   = Vec3[T]

	LerpScalar[T constraints.Float] struct {
		*scalar[T]
		*vectorLerper[T, Scalar[T], *Scalar[T]]
	}

	LerpV2[T constraints.Float] struct {
		*vec2[T]
		*vectorLerper[T, Vec2[T], *Vec2[T]]
	}

	LerpV3[T constraints.Float] struct {
		*vec3[T]
		*vectorLerper[T, Vec3[T], *Vec3[T]]
	}
)
