package vector

import (
	"golang.org/x/exp/constraints"
)

// newVectorLerper creates a new vector lerper with the given initial value. It initializes
// the previous and current values to the initial value and sets the progress to 0.0.
func newVectorLerper[T constraints.Float, V VectorValue[T], P VectorPtr[T, V]](initialValue P) (vl *vectorLerper[T, V, P]) {
	vl = &vectorLerper[T, V, P]{
		previous: initialValue.Copy(),
		current:  initialValue.Copy(),
		progress: 0.0,
	}

	return
}

// Set updates the vector lerper with a new value. It copies the current value into the previous value,
// copies the new value into the current value, and resets the progress to 0.0.
func (vl *vectorLerper[T, V, P]) Set(newValue P) {
	vl.value.CopyInto(vl.previous)
	newValue.CopyInto(vl.current)
	vl.progress = 0.0
}

// Update progresses the interpolation of the vector lerper by a given time step t. It increments the progress
// by t, clamping it to a maximum of 1.0, and then performs the interpolation between the previous and current values
// based on the updated progress. The result is stored in the value field of the vector lerper.
func (vl *vectorLerper[T, V, P]) Update(t T) {
	vl.progress = min(1.0, vl.progress+t)
	vl.value.Lerp(vl.previous, vl.current, vl.progress)
}

// NewLerpScalar creates a new LerpScalar instance with the given initial value. It initializes the scalar field
// with a new Scalar instance and creates a new vector lerper for the scalar. The value field is set to the scalar.
func NewLerpScalar[T constraints.Float](initialValue T) (lvs *LerpScalar[T]) {
	var initialScalar *Scalar[T] = NewScalar(initialValue)

	lvs = &LerpScalar[T]{
		scalar:       initialScalar,
		vectorLerper: newVectorLerper(initialScalar),
	}

	lvs.value = lvs.scalar
	return
}

// Set updates the LerpScalar instance with a new value. It copies the current scalar value into the previous scalar,
// sets the current scalar value to the new value, and resets the progress to 0.0.
func (lvs *LerpScalar[T]) Set(newValue T) {
	lvs.previous.X = lvs.scalar.X
	lvs.current.X = newValue
	lvs.progress = 0.0
}

// NewLerpDirection creates a new LerpDirection instance with the given initial value. It initializes the direction field
// with a new Direction instance and creates a new vector lerper for the direction. The value field is set to the direction.
func NewLerpDirection[T constraints.Float](initialValue T) (lvd *LerpDirection[T]) {
	var initialDirection *Direction[T] = NewDirection(initialValue)

	lvd = &LerpDirection[T]{
		direction:    initialDirection,
		vectorLerper: newVectorLerper(initialDirection),
	}

	lvd.value = lvd.direction
	return
}

// Set updates the LerpDirection instance with a new value. It copies the current direction value into the previous direction,
// sets the current direction value to the new value, and resets the progress to 0.0.
func (lvd *LerpDirection[T]) Set(newValue T) {
	lvd.previous.X = lvd.direction.X
	lvd.current.X = newValue
	lvd.progress = 0.0
}

// NewLerpV2 creates a new LerpV2 instance with the given initial value. It initializes the vec2 field
// with a copy of the initial value and creates a new vector lerper for the vec2. The value field is set to the vec2.
func NewLerpV2[T constraints.Float](initialValue *Vec2[T]) (lv2 *LerpV2[T]) {
	lv2 = &LerpV2[T]{
		vec2:         initialValue.Copy(),
		vectorLerper: newVectorLerper(initialValue),
	}

	lv2.value = lv2.vec2
	return
}

// Set updates the LerpV2 instance with a new value. It copies the current vec2 value into the previous vec2,
// sets the current vec2 value to the new value, and resets the progress to 0.0.
func NewLerpV3[T constraints.Float](initialValue *Vec3[T]) (lv3 *LerpV3[T]) {
	lv3 = &LerpV3[T]{
		vec3:         initialValue.Copy(),
		vectorLerper: newVectorLerper(initialValue),
	}

	lv3.value = lv3.vec3
	return
}
