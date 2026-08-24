package vector

import (
	"golang.org/x/exp/constraints"
)

func newVectorLerper[T constraints.Float, V VectorValue[T], P VectorPtr[T, V]](initialValue P) (vl *vectorLerper[T, V, P]) {
	vl = &vectorLerper[T, V, P]{
		previous: initialValue.Copy(),
		current:  initialValue.Copy(),
		progress: 0.0,
	}

	return
}

func (vl *vectorLerper[T, V, P]) Set(newValue P) {
	vl.value.CopyInto(vl.previous)
	newValue.CopyInto(vl.current)
	vl.progress = 0.0
}

func (vl *vectorLerper[T, V, P]) Update(t T) {
	vl.progress = min(1.0, vl.progress+t)
	vl.value.Lerp(vl.previous, vl.current, vl.progress)
}

func NewLerpScalar[T constraints.Float](initialValue T) (lvs *LerpScalar[T]) {
	var initialScalar *Scalar[T] = NewScalar(initialValue)

	lvs = &LerpScalar[T]{
		scalar:       initialScalar,
		vectorLerper: newVectorLerper(initialScalar),
	}

	lvs.value = lvs.scalar
	return
}

func (lvs *LerpScalar[T]) Set(newValue T) {
	lvs.previous.X = lvs.scalar.X
	lvs.current.X = newValue
	lvs.progress = 0.0
}

func NewLerpV2[T constraints.Float](initialValue *Vec2[T]) (lv2 *LerpV2[T]) {
	lv2 = &LerpV2[T]{
		vec2:         initialValue.Copy(),
		vectorLerper: newVectorLerper(initialValue),
	}

	lv2.value = lv2.vec2
	return
}

func NewLerpV3[T constraints.Float](initialValue *Vec3[T]) (lv3 *LerpV3[T]) {
	lv3 = &LerpV3[T]{
		vec3:         initialValue.Copy(),
		vectorLerper: newVectorLerper(initialValue),
	}

	lv3.value = lv3.vec3
	return
}
