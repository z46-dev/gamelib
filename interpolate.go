package gamelib

import "golang.org/x/exp/constraints"

type InterpolatedValue[T constraints.Float] struct {
	Value, previous, current, progress T
}

func NewInterpolatedValue[T constraints.Float](initialValue T) (iv *InterpolatedValue[T]) {
	iv = &InterpolatedValue[T]{
		previous: initialValue,
		current:  initialValue,
		progress: 0.0,
	}

	return
}

func (iv *InterpolatedValue[T]) Set(newValue T) {
	iv.previous = iv.Value
	iv.current = newValue
	iv.progress = 0.0
}

func (iv *InterpolatedValue[T]) Update(t T) {
	iv.progress = min(1.0, iv.progress+t)
	iv.Value = Lerp(iv.previous, iv.current, iv.progress)
}
