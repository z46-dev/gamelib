package vector

import (
	"github.com/z46-dev/gamelib/gmath"
	"golang.org/x/exp/constraints"
)

func NewScalar[T constraints.Float](x T) (scalar *Scalar[T]) {
	scalar = &Scalar[T]{
		X: x,
	}

	return
}

func Scalar_0[T constraints.Float]() (scalar *Scalar[T]) {
	scalar = &Scalar[T]{
		X: 0,
	}

	return
}

func (s *Scalar[T]) Copy() (copy *Scalar[T]) {
	copy = &Scalar[T]{
		X: s.X,
	}

	return
}

func (s *Scalar[T]) CopyInto(newScalar *Scalar[T]) (self *Scalar[T]) {
	newScalar.X = s.X

	self = s
	return
}

func (s *Scalar[T]) Lerp(previous, current *Scalar[T], t T) (self *Scalar[T]) {
	s.X = gmath.Lerp(previous.X, current.X, t)

	self = s
	return
}
