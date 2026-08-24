package vector

import (
	"github.com/z46-dev/gamelib/gmath"
	"golang.org/x/exp/constraints"
)

// NewScalar creates a new scalar value with the given float value.
func NewScalar[T constraints.Float](x T) (scalar *Scalar[T]) {
	scalar = &Scalar[T]{
		X: x,
	}

	return
}

// Scalar_0 creates a new scalar value with the default float value of 0.
func Scalar_0[T constraints.Float]() (scalar *Scalar[T]) {
	scalar = &Scalar[T]{
		X: 0,
	}

	return
}

// Copy creates a copy of the scalar value and returns a pointer to the new scalar.
func (s *Scalar[T]) Copy() (copy *Scalar[T]) {
	copy = &Scalar[T]{
		X: s.X,
	}

	return
}

// CopyInto copies the scalar value into the provided newScalar pointer and returns a pointer to the original scalar.
func (s *Scalar[T]) CopyInto(newScalar *Scalar[T]) (self *Scalar[T]) {
	newScalar.X = s.X

	self = s
	return
}

// Lerp interpolates between the previous and current scalar values based on the given progress t,
// and updates the scalar value accordingly. It returns a pointer to the updated scalar.
func (s *Scalar[T]) Lerp(previous, current *Scalar[T], t T) (self *Scalar[T]) {
	s.X = gmath.Lerp(previous.X, current.X, t)

	self = s
	return
}
