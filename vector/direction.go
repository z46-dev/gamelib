package vector

import (
	"github.com/z46-dev/gamelib/gmath"
	"golang.org/x/exp/constraints"
)

// NewDirection creates a new direction value with the given float value.
func NewDirection[T constraints.Float](x T) (direction *Direction[T]) {
	direction = &Direction[T]{
		X: x,
	}

	return
}

// Direction_0 creates a new direction value with the default float value of 0.
func Direction_0[T constraints.Float]() (direction *Direction[T]) {
	direction = &Direction[T]{
		X: 0,
	}

	return
}

// Copy creates a copy of the direction value and returns a pointer to the new direction.
func (d *Direction[T]) Copy() (copy *Direction[T]) {
	copy = &Direction[T]{
		X: d.X,
	}

	return
}

// CopyInto copies the direction value into the provided newScalar pointer and returns a pointer to the original direction.
func (d *Direction[T]) CopyInto(newScalar *Direction[T]) (self *Direction[T]) {
	newScalar.X = d.X

	self = d
	return
}

// Lerp interpolates between the previous and current direction values based on the given progress t,
// and updates the direction value accordingly. It returns a pointer to the updated direction.
func (d *Direction[T]) Lerp(previous, current *Direction[T], t T) (self *Direction[T]) {
	d.X = gmath.LerpAngle(previous.X, current.X, t)

	self = d
	return
}
