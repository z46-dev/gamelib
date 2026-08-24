package gmath

import (
	"golang.org/x/exp/constraints"
)

// Lerp performs linear interpolation between two values a and b, based on the parameter t.
func Lerp[T constraints.Float](a, b, t T) (result T) {
	result = a + (b-a)*t
	return
}

// LerpAngle performs linear interpolation between two angles a and b, based on the parameter t, taking into account the circular nature of angles.
func LerpAngle[T constraints.Float](a, b, t T) (result T) {
	result = a + AngleDelta(a, b)*t
	return
}
