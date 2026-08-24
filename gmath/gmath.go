package gmath

import (
	"cmp"
	"math"

	"golang.org/x/exp/constraints"
)

// Clamp clamps a value between a minimum and maximum value.
func Clamp[T cmp.Ordered](value, minValue, maxValue T) (clamped T) {
	clamped = min(maxValue, max(minValue, value))
	return
}

// AngleDelta returns the signed shortest angular distance from a to b.
func AngleDelta[T constraints.Float](a, b T) (result T) {
	result = T(math.Remainder(float64(b-a), 2*math.Pi))
	return
}
