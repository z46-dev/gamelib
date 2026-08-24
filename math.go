package gamelib

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

// AngleDelta returns the signed shortest angular distance from a to b.
func AngleDelta[T constraints.Float](a, b T) (result T) {
	result = T(math.Remainder(float64(b-a), 2*math.Pi))
	return
}

// func LerpAngle[T constraints.Float](a, b, t T) (value T) {
// 	var (
// 		a64, b64, t64 float64 = float64(a), float64(b), float64(t)
// 		it64          float64 = 1 - t64
// 	)

// 	value = T(math.Atan2((it64)*math.Sin(a64)+t64*math.Sin(b64), (it64)*math.Cos(a64)+t64*math.Cos(b64)))
// 	return
// }

// // AngleDelta returns the signed shortest angular distance from a to b.
// func AngleDelta[T constraints.Float](a, b T) (result T) {
// 	var (
// 		a64, b64 float64 = float64(a), float64(b)
// 		delta    float64 = b64 - a64
// 	)

// 	result = T(math.Atan2(math.Sin(delta), math.Cos(delta)))
// 	return
// }
