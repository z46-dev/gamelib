package gamelib

import "math"

// Lerp performs linear interpolation between two float64 values a and b based on the parameter t (0 <= t <= 1).
func Lerp(a, b, t float64) (result float64) {
	result = a + (b-a)*t
	return
}

// Lerp32 performs linear interpolation between two float32 values a and b based on the parameter t (0 <= t <= 1).
func Lerp32(a, b, t float32) (result float32) {
	result = a + (b-a)*t
	return
}

// LerpAngle performs linear interpolation between two angles a and b (in radians) based on the parameter t (0 <= t <= 1).
// It correctly handles the wrap-around at 2*Pi to ensure smooth interpolation between angles.
func LerpAngle(a, b, t float64) (result float64) {
	result = math.Atan2((1-t)*math.Sin(a)+t*math.Sin(b), (1-t)*math.Cos(a)+t*math.Cos(b))
	return
}

// LerpAngle32 performs linear interpolation between two angles a and b (in radians) based on the parameter t (0 <= t <= 1).
// It correctly handles the wrap-around at 2*Pi to ensure smooth interpolation between angles.
func LerpAngle32(a, b, t float32) (result float32) {
	var a64, b64, t64 float64 = float64(a), float64(b), float64(t)
	result = float32(math.Atan2((1-t64)*math.Sin(a64)+t64*math.Sin(b64), (1-t64)*math.Cos(a64)+t64*math.Cos(b64)))
	return
}

// AngleDelta returns the signed shortest angular distance from a to b.
func AngleDelta(a, b float64) (result float64) {
	result = math.Atan2(math.Sin(b-a), math.Cos(b-a))
	return
}
