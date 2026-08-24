package gmath_test

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/z46-dev/gamelib/gmath"
)

func assertAngle(t *testing.T, expected, actual float64) {
	t.Helper()
	assert.InDelta(t, 0, math.Remainder(actual-expected, gmath.TAU), gmath.EPSILON)
}

func TestClampBoundaries(t *testing.T) {
	assert.Equal(t, 0, gmath.Clamp(-1, 0, 10))
	assert.Equal(t, 4, gmath.Clamp(4, 0, 10))
	assert.Equal(t, 10, gmath.Clamp(11, 0, 10))
	assert.InDelta(t, 0.25, gmath.Clamp(0.25, 0.0, 1.0), gmath.EPSILON)
	assert.Equal(t, "m", gmath.Clamp("z", "a", "m"))
}

func TestLinearInterpolation(t *testing.T) {
	assert.InDelta(t, 10, gmath.Lerp(10.0, 20.0, 0.0), gmath.EPSILON)
	assert.InDelta(t, 12.5, gmath.Lerp(10.0, 20.0, 0.25), gmath.EPSILON)
	assert.InDelta(t, 20, gmath.Lerp(10.0, 20.0, 1.0), gmath.EPSILON)
	assert.InDelta(t, 25, gmath.Lerp(10.0, 20.0, 1.5), gmath.EPSILON, "Lerp should support extrapolation")
	assert.InDelta(t, 15, gmath.Lerp(20.0, 10.0, 0.5), gmath.EPSILON)
}

func TestAngularInterpolationUsesShortestPath(t *testing.T) {

	assert.InDelta(t, 20*gmath.DEGREES_TO_RADIANS, gmath.AngleDelta(350*gmath.DEGREES_TO_RADIANS, 10*gmath.DEGREES_TO_RADIANS), gmath.EPSILON)
	assert.InDelta(t, -20*gmath.DEGREES_TO_RADIANS, gmath.AngleDelta(10*gmath.DEGREES_TO_RADIANS, 350*gmath.DEGREES_TO_RADIANS), gmath.EPSILON)
	assert.InDelta(t, 0, gmath.AngleDelta(45*gmath.DEGREES_TO_RADIANS, 45*gmath.DEGREES_TO_RADIANS), gmath.EPSILON)

	assertAngle(t, 355*gmath.DEGREES_TO_RADIANS, gmath.LerpAngle(350*gmath.DEGREES_TO_RADIANS, 10*gmath.DEGREES_TO_RADIANS, 0.25))
	assertAngle(t, 0, gmath.LerpAngle(350*gmath.DEGREES_TO_RADIANS, 10*gmath.DEGREES_TO_RADIANS, 0.5))
	assertAngle(t, 5*gmath.DEGREES_TO_RADIANS, gmath.LerpAngle(10*gmath.DEGREES_TO_RADIANS, 350*gmath.DEGREES_TO_RADIANS, 0.25))
	assertAngle(t, 10*gmath.DEGREES_TO_RADIANS, gmath.LerpAngle(350*gmath.DEGREES_TO_RADIANS, 10*gmath.DEGREES_TO_RADIANS, 1.0))
}
