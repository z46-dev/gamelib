package vector_test

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/z46-dev/gamelib/gmath"
	"github.com/z46-dev/gamelib/vector"
)

func assertVec2(t *testing.T, expectedX, expectedY float64, actual *vector.Vec2[float64]) {
	t.Helper()
	assert.InDelta(t, expectedX, actual.X, gmath.EPSILON)
	assert.InDelta(t, expectedY, actual.Y, gmath.EPSILON)
}

func assertVec3(t *testing.T, expectedX, expectedY, expectedZ float64, actual *vector.Vec3[float64]) {
	t.Helper()
	assert.InDelta(t, expectedX, actual.X, gmath.EPSILON)
	assert.InDelta(t, expectedY, actual.Y, gmath.EPSILON)
	assert.InDelta(t, expectedZ, actual.Z, gmath.EPSILON)
}

func assertAngle(t *testing.T, expected, actual float64) {
	t.Helper()
	assert.InDelta(t, 0, math.Remainder(actual-expected, gmath.TAU), gmath.EPSILON)
}

func TestLerpScalar(t *testing.T) {
	var lvs *vector.LerpScalar[float64] = vector.NewLerpScalar(0.0)
	assert.InDelta(t, 0.0, lvs.X, gmath.EPSILON)

	lvs.Set(10.0)
	lvs.Update(0.25)
	assert.InDelta(t, 2.5, lvs.X, gmath.EPSILON)
	lvs.Update(0.25)
	assert.InDelta(t, 5.0, lvs.X, gmath.EPSILON)
	lvs.Update(1.0)
	assert.InDelta(t, 10.0, lvs.X, gmath.EPSILON, "progress should be capped at one")

	lvs.Set(20.0)
	lvs.Update(0.5)
	assert.InDelta(t, 15.0, lvs.X, gmath.EPSILON, "a new interpolation should start at the displayed value")
}

func TestDirectionInterpolation(t *testing.T) {
	var (
		result *vector.Direction[float64]     = vector.Direction_0[float64]()
		lerper *vector.LerpDirection[float64] = vector.NewLerpDirection(350 * gmath.DEGREES_TO_RADIANS)
	)

	result.Lerp(vector.NewDirection(350*gmath.DEGREES_TO_RADIANS), vector.NewDirection(10*gmath.DEGREES_TO_RADIANS), 0.5)
	assertAngle(t, 0, result.X)

	result.Lerp(vector.NewDirection(10*gmath.DEGREES_TO_RADIANS), vector.NewDirection(350*gmath.DEGREES_TO_RADIANS), 0.5)
	assertAngle(t, 0, result.X)

	lerper.Set(10 * gmath.DEGREES_TO_RADIANS)
	lerper.Update(0.25)
	assertAngle(t, 355*gmath.DEGREES_TO_RADIANS, lerper.X)

	lerper.Set(90 * gmath.DEGREES_TO_RADIANS)
	lerper.Update(0.5)
	assertAngle(t, 42.5*gmath.DEGREES_TO_RADIANS, lerper.X)

	lerper.Update(1)
	assertAngle(t, 90*gmath.DEGREES_TO_RADIANS, lerper.X)
}

func TestVec2ConstructionAndCopy(t *testing.T) {
	var (
		v           *vector.Vec2[float64] = vector.NewVec2(3.0, 4.0)
		copy        *vector.Vec2[float64]
		destination *vector.Vec2[float64]
	)

	assertVec2(t, 3, 4, v)
	assertVec2(t, 0, 0, vector.Vec2_0[float64]())

	copy = v.Copy()
	assert.NotSame(t, v, copy)
	copy.X = 99
	assertVec2(t, 3, 4, v)

	destination = vector.Vec2_0[float64]()
	assert.Same(t, v, v.CopyInto(destination))
	assertVec2(t, 3, 4, destination)

	assertVec2(t, 0, 2, vector.Vec2FromAngleMagnitude(gmath.HPI, 2.0))
}

func TestVec2Measurements(t *testing.T) {
	var v *vector.Vec2[float64] = vector.NewVec2(3.0, 4.0)

	assert.InDelta(t, 5, v.Length(), gmath.EPSILON)
	assert.InDelta(t, 25, v.SquaredLength(), gmath.EPSILON)
	assert.InDelta(t, math.Atan2(4, 3), v.Direction(), gmath.EPSILON)
	assert.InDelta(t, 5, v.Dist(vector.NewVec2(0.0, 0.0)), gmath.EPSILON)
	assert.InDelta(t, 25, v.DistSquared(vector.NewVec2(0.0, 0.0)), gmath.EPSILON)
	assert.InDelta(t, gmath.HPI, vector.NewVec2(1.0, 1.0).AngleTo(vector.NewVec2(1.0, 2.0)), gmath.EPSILON)
	assert.InDelta(t, 11, v.Dot(vector.NewVec2(1.0, 2.0)), gmath.EPSILON)
}

func TestVec2MutatingAndNonMutatingOperations(t *testing.T) {
	var (
		v        *vector.Vec2[float64] = vector.NewVec2(1.0, 2.0)
		original *vector.Vec2[float64]
	)

	assert.Same(t, v, v.Add(vector.NewVec2(3.0, 4.0)).Sub(vector.NewVec2(1.0, 1.0)).Mul(2))
	assertVec2(t, 6, 10, v)

	v.MulVec(vector.NewVec2(0.5, -1.0))
	assertVec2(t, 3, -10, v)
	v.Swap()
	assertVec2(t, -10, 3, v)
	v.Zero()
	assertVec2(t, 0, 0, v)

	original = vector.NewVec2(2.0, 3.0)
	assertVec2(t, 3, 5, original.Added(vector.NewVec2(1.0, 2.0)))
	assertVec2(t, 1, 1, original.Subbed(vector.NewVec2(1.0, 2.0)))
	assertVec2(t, 4, 6, original.Mulled(2))
	assertVec2(t, 4, -3, original.MulledVec(vector.NewVec2(2.0, -1.0)))
	assertVec2(t, 3, 2, original.Swapped())
	assertVec2(t, 2, 3, original)
}

func TestVec2NormalizationRotationAndLerp(t *testing.T) {
	var original *vector.Vec2[float64] = vector.NewVec2(3.0, 4.0)

	assertVec2(t, 0.6, 0.8, original.Normalized())
	assertVec2(t, 3, 4, original)
	assertVec2(t, 0, 0, vector.Vec2_0[float64]().Normalize())
	assertVec2(t, 0, 1, vector.NewVec2(1.0, 0.0).Rotated(gmath.HPI))
	assertVec2(t, 1, 2, vector.NewVec2(2.0, 1.0).RotatedAround(gmath.HPI, vector.NewVec2(1.0, 1.0)))

	assertVec2(t, 2.5, 15, vector.Vec2_0[float64]().Lerp(vector.NewVec2(0.0, 10.0), vector.NewVec2(10.0, 30.0), 0.25))
}

func TestVec3ConstructionAndCopy(t *testing.T) {
	var (
		v           *vector.Vec3[float64] = vector.NewVec3(1.0, 2.0, 3.0)
		copy        *vector.Vec3[float64]
		destination *vector.Vec3[float64]
	)

	assertVec3(t, 1, 2, 3, v)
	assertVec3(t, 0, 0, 0, vector.Vec3_0[float64]())

	copy = v.Copy()
	assert.NotSame(t, v, copy)
	copy.Z = 99
	assertVec3(t, 1, 2, 3, v)

	destination = vector.Vec3_0[float64]()
	assert.Same(t, v, v.CopyInto(destination))
	assertVec3(t, 1, 2, 3, destination)
}

func TestVec3Measurements(t *testing.T) {
	var (
		v    *vector.Vec3[float64] = vector.NewVec3(2.0, 3.0, 6.0)
		zero *vector.Vec3[float64] = vector.Vec3_0[float64]()
	)

	assert.InDelta(t, 7, v.Length(), gmath.EPSILON)
	assert.InDelta(t, 49, v.SquaredLength(), gmath.EPSILON)
	assert.InDelta(t, 7, v.Dist(zero), gmath.EPSILON)
	assert.InDelta(t, 49, v.DistSquared(zero), gmath.EPSILON)
	assert.InDelta(t, 5, v.Dot(vector.NewVec3(1.0, 1.0, 0.0)), gmath.EPSILON)
	assert.InDelta(t, gmath.HPI, vector.NewVec3(1.0, 0.0, 0.0).AngleTo(vector.NewVec3(0.0, 1.0, 0.0)), gmath.EPSILON)
	assert.InDelta(t, 0, zero.AngleTo(v), gmath.EPSILON)
}

func TestVec3MutatingAndNonMutatingOperations(t *testing.T) {
	var (
		v        *vector.Vec3[float64] = vector.NewVec3(1.0, 2.0, 3.0)
		original *vector.Vec3[float64]
	)

	assert.Same(t, v, v.Add(vector.NewVec3(3.0, 4.0, 5.0)).Sub(vector.NewVec3(1.0, 1.0, 1.0)).Mul(2))
	assertVec3(t, 6, 10, 14, v)

	v.MulVec(vector.NewVec3(0.5, -1.0, 2.0))
	assertVec3(t, 3, -10, 28, v)
	v.Zero()
	assertVec3(t, 0, 0, 0, v)

	original = vector.NewVec3(2.0, 3.0, 4.0)
	assertVec3(t, 3, 5, 7, original.Added(vector.NewVec3(1.0, 2.0, 3.0)))
	assertVec3(t, 1, 1, 1, original.Subbed(vector.NewVec3(1.0, 2.0, 3.0)))
	assertVec3(t, 4, 6, 8, original.Mulled(2))
	assertVec3(t, 4, -3, 2, original.MulledVec(vector.NewVec3(2.0, -1.0, 0.5)))
	assertVec3(t, 2, 3, 4, original)
}

func TestVec3CrossProduct(t *testing.T) {
	var (
		x *vector.Vec3[float64] = vector.NewVec3(1.0, 0.0, 0.0)
		y *vector.Vec3[float64] = vector.NewVec3(0.0, 1.0, 0.0)
		z *vector.Vec3[float64] = x.Crossed(y)
	)

	assertVec3(t, 0, 0, 1, z)
	assertVec3(t, 1, 0, 0, x)
	assert.InDelta(t, 0, z.Dot(x), gmath.EPSILON)
	assert.InDelta(t, 0, z.Dot(y), gmath.EPSILON)
	assertVec3(t, 0, 0, -1, y.Crossed(x))

	assert.Same(t, x, x.Cross(y))
	assertVec3(t, 0, 0, 1, x)
}

func TestVec3NormalizationRotationAndLerp(t *testing.T) {
	var original *vector.Vec3[float64] = vector.NewVec3(0.0, 3.0, 4.0)

	assertVec3(t, 0, 0.6, 0.8, original.Normalized())
	assertVec3(t, 0, 3, 4, original)
	assertVec3(t, 0, 0, 0, vector.Vec3_0[float64]().Normalize())
	assertVec3(t, 0, 0, 1, vector.NewVec3(0.0, 1.0, 0.0).RotatedX(gmath.HPI))
	assertVec3(t, 0, 0, -1, vector.NewVec3(1.0, 0.0, 0.0).RotatedY(gmath.HPI))
	assertVec3(t, 0, 1, 0, vector.NewVec3(1.0, 0.0, 0.0).RotatedZ(gmath.HPI))
	assertVec3(t, 1, 2, 1, vector.NewVec3(2.0, 1.0, 1.0).RotatedAroundZ(gmath.HPI, vector.NewVec3(1.0, 1.0, 1.0)))

	assertVec3(t, 2.5, 15, -5, vector.Vec3_0[float64]().Lerp(vector.NewVec3(0.0, 10.0, -10.0), vector.NewVec3(10.0, 30.0, 10.0), 0.25))
}

func TestVectorLerpers(t *testing.T) {
	var (
		lv2 *vector.LerpV2[float64] = vector.NewLerpV2(vector.NewVec2(0.0, 10.0))
		lv3 *vector.LerpV3[float64]
	)

	lv2.Set(vector.NewVec2(10.0, 30.0))
	lv2.Update(0.25)
	assert.InDelta(t, 2.5, lv2.X, gmath.EPSILON)
	assert.InDelta(t, 15, lv2.Y, gmath.EPSILON)
	lv2.Update(1)
	assert.InDelta(t, 10, lv2.X, gmath.EPSILON)
	assert.InDelta(t, 30, lv2.Y, gmath.EPSILON)

	lv3 = vector.NewLerpV3(vector.NewVec3(0.0, 10.0, -10.0))
	lv3.Set(vector.NewVec3(10.0, 30.0, 10.0))
	lv3.Update(0.5)
	assert.InDelta(t, 5, lv3.X, gmath.EPSILON)
	assert.InDelta(t, 20, lv3.Y, gmath.EPSILON)
	assert.InDelta(t, 0, lv3.Z, gmath.EPSILON)
}
