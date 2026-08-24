package vector

import (
	"math"

	"github.com/z46-dev/gamelib/gmath"
	"golang.org/x/exp/constraints"
)

// NewVec2 creates a new Vec2 with the given X and Y values.
func NewVec2[T constraints.Float](x, y T) (vec *Vec2[T]) {
	vec = &Vec2[T]{
		X: x,
		Y: y,
	}

	return
}

// Vec2_0 creates a new Vec2 with both X and Y set to 0.
func Vec2_0[T constraints.Float]() (vec *Vec2[T]) {
	vec = &Vec2[T]{
		X: 0,
		Y: 0,
	}

	return
}

// Vec2FromAngleMagnitude creates a new Vec2 from a given direction (angle in radians) and magnitude.
func Vec2FromAngleMagnitude[T constraints.Float](direction, magnitude T) (vec *Vec2[T]) {
	var direction64 float64 = float64(direction)
	vec = &Vec2[T]{
		X: T(math.Cos(direction64)) * magnitude,
		Y: T(math.Sin(direction64)) * magnitude,
	}

	return
}

// Copy creates a new Vec2 that is a copy of the original.
func (v *Vec2[T]) Copy() (copy *Vec2[T]) {
	copy = &Vec2[T]{
		X: v.X,
		Y: v.Y,
	}

	return
}

// CopyInto copies the values of the current Vec2 into another Vec2, returning the current Vec2. Useful for avoiding extra allocations.
func (v *Vec2[T]) CopyInto(newVec *Vec2[T]) (self *Vec2[T]) {
	newVec.X, newVec.Y = v.X, v.Y

	self = v
	return
}

// Length calculates and returns the length (magnitude) of the Vec2.
func (v *Vec2[T]) Length() (length T) {
	length = T(math.Sqrt(float64(v.X*v.X + v.Y*v.Y)))
	return
}

// SquaredLength calculates and returns the squared length of the Vec2.
// This is more efficient than Length when you only need to compare lengths, as it avoids the costly square root operation.
func (v *Vec2[T]) SquaredLength() (squaredLength T) {
	squaredLength = v.X*v.X + v.Y*v.Y
	return
}

// Direction calculates and returns the direction (angle in radians) of the Vec2 from the positive X-axis.
func (v *Vec2[T]) Direction() (dir T) {
	dir = T(math.Atan2(float64(v.Y), float64(v.X)))
	return
}

// Chainable self-modifying methods \\

// Normalize normalizes the Vec2 to have a length of 1 while maintaining its direction.
func (v *Vec2[T]) Normalize() (self *Vec2[T]) {
	var length T = v.Length()
	if length != 0 {
		v.X /= length
		v.Y /= length
	}

	self = v
	return
}

// Zero sets both X and Y components of the Vec2 to 0.
func (v *Vec2[T]) Zero() (self *Vec2[T]) {
	v.X, v.Y = 0, 0

	self = v
	return
}

// Swap swaps the X and Y components of the Vec2.
func (v *Vec2[T]) Swap() (self *Vec2[T]) {
	v.X, v.Y = v.Y, v.X

	self = v
	return
}

// Add adds another Vec2 to the current Vec2.
func (v *Vec2[T]) Add(other *Vec2[T]) (self *Vec2[T]) {
	v.X += other.X
	v.Y += other.Y

	self = v
	return
}

// Sub subtracts another Vec2 from the current Vec2.
func (v *Vec2[T]) Sub(other *Vec2[T]) (self *Vec2[T]) {
	v.X -= other.X
	v.Y -= other.Y

	self = v
	return
}

// Mul multiplies the Vec2 by a scalar value.
func (v *Vec2[T]) Mul(scalar T) (self *Vec2[T]) {
	v.X *= scalar
	v.Y *= scalar

	self = v
	return
}

// MulVec multiplies the Vec2 by another Vec2 component-wise.
func (v *Vec2[T]) MulVec(other *Vec2[T]) (self *Vec2[T]) {
	v.X *= other.X
	v.Y *= other.Y

	self = v
	return
}

// Dot calculates the dot product of the current Vec2 and another Vec2.
func (v *Vec2[T]) Dot(other *Vec2[T]) (dot T) {
	dot = v.X*other.X + v.Y*other.Y
	return
}

// Rotate rotates the Vec2 by a given angle in radians.
func (v *Vec2[T]) Rotate(angle T) (self *Vec2[T]) {
	var (
		angle64  float64 = float64(angle)
		cos, sin T       = T(math.Cos(angle64)), T(math.Sin(angle64))
	)

	v.X, v.Y = v.X*cos-v.Y*sin, v.X*sin+v.Y*cos

	self = v
	return
}

// RotateAround rotates the Vec2 around a pivot point by a given angle in radians.
func (v *Vec2[T]) RotateAround(angle T, pivot *Vec2[T]) (self *Vec2[T]) {
	v.Sub(pivot).Rotate(angle).Add(pivot)

	self = v
	return
}

// Lerp performs linear interpolation between the current Vec2 and a target Vec2 based on the parameter t (0 <= t <= 1).
func (v *Vec2[T]) Lerp(previous, current *Vec2[T], t T) (self *Vec2[T]) {
	v.X, v.Y = gmath.Lerp(previous.X, current.X, t), gmath.Lerp(previous.Y, current.Y, t)

	self = v
	return
}

// Methods that return new Vec2s \\

// Normalized returns a new Vec2 that is the normalized version of the original.
func (v *Vec2[T]) Normalized() (normalized *Vec2[T]) {
	normalized = v.Copy().Normalize()
	return
}

// Swapped returns a new Vec2 with the X and Y components swapped.
func (v *Vec2[T]) Swapped() (swapped *Vec2[T]) {
	swapped = v.Copy().Swap()
	return
}

// Added returns a new Vec2 that is the sum of the original and another Vec2.
func (v *Vec2[T]) Added(other *Vec2[T]) (added *Vec2[T]) {
	added = v.Copy().Add(other)
	return
}

// Subbed returns a new Vec2 that is the difference between the original and another Vec2.
func (v *Vec2[T]) Subbed(other *Vec2[T]) (subbed *Vec2[T]) {
	subbed = v.Copy().Sub(other)
	return
}

// Mulled returns a new Vec2 that is the original multiplied by a scalar value.
func (v *Vec2[T]) Mulled(scalar T) (mulled *Vec2[T]) {
	mulled = v.Copy().Mul(scalar)
	return
}

// MulledVec returns a new Vec2 that is the original multiplied by another Vec2 component-wise.
func (v *Vec2[T]) MulledVec(other *Vec2[T]) (mulled *Vec2[T]) {
	mulled = v.Copy().MulVec(other)
	return
}

// Rotated returns a new Vec2 that is the original rotated by a given angle in radians.
func (v *Vec2[T]) Rotated(angle T) (rotated *Vec2[T]) {
	rotated = v.Copy().Rotate(angle)
	return
}

// RotatedAround returns a new Vec2 that is the original rotated around a pivot point by a given angle in radians.
func (v *Vec2[T]) RotatedAround(angle T, pivot *Vec2[T]) (rotated *Vec2[T]) {
	rotated = v.Copy().RotateAround(angle, pivot)
	return
}

// Lerped returns a new Vec2 that is the result of linear interpolation between the original and a target Vec2 based on the parameter t (0 <= t <= 1).
func (v *Vec2[T]) Lerped(previous, current *Vec2[T], t T) (lerped *Vec2[T]) {
	lerped = v.Copy().Lerp(previous, current, t)
	return
}

// Other Methods \\

// Dist calculates and returns the distance between the current Vec2 and another Vec2.
func (v *Vec2[T]) Dist(other *Vec2[T]) (dist T) {
	var dx, dy T = v.X - other.X, v.Y - other.Y
	dist = T(math.Sqrt(float64(dx*dx + dy*dy)))
	return
}

// DistSquared calculates and returns the squared distance between the current Vec2 and another Vec2.
// This is more efficient than Dist when you only need to compare distances, as it avoids the costly square root operation.
func (v *Vec2[T]) DistSquared(other *Vec2[T]) (distSquared T) {
	var dx, dy T = v.X - other.X, v.Y - other.Y
	distSquared = dx*dx + dy*dy
	return
}

// AngleTo calculates and returns the angle in radians from the current Vec2 to another Vec2.
func (v *Vec2[T]) AngleTo(other *Vec2[T]) (angle T) {
	angle = T(math.Atan2(float64(other.Y-v.Y), float64(other.X-v.X)))
	return
}

func (v *Vec2[T]) DirectionMagnitudeInto(direction, magnitude T) (self *Vec2[T]) {
	var direction64 float64 = float64(direction)
	v.X += T(math.Cos(direction64)) * magnitude
	v.Y += T(math.Sin(direction64)) * magnitude

	self = v
	return
}
