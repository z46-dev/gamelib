package gamelib

import "math"

// Represents a 2D vector with X and Y components.
type Vec2 struct {
	X, Y float64
}

// NewVec2 creates a new Vec2 with the given X and Y values.
func NewVec2(x, y float64) (vec *Vec2) {
	vec = &Vec2{
		X: x,
		Y: y,
	}

	return
}

// Vec2_0 creates a new Vec2 with both X and Y set to 0.
func Vec2_0() (vec *Vec2) {
	vec = &Vec2{
		X: 0,
		Y: 0,
	}

	return
}

// Copy creates a new Vec2 that is a copy of the original.
func (v *Vec2) Copy() (copy *Vec2) {
	copy = &Vec2{
		X: v.X,
		Y: v.Y,
	}

	return
}

// Length calculates and returns the length (magnitude) of the Vec2.
func (v *Vec2) Length() (length float64) {
	length = math.Sqrt(v.X*v.X + v.Y*v.Y)
	return
}

// Direction calculates and returns the direction (angle in radians) of the Vec2 from the positive X-axis.
func (v *Vec2) Direction() (dir float64) {
	dir = math.Atan2(v.Y, v.X)
	return
}

// Chainable self-modifying methods \\

// Normalize normalizes the Vec2 to have a length of 1 while maintaining its direction.
func (v *Vec2) Normalize() (self *Vec2) {
	var length float64 = v.Length()
	if length != 0 {
		v.X /= length
		v.Y /= length
	}

	self = v
	return
}

// Zero sets both X and Y components of the Vec2 to 0.
func (v *Vec2) Zero() (self *Vec2) {
	v.X = 0
	v.Y = 0

	self = v
	return
}

// Swap swaps the X and Y components of the Vec2.
func (v *Vec2) Swap() (self *Vec2) {
	v.X, v.Y = v.Y, v.X

	self = v
	return
}

// Add adds another Vec2 to the current Vec2.
func (v *Vec2) Add(other *Vec2) (self *Vec2) {
	v.X += other.X
	v.Y += other.Y

	self = v
	return
}

// Sub subtracts another Vec2 from the current Vec2.
func (v *Vec2) Sub(other *Vec2) (self *Vec2) {
	v.X -= other.X
	v.Y -= other.Y

	self = v
	return
}

// Mul multiplies the Vec2 by a scalar value.
func (v *Vec2) Mul(scalar float64) (self *Vec2) {
	v.X *= scalar
	v.Y *= scalar

	self = v
	return
}

// MulVec multiplies the Vec2 by another Vec2 component-wise.
func (v *Vec2) MulVec(other *Vec2) (self *Vec2) {
	v.X *= other.X
	v.Y *= other.Y

	self = v
	return
}

// Rotate rotates the Vec2 by a given angle in radians.
func (v *Vec2) Rotate(angle float64) (self *Vec2) {
	var cos, sin float64 = math.Cos(angle), math.Sin(angle)
	v.X, v.Y = v.X*cos-v.Y*sin, v.X*sin+v.Y*cos

	self = v
	return
}

// RotateAround rotates the Vec2 around a pivot point by a given angle in radians.
func (v *Vec2) RotateAround(angle float64, pivot *Vec2) (self *Vec2) {
	v.Sub(pivot).Rotate(angle).Add(pivot)

	self = v
	return
}

// Lerp performs linear interpolation between the current Vec2 and a target Vec2 based on the parameter t (0 <= t <= 1).
func (v *Vec2) Lerp(target *Vec2, t float64) (self *Vec2) {
	v.X = Lerp(v.X, target.X, t)
	v.Y = Lerp(v.Y, target.Y, t)

	self = v
	return
}

// Methods that return new Vec2s \\

// Normalized returns a new Vec2 that is the normalized version of the original.
func (v *Vec2) Normalized() (normalized *Vec2) {
	normalized = v.Copy().Normalize()
	return
}

// Swapped returns a new Vec2 with the X and Y components swapped.
func (v *Vec2) Swapped() (swapped *Vec2) {
	swapped = v.Copy().Swap()
	return
}

// Added returns a new Vec2 that is the sum of the original and another Vec2.
func (v *Vec2) Added(other *Vec2) (added *Vec2) {
	added = v.Copy().Add(other)
	return
}

// Subbed returns a new Vec2 that is the difference between the original and another Vec2.
func (v *Vec2) Subbed(other *Vec2) (subbed *Vec2) {
	subbed = v.Copy().Sub(other)
	return
}

// Mulled returns a new Vec2 that is the original multiplied by a scalar value.
func (v *Vec2) Mulled(scalar float64) (mulled *Vec2) {
	mulled = v.Copy().Mul(scalar)
	return
}

// MulledVec returns a new Vec2 that is the original multiplied by another Vec2 component-wise.
func (v *Vec2) MulledVec(other *Vec2) (mulled *Vec2) {
	mulled = v.Copy().MulVec(other)
	return
}

// Rotated returns a new Vec2 that is the original rotated by a given angle in radians.
func (v *Vec2) Rotated(angle float64) (rotated *Vec2) {
	rotated = v.Copy().Rotate(angle)
	return
}

// RotatedAround returns a new Vec2 that is the original rotated around a pivot point by a given angle in radians.
func (v *Vec2) RotatedAround(angle float64, pivot *Vec2) (rotated *Vec2) {
	rotated = v.Copy().RotateAround(angle, pivot)
	return
}

// Lerped returns a new Vec2 that is the result of linear interpolation between the original and a target Vec2 based on the parameter t (0 <= t <= 1).
func (v *Vec2) Lerped(target *Vec2, t float64) (lerped *Vec2) {
	lerped = v.Copy().Lerp(target, t)
	return
}

// Other Methods \\

// Dist calculates and returns the distance between the current Vec2 and another Vec2.
func (v *Vec2) Dist(other *Vec2) (dist float64) {
	dist = math.Sqrt((v.X-other.X)*(v.X-other.X) + (v.Y-other.Y)*(v.Y-other.Y))
	return
}

// DistSquared calculates and returns the squared distance between the current Vec2 and another Vec2.
// This is more efficient than Dist when you only need to compare distances, as it avoids the costly square root operation.
func (v *Vec2) DistSquared(other *Vec2) (distSquared float64) {
	distSquared = (v.X-other.X)*(v.X-other.X) + (v.Y-other.Y)*(v.Y-other.Y)
	return
}

// AngleTo calculates and returns the angle in radians from the current Vec2 to another Vec2.
func (v *Vec2) AngleTo(other *Vec2) (angle float64) {
	angle = math.Atan2(other.Y-v.Y, other.X-v.X)
	return
}

func (v *Vec2) DirectionMagnitudeInto(direction, magnitude float64) (self *Vec2) {
	v.X += math.Cos(direction) * magnitude
	v.Y += math.Sin(direction) * magnitude

	self = v
	return
}

func Vec2FromAngleMagnitude(direction, magnitude float64) (vec *Vec2) {
	vec = &Vec2{
		X: math.Cos(direction) * magnitude,
		Y: math.Sin(direction) * magnitude,
	}

	return
}
