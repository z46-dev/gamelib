package vector

import (
	"math"

	"github.com/z46-dev/gamelib/gmath"
	"golang.org/x/exp/constraints"
)

// NewVec3 creates a new Vec3 with the given X, Y, and Z values.
func NewVec3[T constraints.Float](x, y, z T) (vec *Vec3[T]) {
	vec = &Vec3[T]{
		X: x,
		Y: y,
		Z: z,
	}

	return
}

// Vec3_0 creates a new Vec3 with X, Y, and Z set to 0.
func Vec3_0[T constraints.Float]() (vec *Vec3[T]) {
	vec = &Vec3[T]{
		X: 0,
		Y: 0,
		Z: 0,
	}

	return
}

// Copy creates a new Vec3 that is a copy of the original.
func (v *Vec3[T]) Copy() (copy *Vec3[T]) {
	copy = &Vec3[T]{
		X: v.X,
		Y: v.Y,
		Z: v.Z,
	}

	return
}

// CopyInto copies the values of the current Vec3 into another Vec3, returning the current Vec3. Useful for avoiding extra allocations.
func (v *Vec3[T]) CopyInto(newVec *Vec3[T]) (self *Vec3[T]) {
	newVec.X, newVec.Y, newVec.Z = v.X, v.Y, v.Z

	self = v
	return
}

// Length calculates and returns the length (magnitude) of the Vec3.
func (v *Vec3[T]) Length() (length T) {
	length = T(math.Sqrt(float64(v.X*v.X + v.Y*v.Y + v.Z*v.Z)))
	return
}

// SquaredLength calculates and returns the squared length of the Vec3.
// This is more efficient than Length when you only need to compare lengths, as it avoids the costly square root operation.
func (v *Vec3[T]) SquaredLength() (squaredLength T) {
	squaredLength = v.X*v.X + v.Y*v.Y + v.Z*v.Z
	return
}

// Chainable self-modifying methods \\

// Normalize normalizes the Vec3 to have a length of 1 while maintaining its direction.
func (v *Vec3[T]) Normalize() (self *Vec3[T]) {
	var length T = v.Length()
	if length != 0 {
		v.X /= length
		v.Y /= length
		v.Z /= length
	}

	self = v
	return
}

// Zero sets the X, Y, and Z components of the Vec3 to 0.
func (v *Vec3[T]) Zero() (self *Vec3[T]) {
	v.X, v.Y, v.Z = 0, 0, 0

	self = v
	return
}

// Add adds another Vec3 to the current Vec3.
func (v *Vec3[T]) Add(other *Vec3[T]) (self *Vec3[T]) {
	v.X += other.X
	v.Y += other.Y
	v.Z += other.Z

	self = v
	return
}

// Sub subtracts another Vec3 from the current Vec3.
func (v *Vec3[T]) Sub(other *Vec3[T]) (self *Vec3[T]) {
	v.X -= other.X
	v.Y -= other.Y
	v.Z -= other.Z

	self = v
	return
}

// Mul multiplies the Vec3 by a scalar value.
func (v *Vec3[T]) Mul(scalar T) (self *Vec3[T]) {
	v.X *= scalar
	v.Y *= scalar
	v.Z *= scalar

	self = v
	return
}

// MulVec multiplies the Vec3 by another Vec3 component-wise.
func (v *Vec3[T]) MulVec(other *Vec3[T]) (self *Vec3[T]) {
	v.X *= other.X
	v.Y *= other.Y
	v.Z *= other.Z

	self = v
	return
}

// Dot calculates the dot product of the current Vec3 and another Vec3.
func (v *Vec3[T]) Dot(other *Vec3[T]) (dot T) {
	dot = v.X*other.X + v.Y*other.Y + v.Z*other.Z
	return
}

// Cross replaces the Vec3 with its cross product with another Vec3.
func (v *Vec3[T]) Cross(other *Vec3[T]) (self *Vec3[T]) {
	v.X, v.Y, v.Z = v.Y*other.Z-v.Z*other.Y, v.Z*other.X-v.X*other.Z, v.X*other.Y-v.Y*other.X

	self = v
	return
}

// RotateX rotates the Vec3 around the X-axis by a given angle in radians.
func (v *Vec3[T]) RotateX(angle T) (self *Vec3[T]) {
	var (
		angle64  float64 = float64(angle)
		cos, sin T       = T(math.Cos(angle64)), T(math.Sin(angle64))
	)

	v.Y, v.Z = v.Y*cos-v.Z*sin, v.Y*sin+v.Z*cos

	self = v
	return
}

// RotateY rotates the Vec3 around the Y-axis by a given angle in radians.
func (v *Vec3[T]) RotateY(angle T) (self *Vec3[T]) {
	var (
		angle64  float64 = float64(angle)
		cos, sin T       = T(math.Cos(angle64)), T(math.Sin(angle64))
	)

	v.X, v.Z = v.X*cos+v.Z*sin, -v.X*sin+v.Z*cos

	self = v
	return
}

// RotateZ rotates the Vec3 around the Z-axis by a given angle in radians.
func (v *Vec3[T]) RotateZ(angle T) (self *Vec3[T]) {
	var (
		angle64  float64 = float64(angle)
		cos, sin T       = T(math.Cos(angle64)), T(math.Sin(angle64))
	)

	v.X, v.Y = v.X*cos-v.Y*sin, v.X*sin+v.Y*cos

	self = v
	return
}

// RotateAroundX rotates the Vec3 around the X-axis passing through a pivot point.
func (v *Vec3[T]) RotateAroundX(angle T, pivot *Vec3[T]) (self *Vec3[T]) {
	v.Sub(pivot).RotateX(angle).Add(pivot)

	self = v
	return
}

// RotateAroundY rotates the Vec3 around the Y-axis passing through a pivot point.
func (v *Vec3[T]) RotateAroundY(angle T, pivot *Vec3[T]) (self *Vec3[T]) {
	v.Sub(pivot).RotateY(angle).Add(pivot)

	self = v
	return
}

// RotateAroundZ rotates the Vec3 around the Z-axis passing through a pivot point.
func (v *Vec3[T]) RotateAroundZ(angle T, pivot *Vec3[T]) (self *Vec3[T]) {
	v.Sub(pivot).RotateZ(angle).Add(pivot)

	self = v
	return
}

// Lerp performs linear interpolation between the current Vec3 and a target Vec3 based on the parameter t (0 <= t <= 1).
func (v *Vec3[T]) Lerp(previous, current *Vec3[T], t T) (self *Vec3[T]) {
	v.X, v.Y, v.Z = gmath.Lerp(previous.X, current.X, t), gmath.Lerp(previous.Y, current.Y, t), gmath.Lerp(previous.Z, current.Z, t)

	self = v
	return
}

// Methods that return new Vec3s \\

// Normalized returns a new Vec3 that is the normalized version of the original.
func (v *Vec3[T]) Normalized() (normalized *Vec3[T]) {
	normalized = v.Copy().Normalize()
	return
}

// Added returns a new Vec3 that is the sum of the original and another Vec3.
func (v *Vec3[T]) Added(other *Vec3[T]) (added *Vec3[T]) {
	added = v.Copy().Add(other)
	return
}

// Subbed returns a new Vec3 that is the difference between the original and another Vec3.
func (v *Vec3[T]) Subbed(other *Vec3[T]) (subbed *Vec3[T]) {
	subbed = v.Copy().Sub(other)
	return
}

// Mulled returns a new Vec3 that is the original multiplied by a scalar value.
func (v *Vec3[T]) Mulled(scalar T) (mulled *Vec3[T]) {
	mulled = v.Copy().Mul(scalar)
	return
}

// MulledVec returns a new Vec3 that is the original multiplied by another Vec3 component-wise.
func (v *Vec3[T]) MulledVec(other *Vec3[T]) (mulled *Vec3[T]) {
	mulled = v.Copy().MulVec(other)
	return
}

// Crossed returns a new Vec3 containing the cross product of the original and another Vec3.
func (v *Vec3[T]) Crossed(other *Vec3[T]) (crossed *Vec3[T]) {
	crossed = v.Copy().Cross(other)
	return
}

// RotatedX returns a new Vec3 that is the original rotated around the X-axis.
func (v *Vec3[T]) RotatedX(angle T) (rotated *Vec3[T]) {
	rotated = v.Copy().RotateX(angle)
	return
}

// RotatedY returns a new Vec3 that is the original rotated around the Y-axis.
func (v *Vec3[T]) RotatedY(angle T) (rotated *Vec3[T]) {
	rotated = v.Copy().RotateY(angle)
	return
}

// RotatedZ returns a new Vec3 that is the original rotated around the Z-axis.
func (v *Vec3[T]) RotatedZ(angle T) (rotated *Vec3[T]) {
	rotated = v.Copy().RotateZ(angle)
	return
}

// RotatedAroundX returns a new Vec3 rotated around the X-axis passing through a pivot point.
func (v *Vec3[T]) RotatedAroundX(angle T, pivot *Vec3[T]) (rotated *Vec3[T]) {
	rotated = v.Copy().RotateAroundX(angle, pivot)
	return
}

// RotatedAroundY returns a new Vec3 rotated around the Y-axis passing through a pivot point.
func (v *Vec3[T]) RotatedAroundY(angle T, pivot *Vec3[T]) (rotated *Vec3[T]) {
	rotated = v.Copy().RotateAroundY(angle, pivot)
	return
}

// RotatedAroundZ returns a new Vec3 rotated around the Z-axis passing through a pivot point.
func (v *Vec3[T]) RotatedAroundZ(angle T, pivot *Vec3[T]) (rotated *Vec3[T]) {
	rotated = v.Copy().RotateAroundZ(angle, pivot)
	return
}

// Lerped returns a new Vec3 that is the result of linear interpolation between the original and a target Vec3 based on the parameter t (0 <= t <= 1).
func (v *Vec3[T]) Lerped(previous, current *Vec3[T], t T) (lerped *Vec3[T]) {
	lerped = v.Copy().Lerp(previous, current, t)
	return
}

// Other Methods \\

// Dist calculates and returns the distance between the current Vec3 and another Vec3.
func (v *Vec3[T]) Dist(other *Vec3[T]) (dist T) {
	var dx, dy, dz T = v.X - other.X, v.Y - other.Y, v.Z - other.Z
	dist = T(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
	return
}

// DistSquared calculates and returns the squared distance between the current Vec3 and another Vec3.
// This is more efficient than Dist when you only need to compare distances, as it avoids the costly square root operation.
func (v *Vec3[T]) DistSquared(other *Vec3[T]) (distSquared T) {
	var dx, dy, dz T = v.X - other.X, v.Y - other.Y, v.Z - other.Z
	distSquared = dx*dx + dy*dy + dz*dz
	return
}

// AngleTo calculates and returns the smallest angle in radians between the current Vec3 and another Vec3.
// It returns 0 if either Vec3 has zero length.
func (v *Vec3[T]) AngleTo(other *Vec3[T]) (angle T) {
	var lengths T = v.Length() * other.Length()
	if lengths == 0 {
		return
	}

	var cosine float64 = float64(v.Dot(other) / lengths)
	cosine = math.Max(-1, math.Min(1, cosine))
	angle = T(math.Acos(cosine))
	return
}
