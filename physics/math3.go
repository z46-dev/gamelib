package physics

import (
	"math"

	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	// Quaternion represents a normalized three-dimensional rotation.
	Quaternion[T constraints.Float] struct {
		X, Y, Z, W T
	}
)

// IdentityQuaternion returns a quaternion containing no rotation.
func IdentityQuaternion[T constraints.Float]() (rotation Quaternion[T]) {
	rotation.W = 1
	return
}

// QuaternionFromEuler creates a rotation from XYZ Euler angles in radians.
func QuaternionFromEuler[T constraints.Float](rotation vector.Vec3[T]) (quaternion Quaternion[T]) {
	var (
		halfX, halfY, halfZ float64 = float64(rotation.X) / 2, float64(rotation.Y) / 2, float64(rotation.Z) / 2
		cx, sx              float64 = math.Cos(halfX), math.Sin(halfX)
		cy, sy              float64 = math.Cos(halfY), math.Sin(halfY)
		cz, sz              float64 = math.Cos(halfZ), math.Sin(halfZ)
	)
	quaternion = Quaternion[T]{X: T(sx*cy*cz + cx*sy*sz), Y: T(cx*sy*cz - sx*cy*sz), Z: T(cx*cy*sz + sx*sy*cz), W: T(cx*cy*cz - sx*sy*sz)}
	quaternion.Normalize()
	return
}

// Normalize restores unit length and replaces a zero quaternion with identity.
func (q *Quaternion[T]) Normalize() {
	var length T = T(math.Sqrt(float64(q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W)))
	if length == 0 {
		*q = IdentityQuaternion[T]()
		return
	}
	q.X, q.Y, q.Z, q.W = q.X/length, q.Y/length, q.Z/length, q.W/length
}

// Integrate advances orientation from world-space angular velocity.
func (q *Quaternion[T]) Integrate(angularVelocity vector.Vec3[T], dt T) {
	var halfDT T = dt / 2
	var (
		dx T = (angularVelocity.X*q.W + angularVelocity.Y*q.Z - angularVelocity.Z*q.Y) * halfDT
		dy T = (-angularVelocity.X*q.Z + angularVelocity.Y*q.W + angularVelocity.Z*q.X) * halfDT
		dz T = (angularVelocity.X*q.Y - angularVelocity.Y*q.X + angularVelocity.Z*q.W) * halfDT
		dw T = (-angularVelocity.X*q.X - angularVelocity.Y*q.Y - angularVelocity.Z*q.Z) * halfDT
	)
	q.X, q.Y, q.Z, q.W = q.X+dx, q.Y+dy, q.Z+dz, q.W+dw
	q.Normalize()
}

// Rotate transforms a vector from local space into world space.
func (q Quaternion[T]) Rotate(value vector.Vec3[T]) (rotated vector.Vec3[T]) {
	var (
		tx T = 2 * (q.Y*value.Z - q.Z*value.Y)
		ty T = 2 * (q.Z*value.X - q.X*value.Z)
		tz T = 2 * (q.X*value.Y - q.Y*value.X)
	)
	rotated = vector.Vec3[T]{X: value.X + q.W*tx + q.Y*tz - q.Z*ty, Y: value.Y + q.W*ty + q.Z*tx - q.X*tz, Z: value.Z + q.W*tz + q.X*ty - q.Y*tx}
	return
}

// InverseRotate transforms a vector from world space into local space.
func (q Quaternion[T]) InverseRotate(value vector.Vec3[T]) (rotated vector.Vec3[T]) {
	q.X, q.Y, q.Z = -q.X, -q.Y, -q.Z
	rotated = q.Rotate(value)
	return
}

// Multiplied returns the composition of this rotation followed by another.
func (q Quaternion[T]) Multiplied(other Quaternion[T]) (product Quaternion[T]) {
	product = Quaternion[T]{
		X: q.W*other.X + q.X*other.W + q.Y*other.Z - q.Z*other.Y,
		Y: q.W*other.Y - q.X*other.Z + q.Y*other.W + q.Z*other.X,
		Z: q.W*other.Z + q.X*other.Y - q.Y*other.X + q.Z*other.W,
		W: q.W*other.W - q.X*other.X - q.Y*other.Y - q.Z*other.Z,
	}
	return
}

// Conjugated returns the inverse of a normalized quaternion.
func (q Quaternion[T]) Conjugated() (conjugate Quaternion[T]) {
	conjugate = Quaternion[T]{X: -q.X, Y: -q.Y, Z: -q.Z, W: q.W}
	return
}

// RotationVector returns the shortest axis-angle rotation as a vector.
func (q Quaternion[T]) RotationVector() (rotation vector.Vec3[T]) {
	if q.W < 0 {
		q.X, q.Y, q.Z, q.W = -q.X, -q.Y, -q.Z, -q.W
	}
	var vectorLength T = T(math.Sqrt(float64(q.X*q.X + q.Y*q.Y + q.Z*q.Z)))
	if vectorLength == 0 {
		return
	}
	var angle T = 2 * T(math.Atan2(float64(vectorLength), float64(q.W)))
	rotation = vector.Vec3[T]{X: q.X / vectorLength * angle, Y: q.Y / vectorLength * angle, Z: q.Z / vectorLength * angle}
	return
}

// ApplyRotationVector applies a small world-space angular correction.
func (q *Quaternion[T]) ApplyRotationVector(rotation vector.Vec3[T]) {
	var angle T = rotation.Length()
	if angle == 0 {
		return
	}
	var (
		halfAngle float64       = float64(angle) / 2
		scale     T             = T(math.Sin(halfAngle)) / angle
		delta     Quaternion[T] = Quaternion[T]{X: rotation.X * scale, Y: rotation.Y * scale, Z: rotation.Z * scale, W: T(math.Cos(halfAngle))}
	)
	*q = delta.Multiplied(*q)
	q.Normalize()
}

// Euler returns equivalent XYZ Euler angles for geometry adapters.
func (q Quaternion[T]) Euler() (rotation vector.Vec3[T]) {
	var (
		x, y, z, w float64 = float64(q.X), float64(q.Y), float64(q.Z), float64(q.W)
		sinX, cosX float64 = 2 * (w*x - y*z), 1 - 2*(x*x+y*y)
		sinY       float64 = 2 * (w*y + z*x)
		sinZ, cosZ float64 = 2 * (w*z - x*y), 1 - 2*(y*y+z*z)
	)
	rotation.X = T(math.Atan2(sinX, cosX))
	rotation.Y = T(math.Asin(max(-1, min(1, sinY))))
	rotation.Z = T(math.Atan2(sinZ, cosZ))
	return
}
