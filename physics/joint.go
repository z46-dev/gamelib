package physics

import (
	"fmt"
	"math"

	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	// AngularConstraint2Config controls relative angular locking in two dimensions.
	AngularConstraint2Config[T constraints.Float] struct {
		First, Second                                    BodyID
		ReferenceAngle, Compliance, Damping, BreakTorque T
		UseCurrentAngle                                  bool
	}

	// AngularConstraint3Config controls relative orientation locking in three dimensions.
	AngularConstraint3Config[T constraints.Float] struct {
		First, Second                    BodyID
		ReferenceOrientation             Quaternion[T]
		Compliance, Damping, BreakTorque T
		UseCurrentOrientation            bool
	}

	// AngularConstraint2 maintains a relative angle between two bodies.
	AngularConstraint2[T constraints.Float] struct {
		ID                                                           ConstraintID
		First, Second                                                *Body2[T]
		ReferenceAngle, Compliance, Damping, BreakTorque, LastTorque T
		Broken                                                       bool
		lambda                                                       T
	}

	// AngularConstraint3 maintains a relative orientation between two bodies.
	AngularConstraint3[T constraints.Float] struct {
		ID                                           ConstraintID
		First, Second                                *Body3[T]
		ReferenceOrientation                         Quaternion[T]
		Compliance, Damping, BreakTorque, LastTorque T
		Broken                                       bool
		lambda                                       vector.Vec3[T]
	}

	// RevoluteJoint2 pins two anchors while allowing relative rotation.
	RevoluteJoint2[T constraints.Float] struct{ Anchor *DistanceConstraint2[T] }
	// FixedJoint2 pins two anchors and locks their relative angle.
	FixedJoint2[T constraints.Float] struct {
		Anchor *DistanceConstraint2[T]
		Angle  *AngularConstraint2[T]
	}
	// BallJoint3 pins two anchors while allowing relative rotation.
	BallJoint3[T constraints.Float] struct{ Anchor *DistanceConstraint3[T] }
	// FixedJoint3 pins two anchors and locks their relative orientation.
	FixedJoint3[T constraints.Float] struct {
		Anchor      *DistanceConstraint3[T]
		Orientation *AngularConstraint3[T]
	}
)

// AddAngularConstraint creates a relative-angle constraint in a 2D world.
func (w *World2[T]) AddAngularConstraint(config AngularConstraint2Config[T]) (constraint *AngularConstraint2[T], err error) {
	var first, second *Body2[T]
	var firstFound, secondFound bool
	first, firstFound = w.bodies[config.First]
	second, secondFound = w.bodies[config.Second]
	if !firstFound || !secondFound || first == second {
		err = fmt.Errorf("physics: angular constraint requires two distinct World2 bodies")
		return
	}
	if config.UseCurrentAngle {
		config.ReferenceAngle = second.Rotation - first.Rotation
	}
	w.nextConstraintID++
	constraint = &AngularConstraint2[T]{ID: w.nextConstraintID, First: first, Second: second, ReferenceAngle: config.ReferenceAngle, Compliance: max(config.Compliance, 0), Damping: min(max(config.Damping, 0), 1), BreakTorque: max(config.BreakTorque, 0)}
	w.angularConstraints = append(w.angularConstraints, constraint)
	first.Wake()
	second.Wake()
	return
}

// AddAngularConstraint creates a relative-orientation constraint in a 3D world.
func (w *World3[T]) AddAngularConstraint(config AngularConstraint3Config[T]) (constraint *AngularConstraint3[T], err error) {
	var first, second *Body3[T]
	var firstFound, secondFound bool
	first, firstFound = w.bodies[config.First]
	second, secondFound = w.bodies[config.Second]
	if !firstFound || !secondFound || first == second {
		err = fmt.Errorf("physics: angular constraint requires two distinct World3 bodies")
		return
	}
	if config.UseCurrentOrientation || config.ReferenceOrientation == (Quaternion[T]{}) {
		config.ReferenceOrientation = first.Orientation.Conjugated().Multiplied(second.Orientation)
	}
	config.ReferenceOrientation.Normalize()
	w.nextConstraintID++
	constraint = &AngularConstraint3[T]{ID: w.nextConstraintID, First: first, Second: second, ReferenceOrientation: config.ReferenceOrientation, Compliance: max(config.Compliance, 0), Damping: min(max(config.Damping, 0), 1), BreakTorque: max(config.BreakTorque, 0)}
	w.angularConstraints = append(w.angularConstraints, constraint)
	first.Wake()
	second.Wake()
	return
}

// AddRevoluteJoint creates a 2D pin joint from body-local anchors.
func (w *World2[T]) AddRevoluteJoint(config AnchoredDistanceConstraint2Config[T]) (joint RevoluteJoint2[T], err error) {
	joint.Anchor, err = w.AddAnchoredDistanceConstraint(config)
	return
}

// AddFixedJoint creates a 2D pin and angular lock from body-local anchors.
func (w *World2[T]) AddFixedJoint(config AnchoredDistanceConstraint2Config[T]) (joint FixedJoint2[T], err error) {
	if joint.Anchor, err = w.AddAnchoredDistanceConstraint(config); err != nil {
		return
	}
	joint.Angle, err = w.AddAngularConstraint(AngularConstraint2Config[T]{First: config.First, Second: config.Second, Compliance: config.Compliance, Damping: config.Damping, BreakTorque: config.BreakForce, UseCurrentAngle: true})
	return
}

// AddBallJoint creates a 3D ball-and-socket joint from body-local anchors.
func (w *World3[T]) AddBallJoint(config AnchoredDistanceConstraint3Config[T]) (joint BallJoint3[T], err error) {
	joint.Anchor, err = w.AddAnchoredDistanceConstraint(config)
	return
}

// AddFixedJoint creates a 3D positional and orientation lock from body-local anchors.
func (w *World3[T]) AddFixedJoint(config AnchoredDistanceConstraint3Config[T]) (joint FixedJoint3[T], err error) {
	if joint.Anchor, err = w.AddAnchoredDistanceConstraint(config); err != nil {
		return
	}
	joint.Orientation, err = w.AddAngularConstraint(AngularConstraint3Config[T]{First: config.First, Second: config.Second, Compliance: config.Compliance, Damping: config.Damping, BreakTorque: config.BreakForce, UseCurrentOrientation: true})
	return
}

// Repair re-enables a broken 2D angular constraint.
func (c *AngularConstraint2[T]) Repair() {
	c.Broken, c.LastTorque, c.lambda = false, 0, 0
	c.First.Wake()
	c.Second.Wake()
}

// Repair re-enables a broken 3D angular constraint.
func (c *AngularConstraint3[T]) Repair() {
	c.Broken, c.LastTorque = false, 0
	c.lambda = vector.Vec3[T]{}
	c.First.Wake()
	c.Second.Wake()
}

// AngularConstraints returns a detached 2D angular-constraint list.
func (w *World2[T]) AngularConstraints() (result []*AngularConstraint2[T]) {
	result = append(result, w.angularConstraints...)
	return
}

// AngularConstraints returns a detached 3D angular-constraint list.
func (w *World3[T]) AngularConstraints() (result []*AngularConstraint3[T]) {
	result = append(result, w.angularConstraints...)
	return
}

func wrapAngle[T constraints.Float](angle T) (wrapped T) {
	wrapped = T(math.Remainder(float64(angle), 2*math.Pi))
	return
}

func (c *AngularConstraint2[T]) solve(dt T) {
	if c.Broken || c.First.Disabled || c.Second.Disabled {
		return
	}
	var alpha T = c.Compliance / (dt * dt)
	var inverseMass T = c.First.inverseInertia + c.Second.inverseInertia
	if inverseMass+alpha == 0 {
		return
	}
	var delta T = (-wrapAngle(c.Second.Rotation-c.First.Rotation-c.ReferenceAngle) - alpha*c.lambda) / (inverseMass + alpha)
	c.lambda += delta
	c.First.Rotation -= delta * c.First.inverseInertia
	c.Second.Rotation += delta * c.Second.inverseInertia
	c.LastTorque = T(math.Abs(float64(c.lambda / (dt * dt))))
	if c.BreakTorque > 0 && c.LastTorque > c.BreakTorque {
		c.Broken = true
	}
}

func (c *AngularConstraint2[T]) damp() {
	var inverseMass T = c.First.inverseInertia + c.Second.inverseInertia
	if c.Broken || c.Damping == 0 || inverseMass == 0 {
		return
	}
	var impulse T = -(c.Second.AngularVelocity - c.First.AngularVelocity) * c.Damping / inverseMass
	c.First.AngularVelocity -= impulse * c.First.inverseInertia
	c.Second.AngularVelocity += impulse * c.Second.inverseInertia
}

func (c *AngularConstraint3[T]) solve(dt T) {
	if c.Broken || c.First.Disabled || c.Second.Disabled {
		return
	}
	var current Quaternion[T] = c.First.Orientation.Conjugated().Multiplied(c.Second.Orientation)
	var error vector.Vec3[T] = c.ReferenceOrientation.Conjugated().Multiplied(current).RotationVector()
	error = c.First.Orientation.Rotate(error)
	var magnitude T = error.Length()
	if magnitude == 0 {
		return
	}
	var axis vector.Vec3[T] = error
	axis.Mul(1 / magnitude)
	var firstResponse, secondResponse vector.Vec3[T] = c.First.applyInverseInertia(axis), c.Second.applyInverseInertia(axis)
	var alpha T = c.Compliance / (dt * dt)
	var denominator T = axis.Dot(&firstResponse) + axis.Dot(&secondResponse) + alpha
	if denominator == 0 {
		return
	}
	var accumulated T = c.lambda.Dot(&axis)
	var delta T = (-magnitude - alpha*accumulated) / denominator
	c.lambda.X += axis.X * delta
	c.lambda.Y += axis.Y * delta
	c.lambda.Z += axis.Z * delta
	firstResponse.Mul(-delta)
	secondResponse.Mul(delta)
	c.First.Orientation.ApplyRotationVector(firstResponse)
	c.Second.Orientation.ApplyRotationVector(secondResponse)
	c.LastTorque = c.lambda.Length() / (dt * dt)
	if c.BreakTorque > 0 && c.LastTorque > c.BreakTorque {
		c.Broken = true
	}
}

func (c *AngularConstraint3[T]) damp() {
	if c.Broken || c.Damping == 0 {
		return
	}
	var relative vector.Vec3[T] = vector.Vec3[T]{X: c.Second.AngularVelocity.X - c.First.AngularVelocity.X, Y: c.Second.AngularVelocity.Y - c.First.AngularVelocity.Y, Z: c.Second.AngularVelocity.Z - c.First.AngularVelocity.Z}
	var magnitude T = relative.Length()
	if magnitude == 0 {
		return
	}
	var axis vector.Vec3[T] = relative
	axis.Mul(1 / magnitude)
	var firstResponse, secondResponse vector.Vec3[T] = c.First.applyInverseInertia(axis), c.Second.applyInverseInertia(axis)
	var denominator T = axis.Dot(&firstResponse) + axis.Dot(&secondResponse)
	if denominator == 0 {
		return
	}
	var impulse T = -magnitude * c.Damping / denominator
	firstResponse.Mul(-impulse)
	secondResponse.Mul(impulse)
	c.First.AngularVelocity.Add(&firstResponse)
	c.Second.AngularVelocity.Add(&secondResponse)
}
