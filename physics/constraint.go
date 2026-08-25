package physics

import (
	"fmt"
	"math"

	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	// ConstraintID uniquely identifies a constraint within one physics world.
	ConstraintID uint64

	// DistanceConstraintConfig controls a center-to-center distance constraint.
	DistanceConstraintConfig[T constraints.Float] struct {
		First, Second BodyID
		RestLength    T
		Compliance    T
		Damping       T
		BreakForce    T
	}

	// AnchoredDistanceConstraint2Config controls a distance constraint between local 2D anchors.
	AnchoredDistanceConstraint2Config[T constraints.Float] struct {
		First, Second                               BodyID
		LocalAnchorFirst, LocalAnchorSecond         vector.Vec2[T]
		RestLength, Compliance, Damping, BreakForce T
	}

	// AnchoredDistanceConstraint3Config controls a distance constraint between local 3D anchors.
	AnchoredDistanceConstraint3Config[T constraints.Float] struct {
		First, Second                               BodyID
		LocalAnchorFirst, LocalAnchorSecond         vector.Vec3[T]
		RestLength, Compliance, Damping, BreakForce T
	}

	// DistanceConstraint2 maintains a distance between two 2D body centers.
	DistanceConstraint2[T constraints.Float] struct {
		ID                                  ConstraintID
		First, Second                       *Body2[T]
		RestLength, Compliance, Damping     T
		BreakForce, LastForce               T
		Broken                              bool
		LocalAnchorFirst, LocalAnchorSecond vector.Vec2[T]
		lambda                              T
	}

	// DistanceConstraint3 maintains a distance between two 3D body centers.
	DistanceConstraint3[T constraints.Float] struct {
		ID                                  ConstraintID
		First, Second                       *Body3[T]
		RestLength, Compliance, Damping     T
		BreakForce, LastForce               T
		Broken                              bool
		LocalAnchorFirst, LocalAnchorSecond vector.Vec3[T]
		lambda                              T
	}
)

// AddDistanceConstraint creates a 2D distance constraint between existing bodies.
func (w *World2[T]) AddDistanceConstraint(config DistanceConstraintConfig[T]) (constraint *DistanceConstraint2[T], err error) {
	constraint, err = w.AddAnchoredDistanceConstraint(AnchoredDistanceConstraint2Config[T]{First: config.First, Second: config.Second, RestLength: config.RestLength, Compliance: config.Compliance, Damping: config.Damping, BreakForce: config.BreakForce})
	return
}

// AddAnchoredDistanceConstraint creates a 2D distance constraint between body-local anchors.
func (w *World2[T]) AddAnchoredDistanceConstraint(config AnchoredDistanceConstraint2Config[T]) (constraint *DistanceConstraint2[T], err error) {
	var (
		first, second           *Body2[T]
		firstFound, secondFound bool
	)
	first, firstFound = w.bodies[config.First]
	second, secondFound = w.bodies[config.Second]
	if !firstFound || !secondFound || first == second {
		err = fmt.Errorf("physics: distance constraint requires two distinct World2 bodies")
		return
	}
	if config.RestLength <= 0 {
		var firstAnchor, secondAnchor vector.Vec2[T] = bodyAnchor2(first, config.LocalAnchorFirst), bodyAnchor2(second, config.LocalAnchorSecond)
		config.RestLength = firstAnchor.Dist(&secondAnchor)
	}
	w.nextConstraintID++
	constraint = &DistanceConstraint2[T]{
		ID: w.nextConstraintID, First: first, Second: second, RestLength: config.RestLength,
		Compliance: max(config.Compliance, 0), Damping: min(max(config.Damping, 0), 1), BreakForce: max(config.BreakForce, 0),
		LocalAnchorFirst: config.LocalAnchorFirst, LocalAnchorSecond: config.LocalAnchorSecond,
	}
	w.distanceConstraints = append(w.distanceConstraints, constraint)
	first.Wake()
	second.Wake()
	return
}

// AddDistanceConstraint creates a 3D distance constraint between existing bodies.
func (w *World3[T]) AddDistanceConstraint(config DistanceConstraintConfig[T]) (constraint *DistanceConstraint3[T], err error) {
	constraint, err = w.AddAnchoredDistanceConstraint(AnchoredDistanceConstraint3Config[T]{First: config.First, Second: config.Second, RestLength: config.RestLength, Compliance: config.Compliance, Damping: config.Damping, BreakForce: config.BreakForce})
	return
}

// AddAnchoredDistanceConstraint creates a 3D distance constraint between body-local anchors.
func (w *World3[T]) AddAnchoredDistanceConstraint(config AnchoredDistanceConstraint3Config[T]) (constraint *DistanceConstraint3[T], err error) {
	var (
		first, second           *Body3[T]
		firstFound, secondFound bool
	)
	first, firstFound = w.bodies[config.First]
	second, secondFound = w.bodies[config.Second]
	if !firstFound || !secondFound || first == second {
		err = fmt.Errorf("physics: distance constraint requires two distinct World3 bodies")
		return
	}
	if config.RestLength <= 0 {
		var firstAnchor, secondAnchor vector.Vec3[T] = bodyAnchor3(first, config.LocalAnchorFirst), bodyAnchor3(second, config.LocalAnchorSecond)
		config.RestLength = firstAnchor.Dist(&secondAnchor)
	}
	w.nextConstraintID++
	constraint = &DistanceConstraint3[T]{
		ID: w.nextConstraintID, First: first, Second: second, RestLength: config.RestLength,
		Compliance: max(config.Compliance, 0), Damping: min(max(config.Damping, 0), 1), BreakForce: max(config.BreakForce, 0),
		LocalAnchorFirst: config.LocalAnchorFirst, LocalAnchorSecond: config.LocalAnchorSecond,
	}
	w.distanceConstraints = append(w.distanceConstraints, constraint)
	first.Wake()
	second.Wake()
	return
}

// Repair re-enables a broken 2D constraint, optionally replacing its rest length.
func (c *DistanceConstraint2[T]) Repair(restLength T) {
	if restLength > 0 {
		c.RestLength = restLength
	}
	c.Broken, c.LastForce, c.lambda = false, 0, 0
	c.First.Wake()
	c.Second.Wake()
}

// Repair re-enables a broken 3D constraint, optionally replacing its rest length.
func (c *DistanceConstraint3[T]) Repair(restLength T) {
	if restLength > 0 {
		c.RestLength = restLength
	}
	c.Broken, c.LastForce, c.lambda = false, 0, 0
	c.First.Wake()
	c.Second.Wake()
}

// RemoveConstraint removes a 2D constraint by ID.
func (w *World2[T]) RemoveConstraint(id ConstraintID) (removed bool) {
	for i := range w.distanceConstraints {
		if w.distanceConstraints[i].ID == id {
			w.distanceConstraints = append(w.distanceConstraints[:i], w.distanceConstraints[i+1:]...)
			removed = true
			break
		}
	}
	for i := range w.angularConstraints {
		if w.angularConstraints[i].ID == id {
			w.angularConstraints = append(w.angularConstraints[:i], w.angularConstraints[i+1:]...)
			removed = true
			break
		}
	}
	for i := range w.areaConstraints {
		if w.areaConstraints[i].ID == id {
			w.areaConstraints = append(w.areaConstraints[:i], w.areaConstraints[i+1:]...)
			removed = true
			break
		}
	}
	return
}

// RemoveConstraint removes a 3D constraint by ID.
func (w *World3[T]) RemoveConstraint(id ConstraintID) (removed bool) {
	for i := range w.distanceConstraints {
		if w.distanceConstraints[i].ID == id {
			w.distanceConstraints = append(w.distanceConstraints[:i], w.distanceConstraints[i+1:]...)
			removed = true
			break
		}
	}
	for i := range w.angularConstraints {
		if w.angularConstraints[i].ID == id {
			w.angularConstraints = append(w.angularConstraints[:i], w.angularConstraints[i+1:]...)
			removed = true
			break
		}
	}
	for i := range w.volumeConstraints {
		if w.volumeConstraints[i].ID == id {
			w.volumeConstraints = append(w.volumeConstraints[:i], w.volumeConstraints[i+1:]...)
			removed = true
			break
		}
	}
	return
}

// DistanceConstraints returns a detached 2D constraint list.
func (w *World2[T]) DistanceConstraints() (constraints []*DistanceConstraint2[T]) {
	constraints = append([]*DistanceConstraint2[T](nil), w.distanceConstraints...)
	return
}

// DistanceConstraints returns a detached 3D constraint list.
func (w *World3[T]) DistanceConstraints() (constraints []*DistanceConstraint3[T]) {
	constraints = append([]*DistanceConstraint3[T](nil), w.distanceConstraints...)
	return
}

// solve applies one XPBD iteration to a 2D distance constraint.
func (c *DistanceConstraint2[T]) solve(dt T) {
	if c.Broken || c.First.Disabled || c.Second.Disabled {
		return
	}
	var (
		rA, rB   vector.Vec2[T] = bodyAnchorOffset2(c.First, c.LocalAnchorFirst), bodyAnchorOffset2(c.Second, c.LocalAnchorSecond)
		dx, dy   T              = c.Second.Position.X + rB.X - c.First.Position.X - rA.X, c.Second.Position.Y + rB.Y - c.First.Position.Y - rA.Y
		distance T              = T(math.Sqrt(float64(dx*dx + dy*dy)))
	)
	if distance == 0 {
		return
	}
	var (
		normalX, normalY T = dx / distance, dy / distance
		angularFirst     T = rA.X*normalY - rA.Y*normalX
		angularSecond    T = rB.X*normalY - rB.Y*normalX
		inverseMass      T = c.First.inverseMass + c.Second.inverseMass + angularFirst*angularFirst*c.First.inverseInertia + angularSecond*angularSecond*c.Second.inverseInertia
		alpha            T = c.Compliance / (dt * dt)
		deltaLambda      T
	)
	if inverseMass+alpha == 0 {
		return
	}
	deltaLambda = (-(distance - c.RestLength) - alpha*c.lambda) / (inverseMass + alpha)
	c.lambda += deltaLambda
	c.First.Position.X -= normalX * deltaLambda * c.First.inverseMass
	c.First.Position.Y -= normalY * deltaLambda * c.First.inverseMass
	c.Second.Position.X += normalX * deltaLambda * c.Second.inverseMass
	c.Second.Position.Y += normalY * deltaLambda * c.Second.inverseMass
	c.First.Rotation -= angularFirst * deltaLambda * c.First.inverseInertia
	c.Second.Rotation += angularSecond * deltaLambda * c.Second.inverseInertia
	c.LastForce = T(math.Abs(float64(c.lambda / (dt * dt))))
	if c.BreakForce > 0 && c.LastForce > c.BreakForce {
		c.Broken = true
	}
}

// damp reduces relative velocity along a 2D constraint axis.
func (c *DistanceConstraint2[T]) damp() {
	if c.Broken || c.Damping == 0 {
		return
	}
	var (
		rA, rB                      vector.Vec2[T] = bodyAnchorOffset2(c.First, c.LocalAnchorFirst), bodyAnchorOffset2(c.Second, c.LocalAnchorSecond)
		dx, dy                      T              = c.Second.Position.X + rB.X - c.First.Position.X - rA.X, c.Second.Position.Y + rB.Y - c.First.Position.Y - rA.Y
		distance                    T              = T(math.Sqrt(float64(dx*dx + dy*dy)))
		normalX, normalY            T
		angularFirst, angularSecond T
		inverseMass                 T
	)
	if distance == 0 {
		return
	}
	normalX, normalY = dx/distance, dy/distance
	angularFirst, angularSecond = rA.X*normalY-rA.Y*normalX, rB.X*normalY-rB.Y*normalX
	inverseMass = c.First.inverseMass + c.Second.inverseMass + angularFirst*angularFirst*c.First.inverseInertia + angularSecond*angularSecond*c.Second.inverseInertia
	if inverseMass == 0 {
		return
	}
	var relative T = (c.Second.Velocity.X-c.Second.AngularVelocity*rB.Y-c.First.Velocity.X+c.First.AngularVelocity*rA.Y)*normalX + (c.Second.Velocity.Y+c.Second.AngularVelocity*rB.X-c.First.Velocity.Y-c.First.AngularVelocity*rA.X)*normalY
	var impulse T = -relative * c.Damping / inverseMass
	c.First.Velocity.X -= normalX * impulse * c.First.inverseMass
	c.First.Velocity.Y -= normalY * impulse * c.First.inverseMass
	c.Second.Velocity.X += normalX * impulse * c.Second.inverseMass
	c.Second.Velocity.Y += normalY * impulse * c.Second.inverseMass
	c.First.AngularVelocity -= angularFirst * impulse * c.First.inverseInertia
	c.Second.AngularVelocity += angularSecond * impulse * c.Second.inverseInertia
}

// solve applies one XPBD iteration to a 3D distance constraint.
func (c *DistanceConstraint3[T]) solve(dt T) {
	if c.Broken || c.First.Disabled || c.Second.Disabled {
		return
	}
	var (
		rA, rB     vector.Vec3[T] = bodyAnchorOffset3(c.First, c.LocalAnchorFirst), bodyAnchorOffset3(c.Second, c.LocalAnchorSecond)
		dx, dy, dz T              = c.Second.Position.X + rB.X - c.First.Position.X - rA.X, c.Second.Position.Y + rB.Y - c.First.Position.Y - rA.Y, c.Second.Position.Z + rB.Z - c.First.Position.Z - rA.Z
		distance   T              = T(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
	)
	if distance == 0 {
		return
	}
	var (
		normal                        vector.Vec3[T] = vector.Vec3[T]{X: dx / distance, Y: dy / distance, Z: dz / distance}
		angularFirst                  vector.Vec3[T] = rA
		angularSecond                 vector.Vec3[T] = rB
		responseFirst, responseSecond vector.Vec3[T]
		inverseMass                   T
		alpha                         T = c.Compliance / (dt * dt)
		deltaLambda                   T
	)
	angularFirst.Cross(&normal)
	angularSecond.Cross(&normal)
	responseFirst, responseSecond = c.First.applyInverseInertia(angularFirst), c.Second.applyInverseInertia(angularSecond)
	inverseMass = c.First.inverseMass + c.Second.inverseMass + angularFirst.Dot(&responseFirst) + angularSecond.Dot(&responseSecond)
	if inverseMass+alpha == 0 {
		return
	}
	deltaLambda = (-(distance - c.RestLength) - alpha*c.lambda) / (inverseMass + alpha)
	c.lambda += deltaLambda
	c.First.Position.X -= normal.X * deltaLambda * c.First.inverseMass
	c.First.Position.Y -= normal.Y * deltaLambda * c.First.inverseMass
	c.First.Position.Z -= normal.Z * deltaLambda * c.First.inverseMass
	c.Second.Position.X += normal.X * deltaLambda * c.Second.inverseMass
	c.Second.Position.Y += normal.Y * deltaLambda * c.Second.inverseMass
	c.Second.Position.Z += normal.Z * deltaLambda * c.Second.inverseMass
	responseFirst.Mul(-deltaLambda)
	responseSecond.Mul(deltaLambda)
	c.First.Orientation.ApplyRotationVector(responseFirst)
	c.Second.Orientation.ApplyRotationVector(responseSecond)
	c.LastForce = T(math.Abs(float64(c.lambda / (dt * dt))))
	if c.BreakForce > 0 && c.LastForce > c.BreakForce {
		c.Broken = true
	}
}

// damp reduces relative velocity along a 3D constraint axis.
func (c *DistanceConstraint3[T]) damp() {
	if c.Broken || c.Damping == 0 {
		return
	}
	var (
		rA, rB   vector.Vec3[T] = bodyAnchorOffset3(c.First, c.LocalAnchorFirst), bodyAnchorOffset3(c.Second, c.LocalAnchorSecond)
		delta    vector.Vec3[T] = vector.Vec3[T]{X: c.Second.Position.X + rB.X - c.First.Position.X - rA.X, Y: c.Second.Position.Y + rB.Y - c.First.Position.Y - rA.Y, Z: c.Second.Position.Z + rB.Z - c.First.Position.Z - rA.Z}
		distance T              = delta.Length()
	)
	if distance == 0 {
		return
	}
	delta.Mul(1 / distance)
	var angularFirst, angularSecond vector.Vec3[T] = c.First.AngularVelocity, c.Second.AngularVelocity
	angularFirst.Cross(&rA)
	angularSecond.Cross(&rB)
	var relative vector.Vec3[T] = vector.Vec3[T]{X: c.Second.Velocity.X + angularSecond.X - c.First.Velocity.X - angularFirst.X, Y: c.Second.Velocity.Y + angularSecond.Y - c.First.Velocity.Y - angularFirst.Y, Z: c.Second.Velocity.Z + angularSecond.Z - c.First.Velocity.Z - angularFirst.Z}
	var crossFirst, crossSecond vector.Vec3[T] = rA, rB
	crossFirst.Cross(&delta)
	crossSecond.Cross(&delta)
	var responseFirst, responseSecond vector.Vec3[T] = c.First.applyInverseInertia(crossFirst), c.Second.applyInverseInertia(crossSecond)
	var inverseMass T = c.First.inverseMass + c.Second.inverseMass + crossFirst.Dot(&responseFirst) + crossSecond.Dot(&responseSecond)
	if inverseMass == 0 {
		return
	}
	var impulse T = -relative.Dot(&delta) * c.Damping / inverseMass
	c.First.Velocity.X -= delta.X * impulse * c.First.inverseMass
	c.First.Velocity.Y -= delta.Y * impulse * c.First.inverseMass
	c.First.Velocity.Z -= delta.Z * impulse * c.First.inverseMass
	c.Second.Velocity.X += delta.X * impulse * c.Second.inverseMass
	c.Second.Velocity.Y += delta.Y * impulse * c.Second.inverseMass
	c.Second.Velocity.Z += delta.Z * impulse * c.Second.inverseMass
	responseFirst.Mul(-impulse)
	responseSecond.Mul(impulse)
	c.First.AngularVelocity.Add(&responseFirst)
	c.Second.AngularVelocity.Add(&responseSecond)
}

// bodyAnchorOffset2 rotates a local 2D anchor into world space.
func bodyAnchorOffset2[T constraints.Float](body *Body2[T], local vector.Vec2[T]) (offset vector.Vec2[T]) {
	var sine, cosine float64 = math.Sincos(float64(body.Rotation))
	offset = vector.Vec2[T]{X: local.X*T(cosine) - local.Y*T(sine), Y: local.X*T(sine) + local.Y*T(cosine)}
	return
}

// bodyAnchor2 returns a local 2D anchor in world coordinates.
func bodyAnchor2[T constraints.Float](body *Body2[T], local vector.Vec2[T]) (anchor vector.Vec2[T]) {
	anchor = bodyAnchorOffset2(body, local)
	anchor.X += body.Position.X
	anchor.Y += body.Position.Y
	return
}

// bodyAnchorOffset3 rotates a local 3D anchor into world space.
func bodyAnchorOffset3[T constraints.Float](body *Body3[T], local vector.Vec3[T]) (offset vector.Vec3[T]) {
	offset = body.Orientation.Rotate(local)
	return
}

// bodyAnchor3 returns a local 3D anchor in world coordinates.
func bodyAnchor3[T constraints.Float](body *Body3[T], local vector.Vec3[T]) (anchor vector.Vec3[T]) {
	anchor = bodyAnchorOffset3(body, local)
	anchor.Add(&body.Position)
	return
}
