package physics

import (
	"fmt"
	"math"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	cachedContact3[T constraints.Float] struct {
		normalImpulse             T
		tangentImpulse            vector.Vec3[T]
		normal                    vector.Vec3[T]
		firstAnchor, secondAnchor vector.Vec3[T]
		generation                uint64
	}

	// Body3Config describes a three-dimensional rigid body at creation time.
	Body3Config[T constraints.Float] struct {
		Type                                                        BodyType
		Shape                                                       Shape3[T]
		Position, Rotation, Velocity                                vector.Vec3[T]
		Orientation                                                 Quaternion[T]
		AngularVelocity                                             vector.Vec3[T]
		Mass                                                        T
		Material                                                    Material[T]
		Filter                                                      CollisionFilter
		GravityScale, LinearDamping, AngularDamping                 T
		Sensor, FixedRotation, DisableGravity, Disabled, Continuous bool
	}

	// Body3 stores the mutable state of one three-dimensional rigid body.
	Body3[T constraints.Float] struct {
		ID                                          BodyID
		Type                                        BodyType
		Shape                                       Shape3[T]
		Position, Velocity, Force                   vector.Vec3[T]
		Orientation                                 Quaternion[T]
		AngularVelocity, Torque                     vector.Vec3[T]
		Material                                    Material[T]
		Filter                                      CollisionFilter
		GravityScale, LinearDamping, AngularDamping T
		Sensor, FixedRotation, Disabled, Continuous bool
		Sleeping                                    bool

		mass, inverseMass       T
		inertia, inverseInertia vector.Vec3[T]
		inverseInertiaTensor    [9]T
		sleepTime               T
		islandIndex             int
		world                   *World3[T]
	}

	// Contact3 describes one active three-dimensional body contact.
	Contact3[T constraints.Float] struct {
		First, Second *Body3[T]
		Point, Normal vector.Vec3[T]
		Penetration   T

		pair            contactPair
		normalImpulse   T
		tangentImpulse  vector.Vec3[T]
		restitutionBias T
		manifoldCount   int
		feature         uint32
	}

	// World3 owns and advances a collection of three-dimensional rigid bodies.
	World3[T constraints.Float] struct {
		Config   WorldConfig[T]
		Contacts []Contact3[T]

		nextID                                 BodyID
		bodies                                 map[BodyID]*Body3[T]
		bodyOrder                              []*Body3[T]
		spatialHash                            *hshg.SpatialHash3[*Body3[T], T]
		spatialHashDirty                       bool
		candidates                             []*Body3[T]
		contactCache                           map[contactPair]cachedContact3[T]
		generation                             uint64
		nextConstraintID                       ConstraintID
		distanceConstraints                    []*DistanceConstraint3[T]
		angularConstraints                     []*AngularConstraint3[T]
		ropes                                  []*Rope3[T]
		volumeConstraints                      []*VolumeConstraint3[T]
		islandParent, islandRank, islandLookup []int
		islands                                []island3[T]
	}
)

// NewWorld3 creates an empty three-dimensional physics world.
func NewWorld3[T constraints.Float](config WorldConfig[T]) (world *World3[T]) {
	if config.VelocityIterations <= 0 {
		config.VelocityIterations = 8
	}
	if config.PositionIterations <= 0 {
		config.PositionIterations = 3
	}
	if config.PositionCorrection <= 0 {
		config.PositionCorrection = 0.2
	}
	if config.MaxStepDelta <= 0 {
		config.MaxStepDelta = T(1.0 / 60.0)
	}
	var spatialHash *hshg.SpatialHash3[*Body3[T], T]
	if len(config.SpatialHash.Levels) == 0 {
		spatialHash = hshg.NewSpatialHash3[*Body3[T], T]()
	} else {
		spatialHash = hshg.NewSpatialHash3[*Body3[T], T](hshg.WithSpatialHash3Config(config.SpatialHash))
	}
	world = &World3[T]{Config: config, nextID: 1, bodies: make(map[BodyID]*Body3[T]), spatialHash: spatialHash, spatialHashDirty: true, contactCache: make(map[contactPair]cachedContact3[T])}
	return
}

// AddBody creates and registers a body, cloning its shape for independent transforms.
func (w *World3[T]) AddBody(config Body3Config[T]) (body *Body3[T], err error) {
	if config.Shape == nil {
		err = fmt.Errorf("physics: Body3 requires a shape")
		return
	}
	if config.Type > DynamicBody {
		err = fmt.Errorf("physics: invalid Body3 type %d", config.Type)
		return
	}
	if polyhedron, ok := config.Shape.(*Polyhedron3[T]); ok && config.Type == DynamicBody && !polyhedronIsConvex3(polyhedron.Polyhedron) {
		err = fmt.Errorf("physics: dynamic Polyhedron3 must be convex; decompose concave bodies into convex parts")
		return
	}
	if config.Material == (Material[T]{}) {
		config.Material = DefaultMaterial[T]()
	}
	if config.Filter == (CollisionFilter{}) {
		config.Filter = DefaultCollisionFilter()
	}
	if config.DisableGravity {
		config.GravityScale = 0
	} else if config.GravityScale == 0 && config.Type == DynamicBody {
		config.GravityScale = 1
	}
	body = &Body3[T]{
		ID: w.nextID, Type: config.Type, Shape: config.Shape.Clone(), Position: config.Position, Orientation: config.Orientation, Velocity: config.Velocity,
		AngularVelocity: config.AngularVelocity, Material: clampMaterial(config.Material), Filter: config.Filter, GravityScale: config.GravityScale,
		LinearDamping: max(config.LinearDamping, 0), AngularDamping: max(config.AngularDamping, 0), Sensor: config.Sensor,
		FixedRotation: config.FixedRotation, Disabled: config.Disabled, Continuous: config.Continuous,
	}
	if body.Orientation == (Quaternion[T]{}) {
		body.Orientation = QuaternionFromEuler(config.Rotation)
	}
	body.recalculateMass(config.Mass)
	body.syncShape()
	body.world = w
	w.nextID++
	w.bodies[body.ID] = body
	w.bodyOrder = append(w.bodyOrder, body)
	body.islandIndex = len(w.bodyOrder) - 1
	w.spatialHashDirty = true
	return
}

// RemoveBody removes a body by ID and reports whether it existed.
func (w *World3[T]) RemoveBody(id BodyID) (removed bool) {
	if _, removed = w.bodies[id]; !removed {
		return
	}
	delete(w.bodies, id)
	w.spatialHashDirty = true
	for i := len(w.distanceConstraints) - 1; i >= 0; i-- {
		if w.distanceConstraints[i].First.ID == id || w.distanceConstraints[i].Second.ID == id {
			w.distanceConstraints = append(w.distanceConstraints[:i], w.distanceConstraints[i+1:]...)
		}
	}
	for i := len(w.angularConstraints) - 1; i >= 0; i-- {
		if w.angularConstraints[i].First.ID == id || w.angularConstraints[i].Second.ID == id {
			w.angularConstraints = append(w.angularConstraints[:i], w.angularConstraints[i+1:]...)
		}
	}
	for i := len(w.volumeConstraints) - 1; i >= 0; i-- {
		var c *VolumeConstraint3[T] = w.volumeConstraints[i]
		if c.First.ID == id || c.Second.ID == id || c.Third.ID == id || c.Fourth.ID == id {
			w.volumeConstraints = append(w.volumeConstraints[:i], w.volumeConstraints[i+1:]...)
		}
	}
	for i := range w.bodyOrder {
		if w.bodyOrder[i].ID == id {
			w.bodyOrder = append(w.bodyOrder[:i], w.bodyOrder[i+1:]...)
			for index := i; index < len(w.bodyOrder); index++ {
				w.bodyOrder[index].islandIndex = index
			}
			break
		}
	}
	return
}

// Body returns a body by ID.
func (w *World3[T]) Body(id BodyID) (body *Body3[T], found bool) {
	body, found = w.bodies[id]
	return
}

// Bodies returns a detached list of bodies in deterministic creation order.
func (w *World3[T]) Bodies() (bodies []*Body3[T]) {
	bodies = append([]*Body3[T](nil), w.bodyOrder...)
	return
}

// BodiesInAABB returns enabled bodies whose bounds intersect the supplied bounds.
func (w *World3[T]) BodiesInAABB(bounds *hshg.AABB3[T]) (bodies []*Body3[T]) {
	bodies = w.BodiesInAABBInto(nil, bounds)
	return
}

// BodiesInAABBInto replaces bodies with enabled bodies whose bounds intersect the supplied bounds.
func (w *World3[T]) BodiesInAABBInto(bodies []*Body3[T], bounds *hshg.AABB3[T]) (result []*Body3[T]) {
	w.ensureSpatialHash()
	result = w.spatialHash.RetrieveInto(bodies, bounds)
	var count int
	for _, body := range result {
		if !body.Disabled {
			result[count] = body
			count++
		}
	}
	result = result[:count]
	return
}

// BodiesInRadius returns enabled bodies whose bounds intersect a sphere around center.
func (w *World3[T]) BodiesInRadius(center vector.Vec3[T], radius T) (bodies []*Body3[T]) {
	bodies = w.BodiesInRadiusInto(nil, center, radius)
	return
}

// BodiesInRadiusInto replaces bodies with enabled bodies whose bounds intersect a sphere around center.
func (w *World3[T]) BodiesInRadiusInto(bodies []*Body3[T], center vector.Vec3[T], radius T) (result []*Body3[T]) {
	w.ensureSpatialHash()
	radius = max(radius, 0)
	result = w.spatialHash.RetrieveAroundInto(bodies, center.X, center.Y, center.Z, radius)
	var radiusSquared T = radius * radius
	var count int
	for _, body := range result {
		if body.Disabled {
			continue
		}
		var bounds *hshg.AABB3[T] = body.GetAABB()
		var (
			closestX T = min(max(center.X, bounds.X1), bounds.X2)
			closestY T = min(max(center.Y, bounds.Y1), bounds.Y2)
			closestZ T = min(max(center.Z, bounds.Z1), bounds.Z2)
			deltaX   T = center.X - closestX
			deltaY   T = center.Y - closestY
			deltaZ   T = center.Z - closestZ
		)
		if deltaX*deltaX+deltaY*deltaY+deltaZ*deltaZ <= radiusSquared {
			result[count] = body
			count++
		}
	}
	result = result[:count]
	return
}

// GetAABB returns the body's shape bounds for SpatialHash3.
func (b *Body3[T]) GetAABB() (aabb *hshg.AABB3[T]) {
	aabb = b.Shape.GetAABB()
	return
}

// Mass returns the body's finite mass, or zero for non-dynamic bodies.
func (b *Body3[T]) Mass() (mass T) {
	mass = b.mass
	return
}

// ApplyForce accumulates a world-space force for the next step.
func (b *Body3[T]) ApplyForce(force vector.Vec3[T]) {
	if b.Type == DynamicBody && !b.Disabled {
		b.Wake()
		b.Force.X += force.X
		b.Force.Y += force.Y
		b.Force.Z += force.Z
	}
}

// ApplyForceAtPoint accumulates force and torque at a world-space point.
func (b *Body3[T]) ApplyForceAtPoint(force, point vector.Vec3[T]) {
	b.ApplyForce(force)
	if b.Type == DynamicBody && !b.FixedRotation && !b.Disabled {
		var arm vector.Vec3[T] = vector.Vec3[T]{X: point.X - b.Position.X, Y: point.Y - b.Position.Y, Z: point.Z - b.Position.Z}
		arm.Cross(&force)
		b.Torque.Add(&arm)
	}
}

// ApplyImpulse immediately changes linear velocity at the center of mass.
func (b *Body3[T]) ApplyImpulse(impulse vector.Vec3[T]) {
	if b.Type == DynamicBody && !b.Disabled {
		b.Wake()
		b.Velocity.X += impulse.X * b.inverseMass
		b.Velocity.Y += impulse.Y * b.inverseMass
		b.Velocity.Z += impulse.Z * b.inverseMass
	}
}

// ApplyImpulseAtPoint immediately changes linear and angular velocity.
func (b *Body3[T]) ApplyImpulseAtPoint(impulse, point vector.Vec3[T]) {
	b.ApplyImpulse(impulse)
	if b.Type == DynamicBody && !b.FixedRotation && !b.Disabled {
		var arm vector.Vec3[T] = vector.Vec3[T]{X: point.X - b.Position.X, Y: point.Y - b.Position.Y, Z: point.Z - b.Position.Z}
		arm.Cross(&impulse)
		var angularImpulse vector.Vec3[T] = b.applyInverseInertia(arm)
		b.AngularVelocity.Add(&angularImpulse)
	}
}

// SetTransform teleports a body and synchronizes its collider.
func (b *Body3[T]) SetTransform(position vector.Vec3[T], orientation Quaternion[T]) {
	b.Position, b.Orientation = position, orientation
	b.Orientation.Normalize()
	b.Wake()
	b.syncShape()
	if b.world != nil {
		b.world.spatialHashDirty = true
	}
}

// Wake returns a dynamic body to active simulation.
func (b *Body3[T]) Wake() {
	if b.Type == DynamicBody {
		b.Sleeping = false
		b.sleepTime = 0
	}
}

// Sleep immediately deactivates a dynamic body until it is disturbed.
func (b *Body3[T]) Sleep() {
	if b.Type == DynamicBody {
		b.Sleeping = true
		b.Velocity, b.Force = vector.Vec3[T]{}, vector.Vec3[T]{}
		b.AngularVelocity, b.Torque = vector.Vec3[T]{}, vector.Vec3[T]{}
	}
}

// SetEulerTransform teleports a body using XYZ Euler angles in radians.
func (b *Body3[T]) SetEulerTransform(position, rotation vector.Vec3[T]) {
	b.SetTransform(position, QuaternionFromEuler(rotation))
}

// Step advances forces, contacts, impulses, and positions by dt seconds.
func (w *World3[T]) Step(dt T) {
	if dt <= 0 {
		return
	}
	var substeps int = w.ccdSubsteps(dt)
	var substepDT T = dt / T(substeps)
	for range substeps {
		w.stepDiscrete(substepDT)
	}
	for _, body := range w.bodyOrder {
		body.Force, body.Torque = vector.Vec3[T]{}, vector.Vec3[T]{}
	}
	w.rebuildSpatialHash()
}

// stepDiscrete advances one collision-safe substep without clearing forces.
func (w *World3[T]) stepDiscrete(dt T) {
	for _, rope := range w.ropes {
		rope.UpdateReconnection()
	}
	for _, body := range w.bodyOrder {
		body.integrate(w.Config, dt)
		body.syncShape()
	}
	w.buildContacts()
	var islands []island3[T]
	if len(w.Contacts)+len(w.distanceConstraints)+len(w.angularConstraints)+len(w.volumeConstraints) > 0 {
		islands = w.buildIslands()
	}
	for _, constraint := range w.distanceConstraints {
		constraint.lambda = 0
	}
	for _, constraint := range w.angularConstraints {
		constraint.lambda = vector.Vec3[T]{}
	}
	for _, constraint := range w.volumeConstraints {
		constraint.lambda = 0
	}
	if w.Config.EnableWarmStarting {
		for i := range w.Contacts {
			warmStart3(&w.Contacts[i])
		}
	}
	w.solveIslands(islands, dt)
	w.storeContactCache()
	for _, body := range w.bodyOrder {
		body.syncShape()
	}
	if len(islands) > 0 {
		w.updateIslandSleeping(islands, dt)
	} else {
		for _, body := range w.bodyOrder {
			body.updateSleeping(w.Config, dt)
		}
	}
}

func (w *World3[T]) ccdSubsteps(dt T) (substeps int) {
	substeps = max(1, int(math.Ceil(float64(dt/w.Config.MaxStepDelta))))
	if w.Config.CCDMotionThreshold <= 0 {
		return
	}
	for _, body := range w.bodyOrder {
		if body.Type == StaticBody || body.Disabled || body.Sleeping || !w.Config.EnableCCD && !body.Continuous {
			continue
		}
		var bounds *hshg.AABB3[T] = body.GetAABB()
		var (
			width, height, depth T = bounds.X2 - bounds.X1, bounds.Y2 - bounds.Y1, bounds.Z2 - bounds.Z1
			limit                T = min(w.Config.CCDMotionThreshold, min(width, min(height, depth))/2)
			sweepRadius          T = T(math.Sqrt(float64(width*width+height*height+depth*depth))) / 2
		)
		if limit <= 0 {
			continue
		}
		var (
			linearSpeed         T = body.Velocity.Length()
			angularSpeed        T = body.AngularVelocity.Length()
			linearAcceleration  vector.Vec3[T]
			angularAcceleration vector.Vec3[T]
		)
		if body.Type == DynamicBody {
			linearAcceleration = vector.Vec3[T]{X: w.Config.GravityX*body.GravityScale + body.Force.X*body.inverseMass, Y: w.Config.GravityY*body.GravityScale + body.Force.Y*body.inverseMass, Z: w.Config.GravityZ*body.GravityScale + body.Force.Z*body.inverseMass}
			angularAcceleration = body.applyInverseInertia(body.Torque)
		}
		var required int = int(math.Ceil(float64((linearSpeed + linearAcceleration.Length()*dt + (angularSpeed+angularAcceleration.Length()*dt)*sweepRadius) * dt / limit)))
		if required > substeps {
			substeps = required
		}
	}
	return
}

// recalculateMass refreshes inverse mass and diagonal inertia from shape and material.
func (b *Body3[T]) recalculateMass(override T) {
	if b.Type != DynamicBody {
		return
	}
	b.mass = override
	if b.mass <= 0 {
		b.mass = b.Shape.Volume() * b.Material.Density
	}
	if b.mass <= 0 {
		b.mass = 1
	}
	b.inverseMass = 1 / b.mass
	if !b.FixedRotation {
		var tensor [9]T
		if tensorShape, ok := b.Shape.(inertiaTensorShape3[T]); ok {
			tensor = tensorShape.InertiaTensor(b.mass)
		} else {
			b.inertia = b.Shape.MomentOfInertia(b.mass)
			tensor[0], tensor[4], tensor[8] = b.inertia.X, b.inertia.Y, b.inertia.Z
		}
		b.inertia = vector.Vec3[T]{X: tensor[0], Y: tensor[4], Z: tensor[8]}
		b.inverseInertiaTensor = invertMatrix3(tensor)
		b.inverseInertia = vector.Vec3[T]{X: b.inverseInertiaTensor[0], Y: b.inverseInertiaTensor[4], Z: b.inverseInertiaTensor[8]}
	}
}

// integrate advances one body's velocities and transform.
func (b *Body3[T]) integrate(config WorldConfig[T], dt T) {
	if b.Disabled || b.Type == StaticBody || b.Sleeping {
		return
	}
	if b.Type == DynamicBody {
		b.Velocity.X += (config.GravityX*b.GravityScale + b.Force.X*b.inverseMass) * dt
		b.Velocity.Y += (config.GravityY*b.GravityScale + b.Force.Y*b.inverseMass) * dt
		b.Velocity.Z += (config.GravityZ*b.GravityScale + b.Force.Z*b.inverseMass) * dt
		var angularAcceleration vector.Vec3[T] = b.applyInverseInertia(b.Torque)
		b.AngularVelocity.X += angularAcceleration.X * dt
		b.AngularVelocity.Y += angularAcceleration.Y * dt
		b.AngularVelocity.Z += angularAcceleration.Z * dt
		var linearFactor, angularFactor T = 1 + b.LinearDamping*dt, 1 + b.AngularDamping*dt
		b.Velocity.Mul(1 / linearFactor)
		b.AngularVelocity.Mul(1 / angularFactor)
	}
	b.Position.X += b.Velocity.X * dt
	b.Position.Y += b.Velocity.Y * dt
	b.Position.Z += b.Velocity.Z * dt
	b.Orientation.Integrate(b.AngularVelocity, dt)
}

// updateSleeping deactivates sufficiently still dynamic bodies.
func (b *Body3[T]) updateSleeping(config WorldConfig[T], dt T) {
	if !config.EnableSleeping || b.Type != DynamicBody || b.Sleeping {
		return
	}
	if b.Velocity.SquaredLength() > config.SleepLinearThreshold*config.SleepLinearThreshold || b.AngularVelocity.SquaredLength() > config.SleepAngularThreshold*config.SleepAngularThreshold {
		b.sleepTime = 0
		return
	}
	if b.sleepTime += dt; b.sleepTime >= config.SleepTime {
		b.Sleep()
	}
}

// syncShape applies body position and orientation to its collider.
func (b *Body3[T]) syncShape() { b.Shape.Transform(b.Position, b.Orientation) }

// applyInverseInertia applies the local diagonal inverse inertia tensor in world space.
func (b *Body3[T]) applyInverseInertia(value vector.Vec3[T]) (result vector.Vec3[T]) {
	if b.FixedRotation || b.Type != DynamicBody {
		return
	}
	result = b.Orientation.InverseRotate(value)
	var local vector.Vec3[T] = result
	result = vector.Vec3[T]{X: b.inverseInertiaTensor[0]*local.X + b.inverseInertiaTensor[1]*local.Y + b.inverseInertiaTensor[2]*local.Z, Y: b.inverseInertiaTensor[3]*local.X + b.inverseInertiaTensor[4]*local.Y + b.inverseInertiaTensor[5]*local.Z, Z: b.inverseInertiaTensor[6]*local.X + b.inverseInertiaTensor[7]*local.Y + b.inverseInertiaTensor[8]*local.Z}
	result = b.Orientation.Rotate(result)
	return
}

func invertMatrix3[T constraints.Float](matrix [9]T) (inverse [9]T) {
	var determinant T = matrix[0]*(matrix[4]*matrix[8]-matrix[5]*matrix[7]) - matrix[1]*(matrix[3]*matrix[8]-matrix[5]*matrix[6]) + matrix[2]*(matrix[3]*matrix[7]-matrix[4]*matrix[6])
	if determinant == 0 {
		return
	}
	var scale T = 1 / determinant
	inverse = [9]T{(matrix[4]*matrix[8] - matrix[5]*matrix[7]) * scale, (matrix[2]*matrix[7] - matrix[1]*matrix[8]) * scale, (matrix[1]*matrix[5] - matrix[2]*matrix[4]) * scale, (matrix[5]*matrix[6] - matrix[3]*matrix[8]) * scale, (matrix[0]*matrix[8] - matrix[2]*matrix[6]) * scale, (matrix[2]*matrix[3] - matrix[0]*matrix[5]) * scale, (matrix[3]*matrix[7] - matrix[4]*matrix[6]) * scale, (matrix[1]*matrix[6] - matrix[0]*matrix[7]) * scale, (matrix[0]*matrix[4] - matrix[1]*matrix[3]) * scale}
	return
}

// buildContacts rebuilds broad- and narrow-phase contacts for the current step.
func (w *World3[T]) buildContacts() {
	w.Contacts = w.Contacts[:0]
	w.generation++
	w.rebuildSpatialHash()
	for _, first := range w.bodyOrder {
		if first.Disabled || first.Type == StaticBody || first.Sleeping {
			continue
		}
		w.candidates = w.spatialHash.RetrieveInto(w.candidates[:0], first.GetAABB())
		for _, second := range w.candidates {
			if second.ID == first.ID || second.Disabled || second.Type != StaticBody && !second.Sleeping && second.ID < first.ID || first.Type != DynamicBody && second.Type != DynamicBody || !filtersCollide(first.Filter, second.Filter) {
				continue
			}
			var bodyA, bodyB *Body3[T] = first, second
			if bodyA.ID > bodyB.ID {
				bodyA, bodyB = bodyB, bodyA
			}
			var contacts []Contact3[T] = collideBodyManifold3(bodyA, bodyB)
			for contactIndex := range contacts {
				var contact Contact3[T] = contacts[contactIndex]
				if second.Sleeping {
					second.Wake()
				}
				contact.manifoldCount = len(contacts)
				var firstAnchor, secondAnchor vector.Vec3[T] = contactLocalAnchor3(contact.First, contact.Point), contactLocalAnchor3(contact.Second, contact.Point)
				contact.pair = contactPair{first: bodyA.ID, second: bodyB.ID, feature: contactFeature3(contact.feature, firstAnchor, secondAnchor)}
				var persistent bool
				if cached, found := w.contactCache[contact.pair]; found && cached.normal.Dot(&contact.Normal) > 0.9 {
					persistent = true
					if w.Config.EnableWarmStarting && anchorsMatch3(cached, firstAnchor, secondAnchor) {
						contact.normalImpulse, contact.tangentImpulse = cached.normalImpulse, cached.tangentImpulse
					}
				}
				if !persistent {
					contact.restitutionBias = contactRestitutionBias3(&contact, w.Config.RestitutionThreshold)
				}
				w.Contacts = append(w.Contacts, contact)
			}
		}
	}
}

// ensureSpatialHash rebuilds the query index after structural or explicit transform changes.
func (w *World3[T]) ensureSpatialHash() {
	if w.spatialHashDirty {
		w.rebuildSpatialHash()
	}
}

// rebuildSpatialHash refreshes the broad-phase index from current body bounds.
func (w *World3[T]) rebuildSpatialHash() {
	w.spatialHash.Clear()
	for _, body := range w.bodyOrder {
		if !body.Disabled {
			w.spatialHash.Insert(body)
		}
	}
	w.spatialHashDirty = false
}

// collideBodyManifold3 retains every polyhedron contact needed to resist artificial pivoting.
func collideBodyManifold3[T constraints.Float](first, second *Body3[T]) (contacts []Contact3[T]) {
	var firstPolyhedron, firstPolyhedronOK = first.Shape.(*Polyhedron3[T])
	var secondPolyhedron, secondPolyhedronOK = second.Shape.(*Polyhedron3[T])
	if firstPolyhedronOK && secondPolyhedronOK {
		if contacts = collideConvexPolyhedra3(first, second, firstPolyhedron.Polyhedron, secondPolyhedron.Polyhedron); contacts != nil {
			return
		}
		var manifold poly.PolyhedronManifold[T] = poly.GetPolyhedronContactManifold(firstPolyhedron.Polyhedron, secondPolyhedron.Polyhedron)
		for index, manifoldContact := range manifold.Contacts {
			contacts = append(contacts, Contact3[T]{First: first, Second: second, Point: manifoldContact.Point, Normal: manifoldContact.Normal, Penetration: manifoldContact.Penetration, feature: uint32(index)})
		}
		return
	}
	var contact Contact3[T]
	var hit bool
	if contact, hit = collideBodies3(first, second); hit {
		contacts = append(contacts, contact)
	}
	return
}

// collideConvexPolyhedra3 creates a coherent SAT manifold, returning nil for concave inputs.
func collideConvexPolyhedra3[T constraints.Float](firstBody, secondBody *Body3[T], first, second *poly.Polyhedron[T]) (contacts []Contact3[T]) {
	if !polyhedronIsConvex3(first) || !polyhedronIsConvex3(second) {
		return nil
	}
	var (
		normal       vector.Vec3[T]
		penetration  T              = T(math.Inf(1))
		firstCenter  vector.Vec3[T] = first.Centroid()
		secondCenter vector.Vec3[T] = second.Centroid()
	)
	for _, source := range []*poly.Polyhedron[T]{first, second} {
		for triangleIndex := range source.Triangles {
			var axis vector.Vec3[T]
			var ok bool
			if axis, ok = source.FaceNormal(triangleIndex); !ok {
				continue
			}
			if !satAxis3(first, second, axis, firstCenter, secondCenter, &normal, &penetration) {
				return []Contact3[T]{}
			}
		}
	}
	var firstEdges [][2]vector.Vec3[T] = polyhedronEdges3(first)
	var secondEdges [][2]vector.Vec3[T] = polyhedronEdges3(second)
	for _, firstEdge := range firstEdges {
		var firstDirection vector.Vec3[T] = firstEdge[1]
		firstDirection.Sub(&firstEdge[0])
		for _, secondEdge := range secondEdges {
			var axis vector.Vec3[T] = secondEdge[1]
			axis.Sub(&secondEdge[0])
			firstAxis := firstDirection
			firstAxis.Cross(&axis)
			if firstAxis.SquaredLength() <= 1e-12 {
				continue
			}
			firstAxis.Normalize()
			if !satAxis3(first, second, firstAxis, firstCenter, secondCenter, &normal, &penetration) {
				return []Contact3[T]{}
			}
		}
	}
	for index, point := range first.Points {
		if pointInsideConvex3(point, second) {
			contacts = append(contacts, Contact3[T]{First: firstBody, Second: secondBody, Point: point, Normal: normal, Penetration: penetration, feature: uint32(index + 1)})
		}
	}
	for index, point := range second.Points {
		if pointInsideConvex3(point, first) {
			contacts = append(contacts, Contact3[T]{First: firstBody, Second: secondBody, Point: point, Normal: normal, Penetration: penetration, feature: uint32(index+1) | 0x80000000})
		}
	}
	if len(contacts) == 0 {
		var firstSupport, secondSupport vector.Vec3[T] = supportPoint3(first.Points, normal, true), supportPoint3(second.Points, normal, false)
		contacts = append(contacts, Contact3[T]{First: firstBody, Second: secondBody, Point: vector.Vec3[T]{X: (firstSupport.X + secondSupport.X) / 2, Y: (firstSupport.Y + secondSupport.Y) / 2, Z: (firstSupport.Z + secondSupport.Z) / 2}, Normal: normal, Penetration: penetration, feature: 0xffffffff})
	}
	if len(contacts) > 4 {
		contacts = contacts[:4]
	}
	return
}

func satAxis3[T constraints.Float](first, second *poly.Polyhedron[T], axis, firstCenter, secondCenter vector.Vec3[T], normal *vector.Vec3[T], penetration *T) (overlaps bool) {
	axis.Normalize()
	var firstMinimum, firstMaximum T = first.ProjectOnto(&axis)
	var secondMinimum, secondMaximum T = second.ProjectOnto(&axis)
	var overlap T = min(firstMaximum, secondMaximum) - max(firstMinimum, secondMinimum)
	if overlap < 0 {
		return
	}
	if overlap < *penetration {
		var direction vector.Vec3[T] = secondCenter
		direction.Sub(&firstCenter)
		if axis.Dot(&direction) < 0 {
			axis.Mul(-1)
		}
		*normal, *penetration = axis, overlap
	}
	overlaps = true
	return
}

func polyhedronIsConvex3[T constraints.Float](shape *poly.Polyhedron[T]) (convex bool) {
	var center vector.Vec3[T] = shape.Centroid()
	for triangleIndex, triangle := range shape.Triangles {
		var normal vector.Vec3[T]
		var ok bool
		if normal, ok = shape.FaceNormal(triangleIndex); !ok {
			return
		}
		var outward vector.Vec3[T] = shape.Points[triangle.A]
		outward.Sub(&center)
		if normal.Dot(&outward) < 0 {
			normal.Mul(-1)
		}
		var origin vector.Vec3[T] = shape.Points[triangle.A]
		for _, point := range shape.Points {
			var offset vector.Vec3[T] = point
			offset.Sub(&origin)
			if normal.Dot(&offset) > 1e-7 {
				return
			}
		}
	}
	convex = true
	return
}

func pointInsideConvex3[T constraints.Float](point vector.Vec3[T], shape *poly.Polyhedron[T]) (inside bool) {
	var center vector.Vec3[T] = shape.Centroid()
	for triangleIndex, triangle := range shape.Triangles {
		var normal vector.Vec3[T]
		var ok bool
		if normal, ok = shape.FaceNormal(triangleIndex); !ok {
			return
		}
		var outward vector.Vec3[T] = shape.Points[triangle.A]
		outward.Sub(&center)
		if normal.Dot(&outward) < 0 {
			normal.Mul(-1)
		}
		var offset vector.Vec3[T] = point
		offset.Sub(&shape.Points[triangle.A])
		if normal.Dot(&offset) > 1e-7 {
			return
		}
	}
	inside = true
	return
}

func polyhedronEdges3[T constraints.Float](shape *poly.Polyhedron[T]) (edges [][2]vector.Vec3[T]) {
	var uses map[[2]int][]int = make(map[[2]int][]int, len(shape.Triangles)*3/2)
	for triangleIndex, triangle := range shape.Triangles {
		for _, edge := range [][2]int{{triangle.A, triangle.B}, {triangle.B, triangle.C}, {triangle.C, triangle.A}} {
			if edge[0] > edge[1] {
				edge[0], edge[1] = edge[1], edge[0]
			}
			uses[edge] = append(uses[edge], triangleIndex)
		}
	}
	for edge, triangles := range uses {
		if len(triangles) == 2 {
			var firstNormal, secondNormal vector.Vec3[T]
			var firstOK, secondOK bool
			firstNormal, firstOK = shape.FaceNormal(triangles[0])
			secondNormal, secondOK = shape.FaceNormal(triangles[1])
			if firstOK && secondOK && math.Abs(float64(firstNormal.Dot(&secondNormal))) >= 1-1e-7 {
				continue
			}
		}
		edges = append(edges, [2]vector.Vec3[T]{shape.Points[edge[0]], shape.Points[edge[1]]})
	}
	return
}

func supportPoint3[T constraints.Float](points []vector.Vec3[T], axis vector.Vec3[T], maximum bool) (support vector.Vec3[T]) {
	var extreme T = T(math.Inf(1))
	if maximum {
		extreme = T(math.Inf(-1))
	}
	for _, point := range points {
		var projection T = point.Dot(&axis)
		if maximum && projection > extreme || !maximum && projection < extreme {
			extreme, support = projection, point
		}
	}
	return
}

// storeContactCache retains solved impulses and removes contacts absent this step.
func (w *World3[T]) storeContactCache() {
	for i := range w.Contacts {
		var firstAnchor, secondAnchor vector.Vec3[T] = contactLocalAnchor3(w.Contacts[i].First, w.Contacts[i].Point), contactLocalAnchor3(w.Contacts[i].Second, w.Contacts[i].Point)
		w.contactCache[w.Contacts[i].pair] = cachedContact3[T]{normalImpulse: w.Contacts[i].normalImpulse, tangentImpulse: w.Contacts[i].tangentImpulse, normal: w.Contacts[i].Normal, firstAnchor: firstAnchor, secondAnchor: secondAnchor, generation: w.generation}
	}
	for pair, cached := range w.contactCache {
		if cached.generation != w.generation {
			delete(w.contactCache, pair)
		}
	}
}

func contactLocalAnchor3[T constraints.Float](body *Body3[T], point vector.Vec3[T]) (anchor vector.Vec3[T]) {
	anchor = body.Orientation.InverseRotate(vector.Vec3[T]{X: point.X - body.Position.X, Y: point.Y - body.Position.Y, Z: point.Z - body.Position.Z})
	return
}

func anchorsMatch3[T constraints.Float](cached cachedContact3[T], first, second vector.Vec3[T]) (matches bool) {
	first.Sub(&cached.firstAnchor)
	second.Sub(&cached.secondAnchor)
	matches = first.SquaredLength() < .0025 && second.SquaredLength() < .0025
	return
}

// contactRestitutionBias3 captures the desired bounce velocity before iterative solving.
func contactRestitutionBias3[T constraints.Float](contact *Contact3[T], threshold T) (bias T) {
	var (
		rA       vector.Vec3[T] = vector.Vec3[T]{X: contact.Point.X - contact.First.Position.X, Y: contact.Point.Y - contact.First.Position.Y, Z: contact.Point.Z - contact.First.Position.Z}
		rB       vector.Vec3[T] = vector.Vec3[T]{X: contact.Point.X - contact.Second.Position.X, Y: contact.Point.Y - contact.Second.Position.Y, Z: contact.Point.Z - contact.Second.Position.Z}
		angularA vector.Vec3[T] = contact.First.AngularVelocity
		angularB vector.Vec3[T] = contact.Second.AngularVelocity
	)
	angularA.Cross(&rA)
	angularB.Cross(&rB)
	var relative vector.Vec3[T] = vector.Vec3[T]{X: contact.Second.Velocity.X + angularB.X - contact.First.Velocity.X - angularA.X, Y: contact.Second.Velocity.Y + angularB.Y - contact.First.Velocity.Y - angularA.Y, Z: contact.Second.Velocity.Z + angularB.Z - contact.First.Velocity.Z - angularA.Z}
	var velocity T = relative.Dot(&contact.Normal)
	if velocity < -threshold {
		bias = -min(contact.First.Material.Restitution, contact.Second.Material.Restitution) * velocity
	}
	return
}

func contactFeature3[T constraints.Float](source uint32, first, second vector.Vec3[T]) (feature uint32) {
	if source != 0 {
		feature = source
		return
	}
	feature = 2166136261 ^ source
	for _, value := range []T{first.X, first.Y, first.Z, second.X, second.Y, second.Z} {
		feature ^= uint32(int32(math.Round(float64(value) * 1000))) // #nosec G115 -- signed coordinate bits are intentionally folded into the FNV-style hash.
		feature *= 16777619
	}
	return
}

// collideBodies3 dispatches supported three-dimensional shape pairs.
func collideBodies3[T constraints.Float](first, second *Body3[T]) (contact Contact3[T], hit bool) {
	contact.First, contact.Second = first, second
	var firstSphere, firstSphereOK = first.Shape.(*Sphere3[T])
	var secondSphere, secondSphereOK = second.Shape.(*Sphere3[T])
	if firstSphereOK && secondSphereOK {
		contact.Point, contact.Normal, contact.Penetration, hit = collideSpheres3(firstSphere, secondSphere)
		return
	}
	var firstPolyhedron, firstPolyhedronOK = first.Shape.(*Polyhedron3[T])
	var secondPolyhedron, secondPolyhedronOK = second.Shape.(*Polyhedron3[T])
	if firstPolyhedronOK && secondPolyhedronOK {
		var manifold poly.PolyhedronManifold[T] = poly.GetPolyhedronContactManifold(firstPolyhedron.Polyhedron, secondPolyhedron.Polyhedron)
		for i := range manifold.Contacts {
			if !hit || manifold.Contacts[i].Penetration > contact.Penetration {
				contact.Point, contact.Normal, contact.Penetration, hit = manifold.Contacts[i].Point, manifold.Contacts[i].Normal, manifold.Contacts[i].Penetration, true
			}
		}
		return
	}
	if firstSphereOK && secondPolyhedronOK {
		contact.Point, contact.Normal, contact.Penetration, hit = collideSpherePolyhedron3(firstSphere, secondPolyhedron)
		return
	}
	if firstPolyhedronOK && secondSphereOK {
		contact.Point, contact.Normal, contact.Penetration, hit = collideSpherePolyhedron3(secondSphere, firstPolyhedron)
		contact.Normal.Mul(-1)
	}
	return
}

// collideSpheres3 computes an analytic sphere contact.
func collideSpheres3[T constraints.Float](first, second *Sphere3[T]) (point, normal vector.Vec3[T], penetration T, hit bool) {
	var dx, dy, dz T = second.Position.X - first.Position.X, second.Position.Y - first.Position.Y, second.Position.Z - first.Position.Z
	var radius, squared T = first.Radius + second.Radius, dx*dx + dy*dy + dz*dz
	if squared > radius*radius {
		return
	}
	var distance T = T(math.Sqrt(float64(squared)))
	if distance == 0 {
		normal.X = 1
	} else {
		normal = vector.Vec3[T]{X: dx / distance, Y: dy / distance, Z: dz / distance}
	}
	penetration = radius - distance
	point = vector.Vec3[T]{X: first.Position.X + normal.X*(first.Radius-penetration/2), Y: first.Position.Y + normal.Y*(first.Radius-penetration/2), Z: first.Position.Z + normal.Z*(first.Radius-penetration/2)}
	hit = true
	return
}

// collideSpherePolyhedron3 computes the closest surface contact for a sphere and mesh.
func collideSpherePolyhedron3[T constraints.Float](sphere *Sphere3[T], shape *Polyhedron3[T]) (point, normal vector.Vec3[T], penetration T, hit bool) {
	point = shape.Polyhedron.ClosestPoint(&sphere.Position)
	var dx, dy, dz T = point.X - sphere.Position.X, point.Y - sphere.Position.Y, point.Z - sphere.Position.Z
	var distance T = T(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
	var inside bool = shape.Polyhedron.PointIsInside(&sphere.Position)
	if !inside && distance > sphere.Radius {
		return
	}
	if distance == 0 {
		normal.X = 1
	} else if inside {
		normal = vector.Vec3[T]{X: -dx / distance, Y: -dy / distance, Z: -dz / distance}
	} else {
		normal = vector.Vec3[T]{X: dx / distance, Y: dy / distance, Z: dz / distance}
	}
	if inside {
		penetration = sphere.Radius + distance
	} else {
		penetration = sphere.Radius - distance
	}
	hit = true
	return
}

// resolveVelocity3 applies normal and friction impulses for one contact.
func resolveVelocity3[T constraints.Float](contact *Contact3[T]) {
	if contact.First.Sensor || contact.Second.Sensor {
		return
	}
	var first, second *Body3[T] = contact.First, contact.Second
	var rA, rB vector.Vec3[T] = vector.Vec3[T]{X: contact.Point.X - first.Position.X, Y: contact.Point.Y - first.Position.Y, Z: contact.Point.Z - first.Position.Z}, vector.Vec3[T]{X: contact.Point.X - second.Position.X, Y: contact.Point.Y - second.Position.Y, Z: contact.Point.Z - second.Position.Z}
	var angularA, angularB vector.Vec3[T] = first.AngularVelocity, second.AngularVelocity
	angularA.Cross(&rA)
	angularB.Cross(&rB)
	var relative vector.Vec3[T] = vector.Vec3[T]{X: second.Velocity.X + angularB.X - first.Velocity.X - angularA.X, Y: second.Velocity.Y + angularB.Y - first.Velocity.Y - angularA.Y, Z: second.Velocity.Z + angularB.Z - first.Velocity.Z - angularA.Z}
	var velocityAlongNormal T = relative.Dot(&contact.Normal)
	if velocityAlongNormal >= contact.restitutionBias && contact.normalImpulse == 0 {
		return
	}
	var rACrossN, rBCrossN vector.Vec3[T] = rA, rB
	rACrossN.Cross(&contact.Normal)
	rBCrossN.Cross(&contact.Normal)
	var firstAngularResponse, secondAngularResponse vector.Vec3[T] = first.applyInverseInertia(rACrossN), second.applyInverseInertia(rBCrossN)
	var angularTerm T = rACrossN.Dot(&firstAngularResponse) + rBCrossN.Dot(&secondAngularResponse)
	var denominator T = first.inverseMass + second.inverseMass + angularTerm
	if denominator == 0 {
		return
	}
	var magnitude T = -(velocityAlongNormal - contact.restitutionBias) / denominator
	var previousNormalImpulse T = contact.normalImpulse
	contact.normalImpulse = max(previousNormalImpulse+magnitude, 0)
	magnitude = contact.normalImpulse - previousNormalImpulse
	applyContactImpulse3(first, second, rA, rB, vector.Vec3[T]{X: contact.Normal.X * magnitude, Y: contact.Normal.Y * magnitude, Z: contact.Normal.Z * magnitude})

	angularA, angularB = first.AngularVelocity, second.AngularVelocity
	angularA.Cross(&rA)
	angularB.Cross(&rB)
	relative = vector.Vec3[T]{X: second.Velocity.X + angularB.X - first.Velocity.X - angularA.X, Y: second.Velocity.Y + angularB.Y - first.Velocity.Y - angularA.Y, Z: second.Velocity.Z + angularB.Z - first.Velocity.Z - angularA.Z}
	var normalVelocity T = relative.Dot(&contact.Normal)
	var tangent vector.Vec3[T] = vector.Vec3[T]{
		X: relative.X - contact.Normal.X*normalVelocity,
		Y: relative.Y - contact.Normal.Y*normalVelocity,
		Z: relative.Z - contact.Normal.Z*normalVelocity,
	}
	if tangent.SquaredLength() == 0 {
		return
	}
	tangent.Normalize()
	var rACrossTangent, rBCrossTangent vector.Vec3[T] = rA, rB
	rACrossTangent.Cross(&tangent)
	rBCrossTangent.Cross(&tangent)
	firstAngularResponse, secondAngularResponse = first.applyInverseInertia(rACrossTangent), second.applyInverseInertia(rBCrossTangent)
	var tangentDenominator T = first.inverseMass + second.inverseMass + rACrossTangent.Dot(&firstAngularResponse) + rBCrossTangent.Dot(&secondAngularResponse)
	if tangentDenominator == 0 {
		return
	}
	var frictionMagnitude T = -relative.Dot(&tangent) / tangentDenominator
	var previousTangentImpulse vector.Vec3[T] = contact.tangentImpulse
	contact.tangentImpulse.X += tangent.X * frictionMagnitude
	contact.tangentImpulse.Y += tangent.Y * frictionMagnitude
	contact.tangentImpulse.Z += tangent.Z * frictionMagnitude
	var staticFriction T = T(math.Sqrt(float64(first.Material.StaticFriction * second.Material.StaticFriction)))
	var tangentImpulseLength T = contact.tangentImpulse.Length()
	if tangentImpulseLength > contact.normalImpulse*staticFriction {
		var maximum T = contact.normalImpulse * T(math.Sqrt(float64(first.Material.DynamicFriction*second.Material.DynamicFriction)))
		if tangentImpulseLength != 0 {
			contact.tangentImpulse.Mul(maximum / tangentImpulseLength)
		}
	}
	applyContactImpulse3(first, second, rA, rB, vector.Vec3[T]{X: contact.tangentImpulse.X - previousTangentImpulse.X, Y: contact.tangentImpulse.Y - previousTangentImpulse.Y, Z: contact.tangentImpulse.Z - previousTangentImpulse.Z})
}

// warmStart3 reapplies the previous step's accumulated impulses.
func warmStart3[T constraints.Float](contact *Contact3[T]) {
	if contact.First.Sensor || contact.Second.Sensor || contact.normalImpulse == 0 && contact.tangentImpulse.SquaredLength() == 0 {
		return
	}
	var (
		rA      vector.Vec3[T] = vector.Vec3[T]{X: contact.Point.X - contact.First.Position.X, Y: contact.Point.Y - contact.First.Position.Y, Z: contact.Point.Z - contact.First.Position.Z}
		rB      vector.Vec3[T] = vector.Vec3[T]{X: contact.Point.X - contact.Second.Position.X, Y: contact.Point.Y - contact.Second.Position.Y, Z: contact.Point.Z - contact.Second.Position.Z}
		impulse vector.Vec3[T] = vector.Vec3[T]{X: contact.Normal.X*contact.normalImpulse + contact.tangentImpulse.X, Y: contact.Normal.Y*contact.normalImpulse + contact.tangentImpulse.Y, Z: contact.Normal.Z*contact.normalImpulse + contact.tangentImpulse.Z}
	)
	applyContactImpulse3(contact.First, contact.Second, rA, rB, impulse)
}

// applyContactImpulse3 applies an equal and opposite impulse to a body pair.
func applyContactImpulse3[T constraints.Float](first, second *Body3[T], rA, rB, impulse vector.Vec3[T]) {
	if first.Type == DynamicBody {
		first.Velocity.X -= impulse.X * first.inverseMass
		first.Velocity.Y -= impulse.Y * first.inverseMass
		first.Velocity.Z -= impulse.Z * first.inverseMass
	}
	if second.Type == DynamicBody {
		second.Velocity.X += impulse.X * second.inverseMass
		second.Velocity.Y += impulse.Y * second.inverseMass
		second.Velocity.Z += impulse.Z * second.inverseMass
	}
	var torqueA, torqueB vector.Vec3[T] = rA, rB
	torqueA.Cross(&impulse)
	torqueB.Cross(&impulse)
	var angularA, angularB vector.Vec3[T] = first.applyInverseInertia(torqueA), second.applyInverseInertia(torqueB)
	if first.Type == DynamicBody {
		first.AngularVelocity.Sub(&angularA)
	}
	if second.Type == DynamicBody {
		second.AngularVelocity.Add(&angularB)
	}
}

// resolvePosition3 applies inverse-mass-weighted positional correction.
func resolvePosition3[T constraints.Float](contact *Contact3[T], config WorldConfig[T]) {
	if contact.First.Sensor || contact.Second.Sensor {
		return
	}
	var first, second *Body3[T] = contact.First, contact.Second
	var rA, rB vector.Vec3[T] = vector.Vec3[T]{X: contact.Point.X - first.Position.X, Y: contact.Point.Y - first.Position.Y, Z: contact.Point.Z - first.Position.Z}, vector.Vec3[T]{X: contact.Point.X - second.Position.X, Y: contact.Point.Y - second.Position.Y, Z: contact.Point.Z - second.Position.Z}
	var rACrossNormal, rBCrossNormal vector.Vec3[T] = rA, rB
	rACrossNormal.Cross(&contact.Normal)
	rBCrossNormal.Cross(&contact.Normal)
	var firstAngularResponse, secondAngularResponse vector.Vec3[T] = first.applyInverseInertia(rACrossNormal), second.applyInverseInertia(rBCrossNormal)
	var denominator T = first.inverseMass + second.inverseMass + rACrossNormal.Dot(&firstAngularResponse) + rBCrossNormal.Dot(&secondAngularResponse)
	if denominator == 0 {
		return
	}
	var magnitude T = max(contact.Penetration-config.PenetrationSlop, 0) * config.PositionCorrection / T(config.PositionIterations) / denominator
	if contact.manifoldCount > 1 {
		magnitude /= T(contact.manifoldCount)
	}
	if first.Type == DynamicBody {
		first.Position.X -= contact.Normal.X * magnitude * first.inverseMass
		first.Position.Y -= contact.Normal.Y * magnitude * first.inverseMass
		first.Position.Z -= contact.Normal.Z * magnitude * first.inverseMass
		firstAngularResponse.Mul(-magnitude)
		first.Orientation.ApplyRotationVector(firstAngularResponse)
	}
	if second.Type == DynamicBody {
		second.Position.X += contact.Normal.X * magnitude * second.inverseMass
		second.Position.Y += contact.Normal.Y * magnitude * second.inverseMass
		second.Position.Z += contact.Normal.Z * magnitude * second.inverseMass
		secondAngularResponse.Mul(magnitude)
		second.Orientation.ApplyRotationVector(secondAngularResponse)
	}
}
