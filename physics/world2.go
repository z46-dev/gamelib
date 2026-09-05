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
	contactPair struct {
		first, second BodyID
		feature       uint32
	}

	cachedContact2[T constraints.Float] struct {
		normalImpulse, tangentImpulse T
		normalX, normalY              T
		firstAnchorX, firstAnchorY    T
		secondAnchorX, secondAnchorY  T
		generation                    uint64
	}

	// Body2Config describes a two-dimensional rigid body at creation time.
	Body2Config[T constraints.Float] struct {
		Type                                                        BodyType
		Shape                                                       Shape2[T]
		Position, Velocity                                          vector.Vec2[T]
		Rotation, AngularVelocity                                   T
		Mass                                                        T
		Material                                                    Material[T]
		Filter                                                      CollisionFilter
		GravityScale, LinearDamping                                 T
		AngularDamping                                              T
		Sensor, FixedRotation, DisableGravity, Disabled, Continuous bool
	}

	// Body2 stores the mutable state of one two-dimensional rigid body.
	Body2[T constraints.Float] struct {
		ID                                          BodyID
		Type                                        BodyType
		Shape                                       Shape2[T]
		Position, Velocity, Force                   vector.Vec2[T]
		Rotation, AngularVelocity, Torque           T
		Material                                    Material[T]
		Filter                                      CollisionFilter
		GravityScale, LinearDamping                 T
		AngularDamping                              T
		Sensor, FixedRotation, Disabled, Continuous bool
		Sleeping                                    bool

		mass, inverseMass, inertia, inverseInertia T
		sleepTime                                  T
		islandIndex                                int
		world                                      *World2[T]
	}

	// Contact2 describes one active two-dimensional body contact.
	Contact2[T constraints.Float] struct {
		First, Second *Body2[T]
		Point, Normal vector.Vec2[T]
		Penetration   T

		pair                          contactPair
		manifoldCount                 int
		normalImpulse, tangentImpulse T
		restitutionBias               T
	}

	// World2 owns and advances a collection of two-dimensional rigid bodies.
	World2[T constraints.Float] struct {
		Config   WorldConfig[T]
		Contacts []Contact2[T]

		nextID                                 BodyID
		bodies                                 map[BodyID]*Body2[T]
		bodyOrder                              []*Body2[T]
		spatialHash                            *hshg.SpatialHash2[*Body2[T], T]
		spatialHashDirty                       bool
		candidates                             []*Body2[T]
		contactCache                           map[contactPair]cachedContact2[T]
		generation                             uint64
		nextConstraintID                       ConstraintID
		distanceConstraints                    []*DistanceConstraint2[T]
		angularConstraints                     []*AngularConstraint2[T]
		ropes                                  []*Rope2[T]
		areaConstraints                        []*AreaConstraint2[T]
		islandParent, islandRank, islandLookup []int
		islands                                []island2[T]
	}
)

// NewWorld2 creates an empty two-dimensional physics world.
func NewWorld2[T constraints.Float](config WorldConfig[T]) (world *World2[T]) {
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
	var spatialHash *hshg.SpatialHash2[*Body2[T], T]
	if len(config.SpatialHash.Levels) == 0 {
		spatialHash = hshg.NewSpatialHash2[*Body2[T], T]()
	} else {
		spatialHash = hshg.NewSpatialHash2[*Body2[T], T](hshg.WithSpatialHash2Config(config.SpatialHash))
	}
	world = &World2[T]{Config: config, nextID: 1, bodies: make(map[BodyID]*Body2[T]), spatialHash: spatialHash, spatialHashDirty: true, contactCache: make(map[contactPair]cachedContact2[T])}
	return
}

// AddBody creates and registers a body, cloning its shape for independent transforms.
func (w *World2[T]) AddBody(config Body2Config[T]) (body *Body2[T], err error) {
	if config.Shape == nil {
		err = fmt.Errorf("physics: Body2 requires a shape")
		return
	}
	if config.Type > DynamicBody {
		err = fmt.Errorf("physics: invalid Body2 type %d", config.Type)
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

	body = &Body2[T]{
		ID: w.nextID, Type: config.Type, Shape: config.Shape.Clone(), Position: config.Position, Velocity: config.Velocity,
		Rotation: config.Rotation, AngularVelocity: config.AngularVelocity, Material: clampMaterial(config.Material), Filter: config.Filter,
		GravityScale: config.GravityScale, LinearDamping: max(config.LinearDamping, 0), AngularDamping: max(config.AngularDamping, 0),
		Sensor: config.Sensor, FixedRotation: config.FixedRotation, Disabled: config.Disabled, Continuous: config.Continuous,
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
func (w *World2[T]) RemoveBody(id BodyID) (removed bool) {
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
	for i := len(w.areaConstraints) - 1; i >= 0; i-- {
		var c *AreaConstraint2[T] = w.areaConstraints[i]
		if c.First.ID == id || c.Second.ID == id || c.Third.ID == id {
			w.areaConstraints = append(w.areaConstraints[:i], w.areaConstraints[i+1:]...)
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
func (w *World2[T]) Body(id BodyID) (body *Body2[T], found bool) {
	body, found = w.bodies[id]
	return
}

// Bodies returns a detached list of bodies in deterministic creation order.
func (w *World2[T]) Bodies() (bodies []*Body2[T]) {
	bodies = append([]*Body2[T](nil), w.bodyOrder...)
	return
}

// BodiesInAABB returns enabled bodies whose bounds intersect the supplied bounds.
func (w *World2[T]) BodiesInAABB(bounds *hshg.AABB2[T]) (bodies []*Body2[T]) {
	bodies = w.BodiesInAABBInto(nil, bounds)
	return
}

// BodiesInAABBInto replaces bodies with enabled bodies whose bounds intersect the supplied bounds.
func (w *World2[T]) BodiesInAABBInto(bodies []*Body2[T], bounds *hshg.AABB2[T]) (result []*Body2[T]) {
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

// BodiesInRadius returns enabled bodies whose bounds intersect a circle around center.
func (w *World2[T]) BodiesInRadius(center vector.Vec2[T], radius T) (bodies []*Body2[T]) {
	bodies = w.BodiesInRadiusInto(nil, center, radius)
	return
}

// BodiesInRadiusInto replaces bodies with enabled bodies whose bounds intersect a circle around center.
func (w *World2[T]) BodiesInRadiusInto(bodies []*Body2[T], center vector.Vec2[T], radius T) (result []*Body2[T]) {
	w.ensureSpatialHash()
	radius = max(radius, 0)
	result = w.spatialHash.RetrieveAroundInto(bodies, center.X, center.Y, radius)
	var radiusSquared T = radius * radius
	var count int
	for _, body := range result {
		if body.Disabled {
			continue
		}
		var bounds *hshg.AABB2[T] = body.GetAABB()
		var (
			closestX T = min(max(center.X, bounds.X1), bounds.X2)
			closestY T = min(max(center.Y, bounds.Y1), bounds.Y2)
			deltaX   T = center.X - closestX
			deltaY   T = center.Y - closestY
		)
		if deltaX*deltaX+deltaY*deltaY <= radiusSquared {
			result[count] = body
			count++
		}
	}
	result = result[:count]
	return
}

// GetAABB returns the body's shape bounds for SpatialHash2.
func (b *Body2[T]) GetAABB() (aabb *hshg.AABB2[T]) {
	aabb = b.Shape.GetAABB()
	return
}

// Mass returns the body's finite mass, or zero for non-dynamic bodies.
func (b *Body2[T]) Mass() (mass T) {
	mass = b.mass
	return
}

// ApplyForce accumulates a world-space force for the next step.
func (b *Body2[T]) ApplyForce(force vector.Vec2[T]) {
	if b.Type == DynamicBody && !b.Disabled {
		b.Wake()
		b.Force.X += force.X
		b.Force.Y += force.Y
	}
}

// ApplyForceAtPoint accumulates force and torque at a world-space point.
func (b *Body2[T]) ApplyForceAtPoint(force, point vector.Vec2[T]) {
	b.ApplyForce(force)
	if b.Type == DynamicBody && !b.FixedRotation && !b.Disabled {
		b.Torque += (point.X-b.Position.X)*force.Y - (point.Y-b.Position.Y)*force.X
	}
}

// ApplyImpulse immediately changes linear velocity at the center of mass.
func (b *Body2[T]) ApplyImpulse(impulse vector.Vec2[T]) {
	if b.Type == DynamicBody && !b.Disabled {
		b.Wake()
		b.Velocity.X += impulse.X * b.inverseMass
		b.Velocity.Y += impulse.Y * b.inverseMass
	}
}

// ApplyImpulseAtPoint immediately changes linear and angular velocity.
func (b *Body2[T]) ApplyImpulseAtPoint(impulse, point vector.Vec2[T]) {
	b.ApplyImpulse(impulse)
	if b.Type == DynamicBody && !b.FixedRotation && !b.Disabled {
		b.AngularVelocity += ((point.X-b.Position.X)*impulse.Y - (point.Y-b.Position.Y)*impulse.X) * b.inverseInertia
	}
}

// SetTransform teleports a body and synchronizes its collider.
func (b *Body2[T]) SetTransform(position vector.Vec2[T], rotation T) {
	b.Position, b.Rotation = position, rotation
	b.Wake()
	b.syncShape()
	if b.world != nil {
		b.world.spatialHashDirty = true
	}
}

// Wake returns a dynamic body to active simulation.
func (b *Body2[T]) Wake() {
	if b.Type == DynamicBody {
		b.Sleeping = false
		b.sleepTime = 0
	}
}

// Sleep immediately deactivates a dynamic body until it is disturbed.
func (b *Body2[T]) Sleep() {
	if b.Type == DynamicBody {
		b.Sleeping = true
		b.Velocity, b.Force = vector.Vec2[T]{}, vector.Vec2[T]{}
		b.AngularVelocity, b.Torque = 0, 0
	}
}

// Step advances forces, contacts, impulses, and positions by dt seconds.
func (w *World2[T]) Step(dt T) {
	if dt <= 0 {
		return
	}
	var substeps int = w.ccdSubsteps(dt)
	var substepDT T = dt / T(substeps)
	for range substeps {
		w.stepDiscrete(substepDT)
	}
	for _, body := range w.bodyOrder {
		body.Force = vector.Vec2[T]{}
		body.Torque = 0
	}
	w.rebuildSpatialHash()
}

// stepDiscrete advances one collision-safe substep without clearing forces.
func (w *World2[T]) stepDiscrete(dt T) {
	for _, rope := range w.ropes {
		rope.UpdateReconnection()
	}
	for _, body := range w.bodyOrder {
		body.integrate(w.Config, dt)
		body.syncShape()
	}
	w.buildContacts()
	var islands []island2[T]
	if len(w.Contacts)+len(w.distanceConstraints)+len(w.angularConstraints)+len(w.areaConstraints) > 0 {
		islands = w.buildIslands()
	}
	for _, constraint := range w.distanceConstraints {
		constraint.lambda = 0
	}
	for _, constraint := range w.angularConstraints {
		constraint.lambda = 0
	}
	for _, constraint := range w.areaConstraints {
		constraint.lambda = 0
	}
	if w.Config.EnableWarmStarting {
		for i := range w.Contacts {
			warmStart2(&w.Contacts[i])
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

func (w *World2[T]) ccdSubsteps(dt T) (substeps int) {
	substeps = max(1, int(math.Ceil(float64(dt/w.Config.MaxStepDelta))))
	if w.Config.CCDMotionThreshold <= 0 {
		return
	}
	for _, body := range w.bodyOrder {
		if body.Type == StaticBody || body.Disabled || body.Sleeping || !w.Config.EnableCCD && !body.Continuous {
			continue
		}
		var bounds *hshg.AABB2[T] = body.GetAABB()
		var (
			width, height T = bounds.X2 - bounds.X1, bounds.Y2 - bounds.Y1
			limit         T = min(w.Config.CCDMotionThreshold, min(width, height)/2)
			sweepRadius   T = T(math.Hypot(float64(width), float64(height))) / 2
		)
		if limit <= 0 {
			continue
		}
		var angularSpeed T = body.AngularVelocity
		if angularSpeed < 0 {
			angularSpeed = -angularSpeed
		}
		var (
			linearSpeed         T = body.Velocity.Length()
			linearAcceleration  vector.Vec2[T]
			angularAcceleration T
		)
		if body.Type == DynamicBody {
			linearAcceleration = vector.Vec2[T]{X: w.Config.GravityX*body.GravityScale + body.Force.X*body.inverseMass, Y: w.Config.GravityY*body.GravityScale + body.Force.Y*body.inverseMass}
			angularAcceleration = body.Torque * body.inverseInertia
		}
		var endLinearSpeed T = linearSpeed + linearAcceleration.Length()*dt
		var endAngularSpeed T = angularSpeed
		if angularAcceleration < 0 {
			angularAcceleration = -angularAcceleration
		}
		endAngularSpeed += angularAcceleration * dt
		var required int = int(math.Ceil(float64((max(linearSpeed, endLinearSpeed) + max(angularSpeed, endAngularSpeed)*sweepRadius) * dt / limit)))
		if required > substeps {
			substeps = required
		}
	}
	return
}

// recalculateMass refreshes inverse mass and inertia from shape and material.
func (b *Body2[T]) recalculateMass(override T) {
	if b.Type != DynamicBody {
		return
	}
	b.mass = override
	if b.mass <= 0 {
		b.mass = b.Shape.Area() * b.Material.Density
	}
	if b.mass <= 0 {
		b.mass = 1
	}
	b.inverseMass = 1 / b.mass
	if !b.FixedRotation {
		b.inertia = b.Shape.MomentOfInertia(b.mass)
		if b.inertia > 0 {
			b.inverseInertia = 1 / b.inertia
		}
	}
}

// integrate advances one body's velocities and transform.
func (b *Body2[T]) integrate(config WorldConfig[T], dt T) {
	if b.Disabled || b.Type == StaticBody || b.Sleeping {
		return
	}
	if b.Type == DynamicBody {
		b.Velocity.X += (config.GravityX*b.GravityScale + b.Force.X*b.inverseMass) * dt
		b.Velocity.Y += (config.GravityY*b.GravityScale + b.Force.Y*b.inverseMass) * dt
		b.AngularVelocity += b.Torque * b.inverseInertia * dt
		b.Velocity.X /= 1 + b.LinearDamping*dt
		b.Velocity.Y /= 1 + b.LinearDamping*dt
		b.AngularVelocity /= 1 + b.AngularDamping*dt
	}
	b.Position.X += b.Velocity.X * dt
	b.Position.Y += b.Velocity.Y * dt
	b.Rotation += b.AngularVelocity * dt
}

// updateSleeping deactivates sufficiently still dynamic bodies.
func (b *Body2[T]) updateSleeping(config WorldConfig[T], dt T) {
	if !config.EnableSleeping || b.Type != DynamicBody || b.Sleeping {
		return
	}
	if b.Velocity.SquaredLength() > config.SleepLinearThreshold*config.SleepLinearThreshold || b.AngularVelocity > config.SleepAngularThreshold || b.AngularVelocity < -config.SleepAngularThreshold {
		b.sleepTime = 0
		return
	}
	if b.sleepTime += dt; b.sleepTime >= config.SleepTime {
		b.Sleep()
	}
}

// syncShape applies body position and rotation to its collider.
func (b *Body2[T]) syncShape() {
	b.Shape.Transform(b.Position, b.Rotation)
}

// buildContacts rebuilds broad- and narrow-phase contacts for the current step.
func (w *World2[T]) buildContacts() {
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
			var (
				bodyA, bodyB *Body2[T] = first, second
				contact      Contact2[T]
			)
			if bodyA.ID > bodyB.ID {
				bodyA, bodyB = bodyB, bodyA
			}
			var manifold [2]Contact2[T]
			var contactCount int
			manifold, contactCount = collideBodyManifold2(bodyA, bodyB)
			for contactIndex := range contactCount { // #nosec G602 -- collideBodyManifold2 returns a count bounded by the fixed manifold length.
				contact = manifold[contactIndex]
				contact.manifoldCount = contactCount
				if second.Sleeping {
					second.Wake()
				}
				var firstAnchor, secondAnchor vector.Vec2[T] = contactLocalAnchor2(contact.First, contact.Point), contactLocalAnchor2(contact.Second, contact.Point)
				contact.pair = contactPair{first: bodyA.ID, second: bodyB.ID, feature: contactFeature2(firstAnchor, secondAnchor)}
				var persistent bool
				if cached, found := w.contactCache[contact.pair]; found && cached.normalX*contact.Normal.X+cached.normalY*contact.Normal.Y > 0.9 {
					persistent = true
					if w.Config.EnableWarmStarting && anchorsMatch2(cached, firstAnchor, secondAnchor) {
						contact.normalImpulse, contact.tangentImpulse = cached.normalImpulse, cached.tangentImpulse
					}
				}
				if !persistent {
					contact.restitutionBias = contactRestitutionBias2(&contact, w.Config.RestitutionThreshold)
				}
				w.Contacts = append(w.Contacts, contact)
			}
		}
	}
}

// ensureSpatialHash rebuilds the query index after structural or explicit transform changes.
func (w *World2[T]) ensureSpatialHash() {
	if w.spatialHashDirty {
		w.rebuildSpatialHash()
	}
}

// rebuildSpatialHash refreshes the broad-phase index from current body bounds.
func (w *World2[T]) rebuildSpatialHash() {
	w.spatialHash.Clear()
	for _, body := range w.bodyOrder {
		if !body.Disabled {
			w.spatialHash.Insert(body)
		}
	}
	w.spatialHashDirty = false
}

// collideBodyManifold2 returns up to two stable contact points for a body pair.
func collideBodyManifold2[T constraints.Float](first, second *Body2[T]) (contacts [2]Contact2[T], count int) {
	var (
		firstPolygon, firstPolygonOK   = first.Shape.(*Polygon2[T])
		secondPolygon, secondPolygonOK = second.Shape.(*Polygon2[T])
		contact                        Contact2[T]
		hit                            bool
	)
	if firstPolygonOK && secondPolygonOK {
		contacts, count = collidePolygons2(first, second, firstPolygon, secondPolygon)
		return
	}
	contact, hit = collideBodies2(first, second)
	if hit {
		contacts[0], count = contact, 1
	}
	return
}

// collidePolygons2 builds a two-point manifold from the polygon intersection boundary.
func collidePolygons2[T constraints.Float](firstBody, secondBody *Body2[T], first, second *Polygon2[T]) (contacts [2]Contact2[T], count int) {
	var resolution *vector.Vec2[T] = poly.ResolveTwoPolygons(first.Polygon, second.Polygon)
	if resolution == nil {
		return
	}
	var penetration T = resolution.Length()
	if penetration <= 0 {
		return
	}
	var normal vector.Vec2[T] = vector.Vec2[T]{X: -resolution.X / penetration, Y: -resolution.Y / penetration}
	var candidates []vector.Vec2[T] = polygonIntersectionPoints2(first.Polygon.Points, second.Polygon.Points)
	if len(candidates) == 0 {
		candidates = append(candidates, polygonContactPoint2(first, second, normal))
	}
	var tangent vector.Vec2[T] = vector.Vec2[T]{X: -normal.Y, Y: normal.X}
	var minimumIndex, maximumIndex int
	var minimum, maximum T = candidates[0].Dot(&tangent), candidates[0].Dot(&tangent)
	for index := 1; index < len(candidates); index++ {
		var projection T = candidates[index].Dot(&tangent)
		if projection < minimum {
			minimum, minimumIndex = projection, index
		}
		if projection > maximum {
			maximum, maximumIndex = projection, index
		}
	}
	contacts[0] = Contact2[T]{First: firstBody, Second: secondBody, Point: candidates[minimumIndex], Normal: normal, Penetration: penetration}
	count = 1
	if maximumIndex != minimumIndex && maximum-minimum > T(1e-5) {
		contacts[1] = Contact2[T]{First: firstBody, Second: secondBody, Point: candidates[maximumIndex], Normal: normal, Penetration: penetration}
		count = 2
	}
	return
}

// polygonIntersectionPoints2 collects vertices and edge crossings in the overlap region.
func polygonIntersectionPoints2[T constraints.Float](first, second []*vector.Vec2[T]) (points []vector.Vec2[T]) {
	for _, point := range first {
		if pointInsidePolygon2(*point, second) {
			points = appendUniquePoint2(points, *point)
		}
	}
	for _, point := range second {
		if pointInsidePolygon2(*point, first) {
			points = appendUniquePoint2(points, *point)
		}
	}
	for firstIndex := range first {
		for secondIndex := range second {
			var point vector.Vec2[T]
			var intersects bool
			point, intersects = segmentIntersection2(*first[firstIndex], *first[(firstIndex+1)%len(first)], *second[secondIndex], *second[(secondIndex+1)%len(second)])
			if intersects {
				points = appendUniquePoint2(points, point)
			}
		}
	}
	return
}

func pointInsidePolygon2[T constraints.Float](point vector.Vec2[T], polygon []*vector.Vec2[T]) (inside bool) {
	for index, previous := 0, len(polygon)-1; index < len(polygon); previous, index = index, index+1 {
		var first, second *vector.Vec2[T] = polygon[index], polygon[previous]
		if (first.Y > point.Y) != (second.Y > point.Y) && point.X < (second.X-first.X)*(point.Y-first.Y)/(second.Y-first.Y)+first.X {
			inside = !inside
		}
	}
	return
}

func segmentIntersection2[T constraints.Float](a, b, c, d vector.Vec2[T]) (point vector.Vec2[T], intersects bool) {
	var (
		abX, abY    T = b.X - a.X, b.Y - a.Y
		cdX, cdY    T = d.X - c.X, d.Y - c.Y
		denominator T = abX*cdY - abY*cdX
	)
	if math.Abs(float64(denominator)) <= 1e-9 {
		return
	}
	var (
		acX, acY    T = c.X - a.X, c.Y - a.Y
		firstRatio  T = (acX*cdY - acY*cdX) / denominator
		secondRatio T = (acX*abY - acY*abX) / denominator
	)
	if firstRatio < 0 || firstRatio > 1 || secondRatio < 0 || secondRatio > 1 {
		return
	}
	point = vector.Vec2[T]{X: a.X + abX*firstRatio, Y: a.Y + abY*firstRatio}
	intersects = true
	return
}

func appendUniquePoint2[T constraints.Float](points []vector.Vec2[T], point vector.Vec2[T]) (result []vector.Vec2[T]) {
	for index := range points {
		var dx, dy T = points[index].X - point.X, points[index].Y - point.Y
		if dx*dx+dy*dy <= T(1e-8) {
			result = points
			return
		}
	}
	result = append(points, point)
	return
}

// storeContactCache retains solved impulses and removes contacts absent this step.
func (w *World2[T]) storeContactCache() {
	for i := range w.Contacts {
		var firstAnchor, secondAnchor vector.Vec2[T] = contactLocalAnchor2(w.Contacts[i].First, w.Contacts[i].Point), contactLocalAnchor2(w.Contacts[i].Second, w.Contacts[i].Point)
		w.contactCache[w.Contacts[i].pair] = cachedContact2[T]{normalImpulse: w.Contacts[i].normalImpulse, tangentImpulse: w.Contacts[i].tangentImpulse, normalX: w.Contacts[i].Normal.X, normalY: w.Contacts[i].Normal.Y, firstAnchorX: firstAnchor.X, firstAnchorY: firstAnchor.Y, secondAnchorX: secondAnchor.X, secondAnchorY: secondAnchor.Y, generation: w.generation}
	}
	for pair, cached := range w.contactCache {
		if cached.generation != w.generation {
			delete(w.contactCache, pair)
		}
	}
}

func contactLocalAnchor2[T constraints.Float](body *Body2[T], point vector.Vec2[T]) (anchor vector.Vec2[T]) {
	var sine, cosine float64 = math.Sincos(float64(body.Rotation))
	var x, y T = point.X - body.Position.X, point.Y - body.Position.Y
	anchor = vector.Vec2[T]{X: x*T(cosine) + y*T(sine), Y: -x*T(sine) + y*T(cosine)}
	return
}

func anchorsMatch2[T constraints.Float](cached cachedContact2[T], first, second vector.Vec2[T]) (matches bool) {
	var firstX, firstY, secondX, secondY T = first.X - cached.firstAnchorX, first.Y - cached.firstAnchorY, second.X - cached.secondAnchorX, second.Y - cached.secondAnchorY
	matches = firstX*firstX+firstY*firstY < .0025 && secondX*secondX+secondY*secondY < .0025
	return
}

// contactRestitutionBias2 captures the desired bounce velocity before iterative solving.
func contactRestitutionBias2[T constraints.Float](contact *Contact2[T], threshold T) (bias T) {
	var (
		rA             vector.Vec2[T] = vector.Vec2[T]{X: contact.Point.X - contact.First.Position.X, Y: contact.Point.Y - contact.First.Position.Y}
		rB             vector.Vec2[T] = vector.Vec2[T]{X: contact.Point.X - contact.Second.Position.X, Y: contact.Point.Y - contact.Second.Position.Y}
		firstVelocity  vector.Vec2[T] = vector.Vec2[T]{X: contact.First.Velocity.X - contact.First.AngularVelocity*rA.Y, Y: contact.First.Velocity.Y + contact.First.AngularVelocity*rA.X}
		secondVelocity vector.Vec2[T] = vector.Vec2[T]{X: contact.Second.Velocity.X - contact.Second.AngularVelocity*rB.Y, Y: contact.Second.Velocity.Y + contact.Second.AngularVelocity*rB.X}
		velocity       T              = (secondVelocity.X-firstVelocity.X)*contact.Normal.X + (secondVelocity.Y-firstVelocity.Y)*contact.Normal.Y
	)
	if velocity < -threshold {
		bias = -min(contact.First.Material.Restitution, contact.Second.Material.Restitution) * velocity
	}
	return
}

func contactFeature2[T constraints.Float](first, second vector.Vec2[T]) (feature uint32) {
	feature = 2166136261
	for _, value := range []T{first.X, first.Y, second.X, second.Y} {
		feature ^= uint32(int32(math.Round(float64(value) * 1000))) // #nosec G115 -- signed coordinate bits are intentionally folded into the FNV-style hash.
		feature *= 16777619
	}
	return
}

// collideBodies2 dispatches supported two-dimensional shape pairs.
func collideBodies2[T constraints.Float](first, second *Body2[T]) (contact Contact2[T], hit bool) {
	contact.First, contact.Second = first, second
	var firstCircle, firstCircleOK = first.Shape.(*Circle2[T])
	var secondCircle, secondCircleOK = second.Shape.(*Circle2[T])
	if firstCircleOK && secondCircleOK {
		contact.Point, contact.Normal, contact.Penetration, hit = collideCircles2(firstCircle, secondCircle)
		return
	}
	var firstPolygon, firstPolygonOK = first.Shape.(*Polygon2[T])
	var secondPolygon, secondPolygonOK = second.Shape.(*Polygon2[T])
	if firstPolygonOK && secondPolygonOK {
		var resolution *vector.Vec2[T] = poly.ResolveTwoPolygons(firstPolygon.Polygon, secondPolygon.Polygon)
		if resolution == nil {
			return
		}
		contact.Penetration = resolution.Length()
		if contact.Penetration > 0 {
			contact.Normal = vector.Vec2[T]{X: -resolution.X / contact.Penetration, Y: -resolution.Y / contact.Penetration}
		}
		contact.Point = polygonContactPoint2(firstPolygon, secondPolygon, contact.Normal)
		hit = true
		return
	}
	if firstCircleOK && secondPolygonOK {
		contact.Point, contact.Normal, contact.Penetration, hit = collideCirclePolygon2(firstCircle, secondPolygon)
		return
	}
	if firstPolygonOK && secondCircleOK {
		contact.Point, contact.Normal, contact.Penetration, hit = collideCirclePolygon2(secondCircle, firstPolygon)
		contact.Normal.X, contact.Normal.Y = -contact.Normal.X, -contact.Normal.Y
	}
	return
}

// polygonContactPoint2 averages the opposing support faces of a polygon pair.
func polygonContactPoint2[T constraints.Float](first, second *Polygon2[T], normal vector.Vec2[T]) (point vector.Vec2[T]) {
	var firstSupport vector.Vec2[T] = polygonSupportFace2(first.Polygon.Points, normal, true)
	var secondSupport vector.Vec2[T] = polygonSupportFace2(second.Polygon.Points, normal, false)
	point = vector.Vec2[T]{X: (firstSupport.X + secondSupport.X) / 2, Y: (firstSupport.Y + secondSupport.Y) / 2}
	return
}

// polygonSupportFace2 returns the center of the extreme face along an axis.
func polygonSupportFace2[T constraints.Float](points []*vector.Vec2[T], axis vector.Vec2[T], maximum bool) (support vector.Vec2[T]) {
	var extreme T
	if maximum {
		extreme = T(math.Inf(-1))
	} else {
		extreme = T(math.Inf(1))
	}
	for _, point := range points {
		var projection T = point.X*axis.X + point.Y*axis.Y
		if maximum && projection > extreme || !maximum && projection < extreme {
			extreme = projection
		}
	}
	var count T
	for _, point := range points {
		var projection T = point.X*axis.X + point.Y*axis.Y
		if math.Abs(float64(projection-extreme)) <= 1e-5 {
			support.X += point.X
			support.Y += point.Y
			count++
		}
	}
	if count > 0 {
		support.X /= count
		support.Y /= count
	}
	return
}

// collideCircles2 computes an analytic circle contact.
func collideCircles2[T constraints.Float](first, second *Circle2[T]) (point, normal vector.Vec2[T], penetration T, hit bool) {
	var (
		dx, dy          T = second.Position.X - first.Position.X, second.Position.Y - first.Position.Y
		radius          T = first.Radius + second.Radius
		distanceSquared T = dx*dx + dy*dy
	)
	if distanceSquared > radius*radius {
		return
	}
	var distance T = T(math.Sqrt(float64(distanceSquared)))
	if distance == 0 {
		normal.X = 1
	} else {
		normal.X, normal.Y = dx/distance, dy/distance
	}
	penetration = radius - distance
	point = vector.Vec2[T]{X: first.Position.X + normal.X*(first.Radius-penetration/2), Y: first.Position.Y + normal.Y*(first.Radius-penetration/2)}
	hit = true
	return
}

// collideCirclePolygon2 computes the closest boundary contact for a circle and polygon.
func collideCirclePolygon2[T constraints.Float](circle *Circle2[T], polygon *Polygon2[T]) (point, normal vector.Vec2[T], penetration T, hit bool) {
	var bestDistance T = T(math.Inf(1))
	for i := range polygon.Polygon.Points {
		var (
			start         *vector.Vec2[T] = polygon.Polygon.Points[i]
			end           *vector.Vec2[T] = polygon.Polygon.Points[(i+1)%len(polygon.Polygon.Points)]
			edgeX, edgeY  T               = end.X - start.X, end.Y - start.Y
			toX, toY      T               = circle.Position.X - start.X, circle.Position.Y - start.Y
			lengthSquared T               = edgeX*edgeX + edgeY*edgeY
			ratio         T
		)
		if lengthSquared != 0 {
			ratio = max(0, min(1, (edgeX*toX+edgeY*toY)/lengthSquared))
		}
		var (
			candidate vector.Vec2[T] = vector.Vec2[T]{X: start.X + edgeX*ratio, Y: start.Y + edgeY*ratio}
			dx, dy    T              = candidate.X - circle.Position.X, candidate.Y - circle.Position.Y
			distance  T              = dx*dx + dy*dy
		)
		if distance < bestDistance {
			bestDistance, point = distance, candidate
		}
	}
	var (
		inside   bool = polygon.Polygon.PointIsInside(&circle.Position)
		distance T    = T(math.Sqrt(float64(bestDistance)))
	)
	if !inside && distance > circle.Radius {
		return
	}
	if distance == 0 {
		normal.X = 1
	} else if inside {
		normal.X, normal.Y = (circle.Position.X-point.X)/distance, (circle.Position.Y-point.Y)/distance
	} else {
		normal.X, normal.Y = (point.X-circle.Position.X)/distance, (point.Y-circle.Position.Y)/distance
	}
	if inside {
		penetration = circle.Radius + distance
	} else {
		penetration = circle.Radius - distance
	}
	hit = true
	return
}

// resolveVelocity2 applies normal and friction impulses for one contact.
func resolveVelocity2[T constraints.Float](contact *Contact2[T]) {
	if contact.First.Sensor || contact.Second.Sensor {
		return
	}
	var (
		first, second         *Body2[T]      = contact.First, contact.Second
		rA                    vector.Vec2[T] = vector.Vec2[T]{X: contact.Point.X - first.Position.X, Y: contact.Point.Y - first.Position.Y}
		rB                    vector.Vec2[T] = vector.Vec2[T]{X: contact.Point.X - second.Position.X, Y: contact.Point.Y - second.Position.Y}
		firstContactVelocity  vector.Vec2[T] = vector.Vec2[T]{X: first.Velocity.X - first.AngularVelocity*rA.Y, Y: first.Velocity.Y + first.AngularVelocity*rA.X}
		secondContactVelocity vector.Vec2[T] = vector.Vec2[T]{X: second.Velocity.X - second.AngularVelocity*rB.Y, Y: second.Velocity.Y + second.AngularVelocity*rB.X}
		relative              vector.Vec2[T] = vector.Vec2[T]{X: secondContactVelocity.X - firstContactVelocity.X, Y: secondContactVelocity.Y - firstContactVelocity.Y}
		velocityAlongNormal   T              = relative.X*contact.Normal.X + relative.Y*contact.Normal.Y
	)
	if velocityAlongNormal >= contact.restitutionBias && contact.normalImpulse == 0 {
		return
	}
	var (
		rACrossNormal T = rA.X*contact.Normal.Y - rA.Y*contact.Normal.X
		rBCrossNormal T = rB.X*contact.Normal.Y - rB.Y*contact.Normal.X
		denominator   T = first.inverseMass + second.inverseMass + rACrossNormal*rACrossNormal*first.inverseInertia + rBCrossNormal*rBCrossNormal*second.inverseInertia
	)
	if denominator == 0 {
		return
	}
	var (
		magnitude             T = -(velocityAlongNormal - contact.restitutionBias) / denominator
		previousNormalImpulse T = contact.normalImpulse
	)
	contact.normalImpulse = max(previousNormalImpulse+magnitude, 0)
	magnitude = contact.normalImpulse - previousNormalImpulse
	var impulse vector.Vec2[T] = vector.Vec2[T]{X: contact.Normal.X * magnitude, Y: contact.Normal.Y * magnitude}
	applyContactImpulse2(first, second, rA, rB, impulse)

	firstContactVelocity = vector.Vec2[T]{X: first.Velocity.X - first.AngularVelocity*rA.Y, Y: first.Velocity.Y + first.AngularVelocity*rA.X}
	secondContactVelocity = vector.Vec2[T]{X: second.Velocity.X - second.AngularVelocity*rB.Y, Y: second.Velocity.Y + second.AngularVelocity*rB.X}
	relative = vector.Vec2[T]{X: secondContactVelocity.X - firstContactVelocity.X, Y: secondContactVelocity.Y - firstContactVelocity.Y}
	var tangent vector.Vec2[T] = vector.Vec2[T]{X: -contact.Normal.Y, Y: contact.Normal.X}
	var (
		rACrossTangent     T = rA.X*tangent.Y - rA.Y*tangent.X
		rBCrossTangent     T = rB.X*tangent.Y - rB.Y*tangent.X
		tangentDenominator T = first.inverseMass + second.inverseMass + rACrossTangent*rACrossTangent*first.inverseInertia + rBCrossTangent*rBCrossTangent*second.inverseInertia
	)
	if tangentDenominator == 0 {
		return
	}
	var frictionMagnitude T = -(relative.X*tangent.X + relative.Y*tangent.Y) / tangentDenominator
	var staticFriction T = T(math.Sqrt(float64(first.Material.StaticFriction * second.Material.StaticFriction)))
	var previousTangentImpulse T = contact.tangentImpulse
	contact.tangentImpulse = previousTangentImpulse + frictionMagnitude
	if contact.tangentImpulse > contact.normalImpulse*staticFriction || contact.tangentImpulse < -contact.normalImpulse*staticFriction {
		var dynamicFriction T = T(math.Sqrt(float64(first.Material.DynamicFriction * second.Material.DynamicFriction)))
		contact.tangentImpulse = max(-contact.normalImpulse*dynamicFriction, min(contact.normalImpulse*dynamicFriction, contact.tangentImpulse))
	}
	frictionMagnitude = contact.tangentImpulse - previousTangentImpulse
	applyContactImpulse2(first, second, rA, rB, vector.Vec2[T]{X: tangent.X * frictionMagnitude, Y: tangent.Y * frictionMagnitude})
}

// warmStart2 reapplies the previous step's accumulated impulses.
func warmStart2[T constraints.Float](contact *Contact2[T]) {
	if contact.First.Sensor || contact.Second.Sensor || contact.normalImpulse == 0 && contact.tangentImpulse == 0 {
		return
	}
	var (
		rA      vector.Vec2[T] = vector.Vec2[T]{X: contact.Point.X - contact.First.Position.X, Y: contact.Point.Y - contact.First.Position.Y}
		rB      vector.Vec2[T] = vector.Vec2[T]{X: contact.Point.X - contact.Second.Position.X, Y: contact.Point.Y - contact.Second.Position.Y}
		tangent vector.Vec2[T] = vector.Vec2[T]{X: -contact.Normal.Y, Y: contact.Normal.X}
		impulse vector.Vec2[T] = vector.Vec2[T]{X: contact.Normal.X*contact.normalImpulse + tangent.X*contact.tangentImpulse, Y: contact.Normal.Y*contact.normalImpulse + tangent.Y*contact.tangentImpulse}
	)
	applyContactImpulse2(contact.First, contact.Second, rA, rB, impulse)
}

// applyContactImpulse2 applies an equal and opposite impulse to a body pair.
func applyContactImpulse2[T constraints.Float](first, second *Body2[T], rA, rB, impulse vector.Vec2[T]) {
	if first.Type == DynamicBody {
		first.Velocity.X -= impulse.X * first.inverseMass
		first.Velocity.Y -= impulse.Y * first.inverseMass
		first.AngularVelocity -= (rA.X*impulse.Y - rA.Y*impulse.X) * first.inverseInertia
	}
	if second.Type == DynamicBody {
		second.Velocity.X += impulse.X * second.inverseMass
		second.Velocity.Y += impulse.Y * second.inverseMass
		second.AngularVelocity += (rB.X*impulse.Y - rB.Y*impulse.X) * second.inverseInertia
	}
}

// resolvePosition2 applies inverse-mass-weighted positional correction.
func resolvePosition2[T constraints.Float](contact *Contact2[T], config WorldConfig[T]) {
	if contact.First.Sensor || contact.Second.Sensor {
		return
	}
	var (
		first, second *Body2[T]      = contact.First, contact.Second
		rA            vector.Vec2[T] = vector.Vec2[T]{X: contact.Point.X - first.Position.X, Y: contact.Point.Y - first.Position.Y}
		rB            vector.Vec2[T] = vector.Vec2[T]{X: contact.Point.X - second.Position.X, Y: contact.Point.Y - second.Position.Y}
		rACrossNormal T              = rA.X*contact.Normal.Y - rA.Y*contact.Normal.X
		rBCrossNormal T              = rB.X*contact.Normal.Y - rB.Y*contact.Normal.X
		denominator   T              = first.inverseMass + second.inverseMass + rACrossNormal*rACrossNormal*first.inverseInertia + rBCrossNormal*rBCrossNormal*second.inverseInertia
	)
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
		first.Rotation -= rACrossNormal * magnitude * first.inverseInertia
	}
	if second.Type == DynamicBody {
		second.Position.X += contact.Normal.X * magnitude * second.inverseMass
		second.Position.Y += contact.Normal.Y * magnitude * second.inverseMass
		second.Rotation += rBCrossNormal * magnitude * second.inverseInertia
	}
}
