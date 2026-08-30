package main

import (
	"math"
	"math/rand"

	"github.com/z46-dev/gamelib"
	"github.com/z46-dev/gamelib/gmath"
	"github.com/z46-dev/gamelib/physics"
	"github.com/z46-dev/gamelib/vector"
)

var objectColors []string = []string{"#E85D5D", "#F4B942", "#55C1FF", "#8DD35F", "#B48EFA"}

// NewGame creates the mixed-shape physics playground.
func NewGame(sim *Simulation) (g *Game) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.GravityY, config.EnableCCD, config.CCDMotionThreshold = 700, true, 8
	config.PositionCorrection, config.PositionIterations, config.CCDMaxSubsteps = .8, 6, 64
	config.PenetrationSlop, config.SleepLinearThreshold, config.SleepAngularThreshold = .1, 1, .02
	g = &Game{World: physics.NewWorld2(config), Objects: gamelib.NewCollection[*Object](), Simulation: sim, Width: 800, Height: 500, SpawnRequests: make(chan vector.Vec2[float64], 128)}
	g.addWalls()
	g.addShelf()
	g.addRope()
	var err error
	for index := range 24 {
		if _, err = g.addRandomObject(vector.Vec2[float64]{X: float64(index%8)*80 - 280, Y: float64(index/8)*70 - 180}); err != nil {
			panic(err)
		}
	}
	return
}

// Update consumes input, advances physics, and publishes a render snapshot.
func (g *Game) Update(dt float64) {
	var err error
	for draining := true; draining; {
		select {
		case position := <-g.SpawnRequests:
			if _, err = g.addRandomObject(position); err != nil {
				panic(err)
			}
		default:
			draining = false
		}
	}
	g.Objects.Lock.Lock()
	for _, object := range g.Objects.ToAppend {
		g.Objects.Items[object.ID] = object
	}
	for _, id := range g.Objects.ToRemove {
		delete(g.Objects.Items, id)
		g.World.RemoveBody(physics.BodyID(id))
	}
	g.Objects.ToAppend, g.Objects.ToRemove = g.Objects.ToAppend[:0], g.Objects.ToRemove[:0]
	g.Objects.Lock.Unlock()
	g.World.Step(dt)
	g.Publish(g.Simulation)
}

// Publish copies physics transforms into the interpolated render model.
func (g *Game) Publish(sim *Simulation) {
	sim.Metrics.observedUPS++
	sim.WorldSize.Set(vector.NewVec2(g.Width, g.Height))
	sim.Objects.Lock.RLock()
	defer sim.Objects.Lock.RUnlock()
	for _, object := range g.Objects.Items {
		var rendered *SimulationObject
		var exists bool
		if rendered, exists = sim.Objects.Items[object.ID]; !exists {
			sim.Objects.Add(&SimulationObject{ID: object.ID, Simulation: sim, Position: vector.NewLerpV2(vector.NewVec2(object.Body.Position.X, object.Body.Position.Y)), Radius: vector.NewLerpScalar(object.Radius), Rotation: vector.NewLerpDirection(object.Body.Rotation), Points: object.Points, Color: object.Color})
		} else {
			rendered.Position.Set(&object.Body.Position)
			rendered.Rotation.Set(object.Body.Rotation)
		}
	}
	for _, rendered := range sim.Objects.Items {
		var exists bool
		if _, exists = g.Objects.Items[rendered.ID]; !exists {
			sim.Objects.Remove(rendered.ID)
		}
	}
	if g.Rope != nil {
		sim.LinksLock.Lock()
		sim.Links = sim.Links[:0]
		for _, link := range g.Rope.Constraints {
			sim.Links = append(sim.Links, SimulationLink{First: uint64(link.First.ID), Second: uint64(link.Second.ID), Broken: link.Broken})
		}
		sim.LinksLock.Unlock()
	}
}

// QueueSpawn requests a random object at a world-space point without blocking input.
func (g *Game) QueueSpawn(position vector.Vec2[float64]) {
	position.X = gmath.Clamp(position.X, -g.Width/2+32, g.Width/2-32)
	position.Y = gmath.Clamp(position.Y, -g.Height/2+32, g.Height/2-32)
	select {
	case g.SpawnRequests <- position:
	default:
	}
}

func (g *Game) addRandomObject(position vector.Vec2[float64]) (object *Object, err error) {
	var choice int = rand.Intn(5)
	var points []vector.Vec2[float64]
	var radius float64
	if choice == 0 {
		radius = gmath.RandomRange[float64](10, 22)
	} else {
		points = objectPolygon(choice, gmath.RandomRange[float64](14, 27))
	}
	var shape physics.Shape2[float64]
	if radius > 0 {
		shape = physics.NewCircle2(radius)
	} else {
		shape = physics.NewPolygon2(points, 1)
	}
	var bound float64 = radius
	for _, point := range points {
		bound = max(bound, point.Length())
	}
	position = g.clearSpawnPosition(position, bound)
	var body *physics.Body2[float64]
	body, err = g.World.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: shape, Position: position, Rotation: rand.Float64() * gmath.TAU, AngularVelocity: gmath.RandomRange[float64](-1, 1), LinearDamping: .65, AngularDamping: .9, Continuous: true, Material: physics.Material[float64]{Density: .01, Restitution: .06, StaticFriction: .75, DynamicFriction: .55}})
	if err != nil {
		return
	}
	object = &Object{ID: uint64(body.ID), Body: body, Radius: radius, Bound: bound, Points: points, Color: objectColors[rand.Intn(len(objectColors))]}
	g.Objects.Add(object)
	return
}

// clearSpawnPosition finds nearby free space instead of creating deep overlaps.
func (g *Game) clearSpawnPosition(requested vector.Vec2[float64], bound float64) (position vector.Vec2[float64]) {
	position = requested
	for attempt := range 32 {
		var available bool = true
		for _, object := range g.Objects.Items {
			if object.Body.Type == physics.DynamicBody && position.Dist(&object.Body.Position) < bound+object.Bound+3 {
				available = false
				break
			}
		}
		if available {
			for _, object := range g.Objects.ToAppend {
				if object.Body.Type == physics.DynamicBody && position.Dist(&object.Body.Position) < bound+object.Bound+3 {
					available = false
					break
				}
			}
		}
		if available {
			return
		}
		var angle float64 = float64(attempt) * 2.399963229728653
		var distance float64 = float64(attempt/6+1) * (bound*2 + 6)
		position = vector.Vec2[float64]{X: gmath.Clamp(requested.X+math.Cos(angle)*distance, -g.Width/2+bound, g.Width/2-bound), Y: gmath.Clamp(requested.Y+math.Sin(angle)*distance, -g.Height/2+bound, g.Height/2-bound)}
	}
	return
}

func objectPolygon(choice int, size float64) (points []vector.Vec2[float64]) {
	switch choice {
	case 1:
		points = []vector.Vec2[float64]{{X: 0, Y: -size}, {X: size, Y: size}, {X: -size, Y: size}}
	case 2:
		points = rectanglePoints(size*2, size*1.4)
	case 3:
		for index := range 6 {
			var angle float64 = float64(index) * gmath.TAU / 6
			points = append(points, vector.Vec2[float64]{X: math.Cos(angle) * size, Y: math.Sin(angle) * size})
		}
	default:
		points = []vector.Vec2[float64]{{X: -size, Y: -size}, {X: size * .2, Y: -size * .35}, {X: size, Y: -size}, {X: size * .45, Y: size * .15}, {X: size * .75, Y: size}, {X: 0, Y: size * .45}, {X: -size * .75, Y: size}, {X: -size * .45, Y: size * .15}}
	}
	return
}

func (g *Game) addShelf() {
	var body *physics.Body2[float64]
	var err error
	body, err = g.World.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: rectangle(270, 18), Position: vector.Vec2[float64]{X: 120, Y: 65}, Rotation: -.08, Material: physics.Material[float64]{Density: 1, Restitution: .1, StaticFriction: .8, DynamicFriction: .6}})
	if err != nil {
		panic(err)
	}
	g.Objects.Add(&Object{ID: uint64(body.ID), Body: body, Bound: 136, Points: rectanglePoints(270, 18), Color: "#707782"})
}

func (g *Game) addRope() {
	var first, second *physics.Body2[float64]
	var ropeFilter physics.CollisionFilter = physics.CollisionFilter{Category: 2, Mask: ^uint32(2)}
	var err error
	first, err = g.World.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewCircle2(7.0), Position: vector.Vec2[float64]{X: -260, Y: -180}, Filter: ropeFilter})
	if err != nil {
		panic(err)
	}
	var ropeMaterial physics.Material[float64] = physics.Material[float64]{Density: .02, Restitution: .02, StaticFriction: .75, DynamicFriction: .6}
	second, err = g.World.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(10.0), Position: vector.Vec2[float64]{X: -70, Y: -180}, LinearDamping: 2, AngularDamping: 2, Material: ropeMaterial, Filter: ropeFilter})
	if err != nil {
		panic(err)
	}
	g.Rope, err = g.World.AddRope(first.ID, second.ID, physics.RopeConfig[float64]{Segments: 12, Radius: 5, Mass: .4, Compliance: .000002, Damping: .85, LinearDamping: 2, AngularDamping: 2, Reconnection: physics.ReconnectWhenClose, ReconnectDistance: 22, Material: ropeMaterial, Filter: ropeFilter})
	if err != nil {
		panic(err)
	}
	for _, body := range g.Rope.Bodies {
		var radius float64 = body.Shape.(*physics.Circle2[float64]).Radius
		g.Objects.Add(&Object{ID: uint64(body.ID), Body: body, Radius: radius, Bound: radius, Color: "#E6D690"})
	}
}

func (g *Game) addWalls() {
	var halfWidth, halfHeight, thickness float64 = g.Width / 2, g.Height / 2, 32
	var material physics.Material[float64] = physics.Material[float64]{Density: 1, Restitution: .05, StaticFriction: .8, DynamicFriction: .6}
	var walls []physics.Body2Config[float64] = []physics.Body2Config[float64]{{Type: physics.StaticBody, Shape: rectangle(g.Width+thickness*2, thickness), Position: vector.Vec2[float64]{Y: -halfHeight - thickness/2}, Material: material}, {Type: physics.StaticBody, Shape: rectangle(g.Width+thickness*2, thickness), Position: vector.Vec2[float64]{Y: halfHeight + thickness/2}, Material: material}, {Type: physics.StaticBody, Shape: rectangle(thickness, g.Height+thickness*2), Position: vector.Vec2[float64]{X: -halfWidth - thickness/2}, Material: material}, {Type: physics.StaticBody, Shape: rectangle(thickness, g.Height+thickness*2), Position: vector.Vec2[float64]{X: halfWidth + thickness/2}, Material: material}}
	var err error
	for _, wall := range walls {
		if _, err = g.World.AddBody(wall); err != nil {
			panic(err)
		}
	}
}

func rectangle(width, height float64) (shape *physics.Polygon2[float64]) {
	shape = physics.NewPolygon2(rectanglePoints(width, height), 1)
	return
}
func rectanglePoints(width, height float64) (points []vector.Vec2[float64]) {
	points = []vector.Vec2[float64]{{X: -width / 2, Y: -height / 2}, {X: width / 2, Y: -height / 2}, {X: width / 2, Y: height / 2}, {X: -width / 2, Y: height / 2}}
	return
}
