//go:build js && wasm

package main

import (
	"math"
	"math/rand"
	"sync"

	"github.com/z46-dev/gamelib/physics"
	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
)

type (
	MeshKind uint8

	RenderBody struct {
		ID          physics.BodyID
		Mesh        MeshKind
		Position    vector.Vec3[float64]
		Orientation physics.Quaternion[float64]
		Scale       vector.Vec3[float64]
		Color       [3]float32
		Bound       float64
		Movable     bool
	}

	Game struct {
		World      *physics.World3[float64]
		Bodies     []*physics.Body3[float64]
		RenderInfo []RenderBody
		Snapshot   []RenderBody
		stateLock  sync.RWMutex
	}
)

const (
	SphereMesh MeshKind = iota
	CubeMesh
	TetrahedronMesh
	OctahedronMesh
)

// NewGame creates a mixed-shape rigid-body scene inside visible walls.
func NewGame() (game *Game) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.GravityY, config.PositionCorrection, config.VelocityIterations, config.PositionIterations = -18, .75, 16, 7
	config.MaxStepDelta, config.CCDMotionThreshold, config.CCDMaxSubsteps = 1.0/120.0, .05, 128
	game = &Game{World: physics.NewWorld3(config)}
	game.addWalls()
	game.addObjects(42)
	game.publish()
	return
}

// Update advances the 3D physics world and publishes render-ready transforms.
func (game *Game) Update(dt float64) {
	game.stateLock.Lock()
	game.World.Step(dt)
	for index, body := range game.Bodies {
		if game.RenderInfo[index].Movable && (body.Position.Y < -8 || math.Abs(body.Position.X) > 7 || math.Abs(body.Position.Z) > 7) {
			body.SetTransform(spawnPosition(index), physics.IdentityQuaternion[float64]())
			body.Velocity = vector.Vec3[float64]{}
		}
	}
	game.publishLocked()
	game.stateLock.Unlock()
}

// ReadSnapshot copies the latest render state without exposing simulation data.
func (game *Game) ReadSnapshot(destination []RenderBody) (snapshot []RenderBody) {
	game.stateLock.RLock()
	snapshot = append(destination[:0], game.Snapshot...)
	game.stateLock.RUnlock()
	return
}

// MoveBody teleports a picked dynamic body by a camera-relative drag delta.
func (game *Game) MoveBody(id physics.BodyID, delta vector.Vec3[float64]) {
	game.stateLock.Lock()
	var body *physics.Body3[float64]
	var found bool
	if body, found = game.World.Body(id); found && body.Type == physics.DynamicBody {
		var position vector.Vec3[float64] = body.Position
		position.Add(&delta)
		position.X, position.Y, position.Z = max(-5.2, min(5.2, position.X)), max(-3.4, min(7, position.Y)), max(-5.2, min(5.2, position.Z))
		body.SetTransform(position, body.Orientation)
		body.Velocity, body.AngularVelocity = vector.Vec3[float64]{}, vector.Vec3[float64]{}
		game.publishLocked()
	}
	game.stateLock.Unlock()
}

func (game *Game) addObjects(count int) {
	var meshes [3]*poly.Polyhedron[float64] = [3]*poly.Polyhedron[float64]{newCube(), newTetrahedron(), newOctahedron()}
	var (
		body  *physics.Body3[float64]
		shape physics.Shape3[float64]
		err   error
	)
	for index := range count {
		var (
			kind  MeshKind             = MeshKind(index % 4)
			size  float64              = .34 + rand.Float64()*.18
			scale vector.Vec3[float64] = vector.Vec3[float64]{X: size, Y: size, Z: size}
			bound float64              = size
		)
		if kind == SphereMesh {
			shape = physics.NewSphere3(size)
		} else {
			if kind == TetrahedronMesh {
				scale.Y *= 1.18
			}
			shape = physics.NewPolyhedron3(meshes[kind-1], scale)
			bound = math.Sqrt(scale.X*scale.X + scale.Y*scale.Y + scale.Z*scale.Z)
		}
		if body, err = game.World.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: shape, Position: spawnPosition(index), Rotation: vector.Vec3[float64]{X: rand.Float64(), Y: rand.Float64(), Z: rand.Float64()}, LinearDamping: .02, AngularDamping: .02, AngularVelocity: vector.Vec3[float64]{X: rand.Float64() - .5, Y: rand.Float64() - .5, Z: rand.Float64() - .5}, Material: physics.Material[float64]{Density: 1, Restitution: .08, StaticFriction: .72, DynamicFriction: .52}}); err != nil {
			panic(err)
		}
		game.Bodies = append(game.Bodies, body)
		game.RenderInfo = append(game.RenderInfo, RenderBody{ID: body.ID, Mesh: kind, Scale: scale, Color: randomColor(), Bound: bound, Movable: true})
	}
}

func (game *Game) addWalls() {
	var walls []RenderBody = []RenderBody{
		{Mesh: CubeMesh, Position: vector.Vec3[float64]{Y: -4.5}, Scale: vector.Vec3[float64]{X: 6.25, Y: .5, Z: 6.25}, Color: [3]float32{.2, .24, .31}},
		{Mesh: CubeMesh, Position: vector.Vec3[float64]{X: -6.25, Y: 1}, Scale: vector.Vec3[float64]{X: .25, Y: 5, Z: 6.25}, Color: [3]float32{.15, .19, .26}},
		{Mesh: CubeMesh, Position: vector.Vec3[float64]{X: 6.25, Y: 1}, Scale: vector.Vec3[float64]{X: .25, Y: 5, Z: 6.25}, Color: [3]float32{.15, .19, .26}},
		{Mesh: CubeMesh, Position: vector.Vec3[float64]{Y: 1, Z: -6.25}, Scale: vector.Vec3[float64]{X: 6.25, Y: 5, Z: .25}, Color: [3]float32{.17, .21, .28}},
		{Mesh: CubeMesh, Position: vector.Vec3[float64]{Y: 1, Z: 6.25}, Scale: vector.Vec3[float64]{X: 6.25, Y: 5, Z: .25}, Color: [3]float32{.17, .21, .28}},
	}
	var cube *poly.Polyhedron[float64] = newCube()
	for _, wall := range walls {
		var (
			body *physics.Body3[float64]
			err  error
		)
		if body, err = game.World.AddBody(physics.Body3Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolyhedron3(cube, wall.Scale), Position: wall.Position}); err != nil {
			panic(err)
		}
		wall.ID, wall.Orientation = body.ID, physics.IdentityQuaternion[float64]()
		game.Bodies = append(game.Bodies, body)
		game.RenderInfo = append(game.RenderInfo, wall)
	}
}

func (game *Game) publish() {
	game.stateLock.Lock()
	game.publishLocked()
	game.stateLock.Unlock()
}

func (game *Game) publishLocked() {
	game.Snapshot = game.Snapshot[:0]
	for index, body := range game.Bodies {
		var rendered RenderBody = game.RenderInfo[index]
		rendered.Position, rendered.Orientation = body.Position, body.Orientation
		game.Snapshot = append(game.Snapshot, rendered)
	}
}

func newCube() (shape *poly.Polyhedron[float64]) {
	shape = mustPolyhedron([]vector.Vec3[float64]{{X: -1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: -1}, {X: 1, Y: 1, Z: -1}, {X: -1, Y: 1, Z: -1}, {X: -1, Y: -1, Z: 1}, {X: 1, Y: -1, Z: 1}, {X: 1, Y: 1, Z: 1}, {X: -1, Y: 1, Z: 1}}, []poly.Triangle3{{A: 0, B: 2, C: 1}, {A: 0, B: 3, C: 2}, {A: 4, B: 5, C: 6}, {A: 4, B: 6, C: 7}, {A: 0, B: 1, C: 5}, {A: 0, B: 5, C: 4}, {A: 3, B: 7, C: 6}, {A: 3, B: 6, C: 2}, {A: 0, B: 4, C: 7}, {A: 0, B: 7, C: 3}, {A: 1, B: 2, C: 6}, {A: 1, B: 6, C: 5}})
	return
}

func newTetrahedron() (shape *poly.Polyhedron[float64]) {
	shape = mustPolyhedron([]vector.Vec3[float64]{{X: 1, Y: 1, Z: 1}, {X: -1, Y: -1, Z: 1}, {X: -1, Y: 1, Z: -1}, {X: 1, Y: -1, Z: -1}}, []poly.Triangle3{{A: 0, B: 1, C: 2}, {A: 0, B: 3, C: 1}, {A: 0, B: 2, C: 3}, {A: 1, B: 3, C: 2}})
	return
}

func newOctahedron() (shape *poly.Polyhedron[float64]) {
	shape = mustPolyhedron([]vector.Vec3[float64]{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}, {Z: 1}, {Z: -1}}, []poly.Triangle3{{A: 0, B: 2, C: 4}, {A: 4, B: 2, C: 1}, {A: 1, B: 2, C: 5}, {A: 5, B: 2, C: 0}, {A: 4, B: 3, C: 0}, {A: 1, B: 3, C: 4}, {A: 5, B: 3, C: 1}, {A: 0, B: 3, C: 5}})
	return
}

func mustPolyhedron(vertices []vector.Vec3[float64], triangles []poly.Triangle3) (shape *poly.Polyhedron[float64]) {
	var err error
	if shape, err = poly.NewPolyhedron(vertices, triangles); err != nil {
		panic(err)
	}
	return
}

func randomColor() (color [3]float32) {
	color = [3]float32{.25 + rand.Float32()*.7, .3 + rand.Float32()*.65, .4 + rand.Float32()*.55}
	return
}

func spawnPosition(index int) (position vector.Vec3[float64]) {
	position = vector.Vec3[float64]{X: float64(index%4)*1.3 - 1.95, Y: float64(index/16)*1.35 + float64((index/4)%4)*1.25 - 2.4, Z: float64((index/4)%4)*1.3 - 1.95}
	return
}
