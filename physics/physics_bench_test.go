package physics_test

import (
	"testing"

	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/physics"
	"github.com/z46-dev/gamelib/vector"
)

var benchmarkBodyPosition float64

func BenchmarkWorld2Step100Circles(b *testing.B) {
	var (
		config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world  *physics.World2[float64]
		body   *physics.Body2[float64]
		first  *physics.Body2[float64]
		err    error
	)
	world = physics.NewWorld2(config)
	for i := range 100 {
		body, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(0.45), Position: vector.Vec2[float64]{X: float64(i % 10), Y: float64(i / 10)}})
		if err != nil {
			b.Fatal(err)
		}
		if i == 0 {
			first = body
		}
	}
	world.Step(1.0 / 120.0)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		world.Step(1.0 / 120.0)
	}
	benchmarkBodyPosition = first.Position.Y
}

func BenchmarkWorld3Step100Spheres(b *testing.B) {
	var (
		config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world  *physics.World3[float64]
		body   *physics.Body3[float64]
		first  *physics.Body3[float64]
		err    error
	)
	world = physics.NewWorld3(config)
	for i := range 100 {
		body, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(0.45), Position: vector.Vec3[float64]{X: float64(i % 10), Y: float64((i / 10) % 10), Z: float64(i / 100)}})
		if err != nil {
			b.Fatal(err)
		}
		if i == 0 {
			first = body
		}
	}
	world.Step(1.0 / 120.0)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		world.Step(1.0 / 120.0)
	}
	benchmarkBodyPosition = first.Position.Z
}

func BenchmarkWorld2Step10000SparseCircles(b *testing.B) {
	var (
		config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world  *physics.World2[float64]
		err    error
	)
	config.EnableSleeping = false
	world = physics.NewWorld2(config)
	for i := range 10000 {
		_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(0.5), Position: vector.Vec2[float64]{X: float64(i%100) * 128, Y: float64(i/100) * 128}, DisableGravity: true})
		if err != nil {
			b.Fatal(err)
		}
	}
	world.Step(1.0 / 120.0)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		world.Step(1.0 / 120.0)
	}
}

func BenchmarkWorld3Step10000SparseSpheres(b *testing.B) {
	var (
		config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world  *physics.World3[float64]
		err    error
	)
	config.EnableSleeping = false
	world = physics.NewWorld3(config)
	for i := range 10000 {
		_, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(0.5), Position: vector.Vec3[float64]{X: float64(i%100) * 128, Y: float64((i/100)%100) * 128, Z: float64(i/10000) * 128}, DisableGravity: true})
		if err != nil {
			b.Fatal(err)
		}
	}
	world.Step(1.0 / 120.0)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		world.Step(1.0 / 120.0)
	}
}

func BenchmarkWorld2Step10000TouchingCircles(b *testing.B) {
	var (
		config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world  *physics.World2[float64]
		err    error
	)
	config.EnableSleeping = false
	config.SpatialHash = hshg.SpatialHashConfig{Levels: []hshg.SpatialHashLevelConfig{{Shift: 0, MaxCellsPerObject: 8}, {Shift: 3, MaxCellsPerObject: 16}, {Shift: 6, MaxCellsPerObject: 64}}}
	world = physics.NewWorld2(config)
	for i := range 10000 {
		_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(0.5), Position: vector.Vec2[float64]{X: float64(i % 100), Y: float64(i / 100)}, DisableGravity: true})
		if err != nil {
			b.Fatal(err)
		}
	}
	world.Step(1.0 / 120.0)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		world.Step(1.0 / 120.0)
	}
}

func BenchmarkWorld3Step10000TouchingSpheres(b *testing.B) {
	var (
		config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world  *physics.World3[float64]
		err    error
	)
	config.EnableSleeping = false
	config.SpatialHash = hshg.SpatialHashConfig{Levels: []hshg.SpatialHashLevelConfig{{Shift: 0, MaxCellsPerObject: 16}, {Shift: 3, MaxCellsPerObject: 32}, {Shift: 6, MaxCellsPerObject: 128}}}
	world = physics.NewWorld3(config)
	for i := range 10000 {
		_, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(0.5), Position: vector.Vec3[float64]{X: float64(i % 22), Y: float64((i / 22) % 22), Z: float64(i / 484)}, DisableGravity: true})
		if err != nil {
			b.Fatal(err)
		}
	}
	world.Step(1.0 / 120.0)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		world.Step(1.0 / 120.0)
	}
}

func benchmarkWorld3IndependentIslands(b *testing.B, workers int) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.EnableSleeping = false
	config.ParallelWorkers = workers
	config.MinimumParallelIslandBodies = 1
	var world *physics.World3[float64] = physics.NewWorld3(config)
	var err error
	for island := range 256 {
		var previous *physics.Body3[float64]
		for particle := range 16 {
			var body *physics.Body3[float64]
			body, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(0.5), Position: vector.Vec3[float64]{X: float64(island)*32 + float64(particle)*0.9}, DisableGravity: true})
			if err != nil {
				b.Fatal(err)
			}
			if previous != nil {
				_, err = world.AddDistanceConstraint(physics.DistanceConstraintConfig[float64]{First: previous.ID, Second: body.ID, RestLength: 0.9, Damping: 0.2})
				if err != nil {
					b.Fatal(err)
				}
			}
			previous = body
		}
	}
	world.Step(1.0 / 120.0)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		world.Step(1.0 / 120.0)
	}
}

func BenchmarkWorld3StepIndependentIslandsSerial(b *testing.B) {
	benchmarkWorld3IndependentIslands(b, 1)
}
func BenchmarkWorld3StepIndependentIslandsParallel(b *testing.B) {
	benchmarkWorld3IndependentIslands(b, 8)
}
