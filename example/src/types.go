package main

import (
	"sync"

	"github.com/z46-dev/gamelib"
	"github.com/z46-dev/gamelib/physics"
	"github.com/z46-dev/gamelib/vector"
)

type (
	// "Server"-side

	Game struct {
		World         *physics.World2[float64]
		Objects       *gamelib.Collection[*Object]
		Width, Height float64
		SpawnRequests chan vector.Vec2[float64]
		Rope          *physics.Rope2[float64]

		Simulation *Simulation
	}

	Object struct {
		ID     uint64
		Body   *physics.Body2[float64]
		Radius float64
		Bound  float64
		Points []vector.Vec2[float64]
		Color  string
	}

	// "Client"-side

	Simulation struct {
		Metrics struct {
			timer            float64
			UPS, observedUPS int
			FPS, observedFPS int

			Time struct {
				records                     []float64
				Min, Mean, Median, Max, Sum float64
			}
		}

		WorldSize *vector.LerpV2[float64]
		Objects   *gamelib.Collection[*SimulationObject]
		Links     []SimulationLink
		LinksLock sync.RWMutex
	}

	SimulationObject struct {
		ID         uint64
		Simulation *Simulation
		Position   *vector.LerpV2[float64]
		Radius     *vector.LerpScalar[float64]
		Rotation   *vector.LerpDirection[float64]
		Points     []vector.Vec2[float64]
		Color      string
	}

	SimulationLink struct {
		First, Second uint64
		Broken        bool
	}
)
