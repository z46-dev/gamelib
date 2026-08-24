package main

import (
	"github.com/z46-dev/gamelib"
	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/vector"
)

type (
	// "Server"-side

	Game struct {
		nextBallID    uint64
		Balls         *gamelib.Collection[*Ball]
		SpatialHash   *hshg.SpatialHash[*Ball, float64]
		Width, Height float64
	}

	Ball struct {
		game               *Game
		ID                 uint64
		Position, Velocity *vector.Vec2[float64]
		Size               float64
		AABB               *hshg.AABB[float64]
		collisionIDs       []uint64
	}

	// "Client"-side
	Simulation struct{}

	SimulationBall struct {
	}
)
