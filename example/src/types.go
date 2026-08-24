package main

import "github.com/z46-dev/gamelib"

type (
	Game struct {
		nextBallID    uint64
		Balls         *gamelib.Collection[*Ball]
		SpatialHash   *gamelib.SpatialHash[*Ball, float64]
		Width, Height float64
	}

	Ball struct {
		game               *Game
		ID                 uint64
		Position, Velocity *gamelib.Vec2[float64]
		Size               float64
		AABB               *gamelib.AABB[float64]
		collisionIDs       []uint64
	}
)
