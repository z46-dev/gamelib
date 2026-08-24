package main

import (
	"math"

	"github.com/z46-dev/gamelib"
	"github.com/z46-dev/gamelib/hshg"
)

const (
	velocityIntegral  float64 = 125
	velocityRetention float64 = 0.992
)

func NewGame() (g *Game) {
	g = &Game{
		Balls:       gamelib.NewCollection[*Ball](),
		SpatialHash: hshg.NewSpatialHash[*Ball](),
	}

	return
}

func (g *Game) Update(dt float64) {
	var (
		damping        float64 = math.Pow(velocityRetention, dt*0.001)
		positionFactor float64 = velocityIntegral * (1 - damping)
	)

	// Reset the spatial hash
	g.SpatialHash.Clear()

	// Update the asynchronous collection
	g.Balls.Lock.Lock()

	for _, toAdd := range g.Balls.ToAppend {
		g.Balls.Items[toAdd.ID] = toAdd
	}

	for _, toRemove := range g.Balls.ToRemove {
		delete(g.Balls.Items, toRemove)
	}

	g.Balls.ToAppend = g.Balls.ToAppend[:0]
	g.Balls.ToRemove = g.Balls.ToRemove[:0]

	g.Balls.Lock.Unlock()

	// Update balls physics
	for _, ball := range g.Balls.Items {
		ball.Update(damping, positionFactor)
	}

	// Collide balls
	for _, ball := range g.Balls.Items {
		ball.Collide()
	}
}
