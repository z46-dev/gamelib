package main

import (
	"slices"

	"github.com/z46-dev/gamelib/gmath"
	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/vector"
)

func NewBall(g *Game) (b *Ball) {
	b = &Ball{
		game:     g,
		ID:       g.nextBallID,
		Position: vector.Vec2_0[float64](),
		Velocity: vector.Vec2_0[float64](),
	}

	g.nextBallID++
	g.Balls.Add(b)
	return
}

func (b *Ball) GetAABB() (aabb *hshg.AABB[float64]) {
	aabb = b.AABB
	return
}

func (b *Ball) Update(damping, positionFactor float64) {
	b.Position.X += b.Velocity.X * positionFactor
	b.Position.Y += b.Velocity.Y * positionFactor
	b.Velocity.X *= damping
	b.Velocity.Y *= damping
	b.confine()
	b.syncEntityAABB()
	b.collisionIDs = b.collisionIDs[:0]
}

func (b *Ball) confine() {
	b.Position.X = gmath.Clamp(b.Position.X, b.Size, b.game.Width-b.Size)
	b.Position.Y = gmath.Clamp(b.Position.Y, b.Size, b.game.Height-b.Size)
}

func (b *Ball) syncEntityAABB() {
	b.AABB.X1 = b.Position.X - b.Size
	b.AABB.Y1 = b.Position.Y - b.Size
	b.AABB.X2 = b.Position.X + b.Size
	b.AABB.Y2 = b.Position.Y + b.Size
}

func (b *Ball) Collide() {
	for _, other := range b.game.SpatialHash.Retrieve(b.AABB) {
		if slices.Contains(b.collisionIDs, other.ID) || slices.Contains(other.collisionIDs, b.ID) {
			continue
		}

		b.collisionIDs = append(b.collisionIDs, other.ID)
		other.collisionIDs = append(other.collisionIDs, b.ID)

		CollideBalls(b, other)
	}
}
