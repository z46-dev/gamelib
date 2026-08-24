package main

import (
	"math"
)

func CollideBalls(ballA, ballB *Ball) (collided bool) {
	var (
		deltaX, deltaY             float64 = ballA.Position.X - ballB.Position.X, ballA.Position.Y - ballB.Position.Y
		distanceSquared, radiusSum float64 = deltaX*deltaX + deltaY*deltaY, ballA.Size + ballB.Size
	)

	if distanceSquared <= 0 || distanceSquared >= radiusSum*radiusSum {
		return
	}

	var (
		relativeVelocityX, relativeVelocityY float64 = ballA.Velocity.X - ballB.Velocity.X, ballA.Velocity.Y - ballB.Velocity.Y
		reflection                           float64 = (relativeVelocityX*deltaX + relativeVelocityY*deltaY) * -2 / distanceSquared
		influenceA, influenceB               float64 = splitCollisionInfluence(collisionInfluence(ballB, ballA), collisionInfluence(ballA, ballB))
	)

	ballA.Velocity.X += deltaX * reflection * influenceA
	ballA.Velocity.Y += deltaY * reflection * influenceA
	ballB.Velocity.X -= deltaX * reflection * influenceB
	ballB.Velocity.Y -= deltaY * reflection * influenceB

	var (
		dist             float64 = math.Sqrt(distanceSquared)
		overlap, distDiv float64 = radiusSum - dist, 1 / dist
	)

	ballA.Position.X += deltaX * distDiv * overlap * influenceA
	ballA.Position.Y += deltaY * distDiv * overlap * influenceA
	ballB.Position.X -= deltaX * distDiv * overlap * influenceB
	ballB.Position.Y -= deltaY * distDiv * overlap * influenceB
	ballA.collisionIDs, ballB.collisionIDs = append(ballA.collisionIDs, ballB.ID), append(ballB.collisionIDs, ballA.ID)
	ballA.syncEntityAABB()
	ballB.syncEntityAABB()

	collided = true
	return
}

func collisionInfluence(pusher, receiver *Ball) (influence float64) {
	var (
		pusherSize, receiverSize float64 = max(pusher.Size, 0.001), max(receiver.Size, 0.001)
		sizeRatio                float64 = pusherSize / receiverSize
	)

	influence = sizeRatio * sizeRatio
	return
}

func splitCollisionInfluence(influenceA, influenceB float64) (shareA, shareB float64) {
	var total float64 = influenceA + influenceB
	if total <= 0 {
		return
	}

	shareA, shareB = influenceA/total, influenceB/total
	return
}
