package gamelib

import "golang.org/x/exp/constraints"

type AABB[T constraints.Float] struct {
	X1, Y1, X2, Y2 T
}

func (a *AABB[T]) Contains(x, y T) (contains bool) {
	contains = x >= a.X1 && x <= a.X2 && y >= a.Y1 && y <= a.Y2
	return
}

func (a *AABB[T]) Intersects(b *AABB[T]) (intersects bool) {
	intersects = a.X1 <= b.X2 && a.X2 >= b.X1 && a.Y1 <= b.Y2 && a.Y2 >= b.Y1
	return
}

func (a *AABB[T]) GetCenter() (center *Vec2[T]) {
	center = NewVec2((a.X1+a.X2)/2, (a.Y1+a.Y2)/2)
	return
}
