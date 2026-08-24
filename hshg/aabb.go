package hshg

import "github.com/z46-dev/gamelib/vector"

func (a *AABB[T]) Contains(x, y T) (contains bool) {
	contains = x >= a.X1 && x <= a.X2 && y >= a.Y1 && y <= a.Y2
	return
}

func (a *AABB[T]) Intersects(b *AABB[T]) (intersects bool) {
	intersects = a.X1 <= b.X2 && a.X2 >= b.X1 && a.Y1 <= b.Y2 && a.Y2 >= b.Y1
	return
}

func (a *AABB[T]) GetCenter() (center *vector.Vec2[T]) {
	center = vector.NewVec2((a.X1+a.X2)/2, (a.Y1+a.Y2)/2)
	return
}
