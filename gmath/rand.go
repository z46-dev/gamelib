package gmath

import (
	"math/rand/v2"

	"golang.org/x/exp/constraints"
)

func RandomRange[T constraints.Float | constraints.Integer](min, max T) (result T) {
	result = min + T(rand.Float64())*(max-min)
	return
}
