package gamelib

import "sync"

type Collection[T any] struct {
	Items    map[uint64]T
	ToAppend []T
	ToRemove []uint64
	Lock     sync.RWMutex
}

// NewCollection creates and initializes a new Collection instance for a given type T,
// setting up the internal map and slices for managing items, additions, and removals.
func NewCollection[T any]() (collection *Collection[T]) {
	collection = &Collection[T]{
		Items:    make(map[uint64]T),
		ToAppend: make([]T, 0),
		ToRemove: make([]uint64, 0),
	}

	return
}

// Add adds an item to the ToAppend slice of the Collection,
// marking it for addition to the main Items map during the next simulation tick.
func (c *Collection[T]) Add(item T) {
	c.ToAppend = append(c.ToAppend, item)
}

// Remove adds an item's ID to the ToRemove slice of the Collection,
// marking it for removal from the main Items map during the next simulation tick.
func (c *Collection[T]) Remove(id uint64) {
	c.ToRemove = append(c.ToRemove, id)
}
