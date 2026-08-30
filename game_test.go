package gamelib_test

import (
	"sync"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/z46-dev/gamelib"
)

type delayedGame struct {
	lock    sync.Mutex
	updates []float64
	reached chan struct{}
}

func (g *delayedGame) Update(dt float64) {
	g.lock.Lock()
	g.updates = append(g.updates, dt)
	var count int = len(g.updates)
	g.lock.Unlock()
	if count == 1 {
		time.Sleep(80 * time.Millisecond)
	}
	if count == 2 {
		close(g.reached)
	}
}

func TestGameWrapperCatchUpPreservesAllElapsedTime(t *testing.T) {
	var stalled *catchUpGame = &catchUpGame{updates: make(chan float64, 64)}
	var wrapper *gamelib.GameWrapper = gamelib.New(stalled, 100)
	wrapper.MaximumDelta = 0.025
	go wrapper.Start()
	var total float64
	for total < 0.075 {
		select {
		case dt := <-stalled.updates:
			total += dt
		case <-time.After(time.Second):
			require.FailNow(t, "game wrapper did not catch up")
		}
	}
	wrapper.Stop()
	assert.LessOrEqual(t, stalled.maximum, wrapper.MaximumDelta)
	assert.GreaterOrEqual(t, total, 0.075)
}

type catchUpGame struct {
	updates chan float64
	maximum float64
	once    sync.Once
}

func (g *catchUpGame) Update(dt float64) {
	g.maximum = max(g.maximum, dt)
	g.updates <- dt
	g.once.Do(func() { time.Sleep(80 * time.Millisecond) })
}

func TestGameWrapperPreservesBoundedTickerDelay(t *testing.T) {
	var game *delayedGame = &delayedGame{reached: make(chan struct{})}
	var wrapper *gamelib.GameWrapper = gamelib.New(game, 100)
	go wrapper.Start()
	select {
	case <-game.reached:
	case <-time.After(time.Second):
		require.FailNow(t, "game wrapper did not resume after delayed update")
	}
	wrapper.Stop()
	game.lock.Lock()
	defer game.lock.Unlock()
	require.GreaterOrEqual(t, len(game.updates), 2)
	assert.InDelta(t, .01, game.updates[0], .01)
	assert.Greater(t, game.updates[1], .05)
	assert.LessOrEqual(t, game.updates[1], wrapper.MaximumDelta)
}
