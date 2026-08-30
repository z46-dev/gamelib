package gamelib

import (
	"slices"
	"sync"
	"time"

	"github.com/z46-dev/gamelib/gmath"
)

type (
	GameMetrics struct {
		Ticks struct {
			current        int
			Target, Actual float64
		}

		Time struct {
			records                     []float64
			Target                      float64
			Min, Mean, Median, Max, Sum float64
		}
	}

	GameWrapper struct {
		Game               Game
		MaximumDelta       float64
		runOnce, stopOnce  sync.Once
		doneChan, stopChan chan struct{}
		ticksPerSecond     float64
		tickInterval       time.Duration
		lastTime           time.Time
		Metrics            *GameMetrics
	}

	Game interface {
		Update(dt float64) // dt is in seconds
	}
)

func New(game Game, ticksPerSecond float64) (gw *GameWrapper) {
	gw = &GameWrapper{
		Game:           game,
		MaximumDelta:   0.25,
		ticksPerSecond: ticksPerSecond,
		doneChan:       make(chan struct{}),
		stopChan:       make(chan struct{}),
		Metrics:        &GameMetrics{},
	}

	gw.tickInterval = time.Duration(float64(time.Second) / gw.ticksPerSecond)
	gw.Metrics.Ticks.Target = gw.ticksPerSecond
	gw.Metrics.Time.Target = 1.0 / gw.ticksPerSecond

	return
}

func (gw *GameWrapper) Start() {
	gw.runOnce.Do(func() {
		var (
			ticker        *time.Ticker = time.NewTicker(gw.tickInterval)
			lastTime      time.Time    = time.Now()
			secondCounter float64      = 0
		)

		defer ticker.Stop()
		defer close(gw.doneChan)

		for {
			select {
			case <-ticker.C:
				var (
					now     time.Time = time.Now()
					elapsed float64   = max(now.Sub(lastTime).Seconds(), gmath.EPSILON)
				)

				lastTime = now
				gw.updateElapsed(elapsed)

				gw.Metrics.Ticks.current++
				gw.Metrics.Time.records = append(gw.Metrics.Time.records, time.Since(lastTime).Seconds())

				if secondCounter += elapsed; secondCounter >= 1.0 {
					secondCounter = 0
					gw.Metrics.Ticks.Actual = float64(gw.Metrics.Ticks.current)
					gw.Metrics.Ticks.current = 0

					gw.Metrics.Time.Min, gw.Metrics.Time.Mean, gw.Metrics.Time.Median, gw.Metrics.Time.Max, gw.Metrics.Time.Sum = 0, 0, 0, 0, 0
					if len(gw.Metrics.Time.records) > 0 {
						slices.Sort(gw.Metrics.Time.records)
						for _, record := range gw.Metrics.Time.records {
							gw.Metrics.Time.Sum += record
							gw.Metrics.Time.Min = min(gw.Metrics.Time.Min, record)
							gw.Metrics.Time.Max = max(gw.Metrics.Time.Max, record)
						}

						gw.Metrics.Time.Mean = gw.Metrics.Time.Sum / float64(len(gw.Metrics.Time.records))
						gw.Metrics.Time.Median = gw.Metrics.Time.records[len(gw.Metrics.Time.records)/2]
					}
				}
			case <-gw.stopChan:
				return
			}
		}
	})
}

// updateElapsed advances all elapsed time in bounded chunks so lag is never discarded.
func (gw *GameWrapper) updateElapsed(elapsed float64) {
	var maximumDelta float64 = gw.MaximumDelta
	if maximumDelta <= 0 {
		maximumDelta = elapsed
	}
	for elapsed > maximumDelta {
		gw.Game.Update(maximumDelta)
		elapsed -= maximumDelta
	}
	if elapsed > 0 {
		gw.Game.Update(elapsed)
	}
}

func (gw *GameWrapper) Stop() {
	gw.stopOnce.Do(func() {
		close(gw.stopChan)
	})

	<-gw.doneChan
}

func (gw *GameWrapper) Wait() {
	<-gw.doneChan
}
