//go:build js && wasm

package main

import (
	"github.com/z46-dev/gamelib"
	"github.com/z46-dev/gamelib/gmath"
	"github.com/z46-dev/gamelib/vector"
	"github.com/z46-dev/wasmdraw/ctx2d"
)

func NewSimulation() (sim *Simulation) {
	sim = &Simulation{
		WorldSize: vector.NewLerpV2(vector.NewVec2[float64](1, 1)),
		Objects:   gamelib.NewCollection[*SimulationObject](),
	}

	sim.Metrics.UPS = 60
	return
}

func (sim *Simulation) Draw(ctx *ctx2d.Context, dt, width, height float64) {
	sim.Metrics.timer += dt
	if sim.Metrics.timer >= 1.0 {
		sim.Metrics.timer -= 1.0

		sim.Metrics.UPS, sim.Metrics.FPS = sim.Metrics.observedUPS, sim.Metrics.observedFPS
		sim.Metrics.observedUPS, sim.Metrics.observedFPS = 0, 0
	}

	var lerpConstant float64 = float64(sim.Metrics.UPS) * dt

	sim.WorldSize.Update(lerpConstant)

	// Update render objects.
	sim.Objects.Lock.Lock()

	for _, toAdd := range sim.Objects.ToAppend {
		sim.Objects.Items[toAdd.ID] = toAdd
	}

	for _, toRemove := range sim.Objects.ToRemove {
		delete(sim.Objects.Items, toRemove)
	}

	sim.Objects.ToAppend = sim.Objects.ToAppend[:0]
	sim.Objects.ToRemove = sim.Objects.ToRemove[:0]

	sim.Objects.Lock.Unlock()
	sim.Objects.Lock.RLock()
	defer sim.Objects.Lock.RUnlock()

	// Draw world

	ctx.FillStyle = "#222222"
	ctx.FillRect(0, 0, width, height)

	// Translate so 0,0 is the center of the screen
	ctx.Save()
	ctx.Translate(width*0.5, height*0.5)

	ctx.FillStyle = "#444444"
	ctx.FillRect(-sim.WorldSize.X*0.5, -sim.WorldSize.Y*0.5, sim.WorldSize.X, sim.WorldSize.Y)

	for _, object := range sim.Objects.Items {
		object.Update(lerpConstant)
	}
	sim.LinksLock.RLock()
	ctx.LineWidth = 3
	for _, link := range sim.Links {
		if link.Broken {
			continue
		}
		var first, second *SimulationObject = sim.Objects.Items[link.First], sim.Objects.Items[link.Second]
		if first == nil || second == nil {
			continue
		}
		ctx.BeginPath().MoveTo(first.Position.X, first.Position.Y).LineTo(second.Position.X, second.Position.Y)
		ctx.StrokeStyle = "#D8C77B"
		ctx.Stroke()
	}
	sim.LinksLock.RUnlock()
	for _, object := range sim.Objects.Items {
		object.Draw(ctx)
	}

	ctx.Restore()
}

// Update advances an object's interpolated render transform.
func (object *SimulationObject) Update(lerpConstant float64) {
	object.Position.Update(lerpConstant)
	object.Radius.Update(lerpConstant)
	object.Rotation.Update(lerpConstant)
}

// Draw renders either a circle or local-space polygon.
func (object *SimulationObject) Draw(ctx *ctx2d.Context) {
	ctx.Save().Translate(object.Position.X, object.Position.Y).Rotate(object.Rotation.X)
	ctx.BeginPath()
	if object.Radius.X > 0 {
		ctx.Arc(0, 0, object.Radius.X, 0, gmath.TAU)
	} else if len(object.Points) > 0 {
		ctx.MoveTo(object.Points[0].X, object.Points[0].Y)
		for _, point := range object.Points[1:] {
			ctx.LineTo(point.X, point.Y)
		}
		ctx.ClosePath()
	}
	ctx.FillStyle = object.Color
	ctx.Fill()
	ctx.Restore()
}
