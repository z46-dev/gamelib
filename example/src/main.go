//go:build js && wasm

package main

import (
	"fmt"
	"syscall/js"

	"github.com/z46-dev/gamelib"
	"github.com/z46-dev/gamelib/vector"
	"github.com/z46-dev/wasmdraw"
	"github.com/z46-dev/wasmdraw/ctx2d"
)

var (
	canvas *wasmdraw.CanvasElement
	ctx    *ctx2d.Context
	gw     *gamelib.GameWrapper
	sim    *Simulation
)

func main() {
	var (
		game *Game
		err  error
	)

	if canvas, err = wasmdraw.GetCanvasById("canvas", wasmdraw.DefaultOptions()); err != nil {
		panic(err)
	}

	if ctx, err = ctx2d.GetContext(canvas, ctx2d.DefaultOptions()); err != nil {
		panic(err)
	}

	sim = NewSimulation()
	game = NewGame(sim)
	gw = gamelib.New(game, 60.0)
	var (
		lastPointerX, lastPointerY float64
		hasLastPointer             bool
	)
	var spawnAtPointer func(js.Value) any = func(event js.Value) (result any) {
		var (
			width, height  float64 = canvas.ElementSize()
			x, y           float64 = event.Get("clientX").Float() - width/2, event.Get("clientY").Float() - height/2
			deltaX, deltaY float64 = x - lastPointerX, y - lastPointerY
		)
		if !hasLastPointer || deltaX*deltaX+deltaY*deltaY >= 24*24 {
			game.QueueSpawn(vector.Vec2[float64]{X: x, Y: y})
			lastPointerX, lastPointerY, hasLastPointer = x, y, true
		}
		return
	}
	canvas.AddEventListener(wasmdraw.EventType_MouseDown, func(event js.Value) (result any) { hasLastPointer = false; return spawnAtPointer(event) })
	canvas.AddEventListener(wasmdraw.EventType_MouseMove, func(event js.Value) (result any) {
		if event.Get("buttons").Int()&1 != 0 {
			return spawnAtPointer(event)
		}
		return
	})
	ctx.OnFrame = func(dt, width, height float64) {
		sim.Draw(ctx, dt, width, height)
	}

	fmt.Println("Starting game loop...")
	gw.Start()
	gw.Wait()
}
