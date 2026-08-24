//go:build js && wasm

package main

import (
	"fmt"

	"github.com/z46-dev/gamelib"
	"github.com/z46-dev/wasmdraw"
	"github.com/z46-dev/wasmdraw/ctx2d"
)

var (
	canvas *wasmdraw.CanvasElement
	ctx    *ctx2d.Context
	gw     *gamelib.GameWrapper
)

func main() {
	var err error

	if canvas, err = wasmdraw.GetCanvasById("canvas", wasmdraw.DefaultOptions()); err != nil {
		panic(err)
	}

	if ctx, err = ctx2d.GetContext(canvas, ctx2d.DefaultOptions()); err != nil {
		panic(err)
	}

	gw = gamelib.New(NewGame(), 20.0)
	ctx.OnFrame = drawState
	fmt.Println("Starting game loop...")

	select {}
}
