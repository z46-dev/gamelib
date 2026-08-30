//go:build js && wasm

package main

import (
	"fmt"

	"github.com/z46-dev/gamelib"
	"github.com/z46-dev/wasmdraw"
	"github.com/z46-dev/wasmdraw/webgl2"
)

func main() {
	var (
		canvas   *wasmdraw.CanvasElement
		context  *webgl2.Context
		game     *Game
		renderer *Renderer
		err      error
	)

	if canvas, err = wasmdraw.GetCanvasById("canvas", wasmdraw.DefaultOptions()); err != nil {
		panic(err)
	}

	if context, err = webgl2.GetContext(canvas, webgl2.DefaultOptions().WithAlpha(false).WithDepth(true).WithAntialias(true)); err != nil {
		panic(err)
	}

	game = NewGame()
	if renderer, err = NewRenderer(canvas, context, game); err != nil {
		panic(err)
	}

	context.OnFrame = renderer.Draw
	fmt.Println("Starting 3D physics example with WebGL2...")
	gamelib.New(game, 60).Start()
}
