# Browser examples

The `2d` example is a canvas physics playground. The `3d` example simulates a
pile of mixed rigid bodies and renders it with the WebGL2 package from
`wasmdraw`. Right-drag the 3D canvas to orbit the camera, or left-drag a body
to move it.

Build both WebAssembly programs from the repository root:

```powershell
./example/build.ps1
```

On macOS or Linux, use `./example/build.bash` instead. Serve the repository
with an HTTP server, then open `example/2d/public/` or `example/3d/public/` in a
browser. WebAssembly cannot be loaded directly from a `file://` URL.
