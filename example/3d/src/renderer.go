//go:build js && wasm

package main

import (
	"math"
	"syscall/js"

	"github.com/z46-dev/gamelib/physics"
	"github.com/z46-dev/gamelib/vector"
	"github.com/z46-dev/wasmdraw"
	"github.com/z46-dev/wasmdraw/webgl"
	"github.com/z46-dev/wasmdraw/webgl2"
)

const vertexShader = `#version 300 es
precision highp float;
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
uniform mat4 viewProjection;
out vec3 surfaceNormal;
out vec3 surfaceColor;
void main() {
    gl_Position = viewProjection * vec4(position, 1.0);
    surfaceNormal = normal;
    surfaceColor = color;
}`

const fragmentShader = `#version 300 es
precision highp float;
in vec3 surfaceNormal;
in vec3 surfaceColor;
out vec4 outputColor;
void main() {
    vec3 lightDirection = normalize(vec3(-0.45, 0.8, 0.65));
    float diffuse = max(dot(normalize(surfaceNormal), lightDirection), 0.0);
    outputColor = vec4(surfaceColor * (0.24 + diffuse * 0.76), 1.0);
}`

type (
	mesh struct {
		vertices []vector.Vec3[float64]
		normals  []vector.Vec3[float64]
	}

	Renderer struct {
		context                  *webgl2.Context
		game                     *Game
		program                  *webgl.Program
		array                    *webgl2.VertexArray
		buffer                   *webgl.Buffer
		viewProjection           *webgl.UniformLocation
		meshes                   [4]mesh
		yaw, pitch, lastX, lastY float64
		width, height            float64
		orbiting                 bool
		dragged                  physics.BodyID
		snapshot                 []RenderBody
		vertices                 []float32
	}
	matrix [16]float32
)

// NewRenderer initializes depth-correct triangle meshes and mouse controls.
func NewRenderer(canvas *wasmdraw.CanvasElement, context *webgl2.Context, game *Game) (renderer *Renderer, err error) {
	renderer = &Renderer{context: context, game: game, array: context.CreateVertexArray(), buffer: context.CreateBuffer(), yaw: .55, pitch: -.28}
	if renderer.program, err = compileProgram(context, vertexShader, fragmentShader); err != nil {
		return
	}
	renderer.viewProjection = context.GetUniformLocation(renderer.program, "viewProjection")
	renderer.meshes = [4]mesh{sphereMesh(16, 10), polyhedronMesh(SphereMesh), polyhedronMesh(TetrahedronMesh), polyhedronMesh(OctahedronMesh)}
	context.BindVertexArray(renderer.array).BindBuffer(webgl.ArrayBuffer, renderer.buffer)
	context.EnableVertexAttribArray(0).VertexAttribPointer(0, 3, webgl.Float, false, 36, 0)
	context.EnableVertexAttribArray(1).VertexAttribPointer(1, 3, webgl.Float, false, 36, 12)
	context.EnableVertexAttribArray(2).VertexAttribPointer(2, 3, webgl.Float, false, 36, 24)
	context.BindVertexArray(nil).ClearColor(.035, .045, .075, 1)
	renderer.addMouseControls(canvas)
	return
}

// Draw builds transformed geometry and renders it with a conventional depth buffer.
func (renderer *Renderer) Draw(dt, width, height float64) {
	_ = dt
	renderer.width, renderer.height = width, height
	renderer.snapshot = renderer.game.ReadSnapshot(renderer.snapshot)
	renderer.vertices = renderer.buildVertices(renderer.vertices[:0])
	var transform matrix = cameraMatrix(width/max(height, 1), renderer.yaw, renderer.pitch)
	renderer.context.Viewport(0, 0, int(width), int(height)).Enable(webgl.DepthTest).DepthFunc(webgl.LEqual)
	renderer.context.Clear(webgl.ColorBufferBit | webgl.DepthBufferBit).UseProgram(renderer.program)
	renderer.context.BindVertexArray(renderer.array).BindBuffer(webgl.ArrayBuffer, renderer.buffer)
	renderer.context.BufferDataFloat32(webgl.ArrayBuffer, renderer.vertices, webgl.DynamicDraw)
	renderer.context.UniformMatrix4fv(renderer.viewProjection, false, transform[:])
	renderer.context.DrawArrays(webgl.Triangles, 0, len(renderer.vertices)/9)
	renderer.context.BindVertexArray(nil)
}

func (renderer *Renderer) buildVertices(destination []float32) (vertices []float32) {
	vertices = destination
	for _, body := range renderer.snapshot {
		var source mesh = renderer.meshes[body.Mesh]
		for index, local := range source.vertices {
			var scaled vector.Vec3[float64] = vector.Vec3[float64]{X: local.X * body.Scale.X, Y: local.Y * body.Scale.Y, Z: local.Z * body.Scale.Z}
			var position vector.Vec3[float64] = body.Orientation.Rotate(scaled)
			position.Add(&body.Position)
			var normal vector.Vec3[float64] = vector.Vec3[float64]{X: source.normals[index].X / body.Scale.X, Y: source.normals[index].Y / body.Scale.Y, Z: source.normals[index].Z / body.Scale.Z}
			normal = body.Orientation.Rotate(normal)
			normal.Normalize()
			vertices = append(vertices, float32(position.X), float32(position.Y), float32(position.Z), float32(normal.X), float32(normal.Y), float32(normal.Z), body.Color[0], body.Color[1], body.Color[2])
		}
	}
	return
}

func (renderer *Renderer) addMouseControls(canvas *wasmdraw.CanvasElement) {
	canvas.AddEventListener(wasmdraw.EventType_MouseDown, func(event js.Value) (result any) {
		renderer.lastX, renderer.lastY = event.Get("clientX").Float(), event.Get("clientY").Float()
		if event.Get("button").Int() == 2 {
			renderer.orbiting = true
			event.Call("preventDefault")
		} else if event.Get("button").Int() == 0 {
			renderer.dragged = renderer.pick(renderer.lastX, renderer.lastY)
		}
		return
	})
	canvas.AddEventListener(wasmdraw.EventType_MouseUp, func(event js.Value) (result any) {
		if event.Get("button").Int() == 2 {
			renderer.orbiting = false
		} else if event.Get("button").Int() == 0 {
			renderer.dragged = 0
		}
		return
	})
	canvas.AddEventListener(wasmdraw.EventType_MouseMove, func(event js.Value) (result any) {
		var (
			x, y   float64 = event.Get("clientX").Float(), event.Get("clientY").Float()
			dx, dy float64 = x - renderer.lastX, y - renderer.lastY
		)
		if renderer.orbiting {
			renderer.yaw += dx * .008
			renderer.pitch = max(-1.52, min(1.52, renderer.pitch+dy*.008))
		} else if renderer.dragged != 0 {
			var speed float64 = .014
			renderer.game.MoveBody(renderer.dragged, vector.Vec3[float64]{X: dx * math.Cos(renderer.yaw) * speed, Y: -dy * speed, Z: dx * math.Sin(renderer.yaw) * speed})
		}
		renderer.lastX, renderer.lastY = x, y
		return
	})
}

func (renderer *Renderer) pick(x, y float64) (id physics.BodyID) {
	var transform matrix = cameraMatrix(renderer.width/max(renderer.height, 1), renderer.yaw, renderer.pitch)
	var closest float64 = math.MaxFloat64
	for _, body := range renderer.snapshot {
		if !body.Movable {
			continue
		}
		var clip [4]float64 = transformPoint(transform, body.Position)
		if clip[3] <= 0 {
			continue
		}
		var (
			screenX float64 = (clip[0]/clip[3]*.5 + .5) * renderer.width
			screenY float64 = (1 - (clip[1]/clip[3]*.5 + .5)) * renderer.height
			radius  float64 = max(8, body.Bound*renderer.height*.9/clip[3])
			dx, dy  float64 = x - screenX, y - screenY
		)
		if dx*dx+dy*dy <= radius*radius && clip[3] < closest {
			id, closest = body.ID, clip[3]
		}
	}
	return
}

func sphereMesh(columns, rows int) (result mesh) {
	for row := range rows {
		for column := range columns {
			var (
				latitudeA  float64              = float64(row)*math.Pi/float64(rows) - math.Pi/2
				latitudeB  float64              = float64(row+1)*math.Pi/float64(rows) - math.Pi/2
				longitudeA float64              = float64(column) * 2 * math.Pi / float64(columns)
				longitudeB float64              = float64(column+1) * 2 * math.Pi / float64(columns)
				a          vector.Vec3[float64] = spherePoint(latitudeA, longitudeA)
				b          vector.Vec3[float64] = spherePoint(latitudeB, longitudeA)
				c          vector.Vec3[float64] = spherePoint(latitudeB, longitudeB)
				d          vector.Vec3[float64] = spherePoint(latitudeA, longitudeB)
			)
			result.vertices = append(result.vertices, a, b, c, a, c, d)
			result.normals = append(result.normals, a, b, c, a, c, d)
		}
	}
	return
}

func spherePoint(latitude, longitude float64) (point vector.Vec3[float64]) {
	point = vector.Vec3[float64]{X: math.Cos(latitude) * math.Cos(longitude), Y: math.Sin(latitude), Z: math.Cos(latitude) * math.Sin(longitude)}
	return
}

func polyhedronMesh(kind MeshKind) (result mesh) {
	var vertices []vector.Vec3[float64]
	var faces [][3]int
	switch kind {
	case TetrahedronMesh:
		vertices = []vector.Vec3[float64]{{X: 1, Y: 1, Z: 1}, {X: -1, Y: -1, Z: 1}, {X: -1, Y: 1, Z: -1}, {X: 1, Y: -1, Z: -1}}
		faces = [][3]int{{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}}
	case OctahedronMesh:
		vertices = []vector.Vec3[float64]{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}, {Z: 1}, {Z: -1}}
		faces = [][3]int{{0, 2, 4}, {4, 2, 1}, {1, 2, 5}, {5, 2, 0}, {4, 3, 0}, {1, 3, 4}, {5, 3, 1}, {0, 3, 5}}
	default:
		vertices = []vector.Vec3[float64]{{X: -1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: -1}, {X: 1, Y: 1, Z: -1}, {X: -1, Y: 1, Z: -1}, {X: -1, Y: -1, Z: 1}, {X: 1, Y: -1, Z: 1}, {X: 1, Y: 1, Z: 1}, {X: -1, Y: 1, Z: 1}}
		faces = [][3]int{{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2}, {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}}
	}
	for _, face := range faces {
		var a, b, c vector.Vec3[float64] = vertices[face[0]], vertices[face[1]], vertices[face[2]]
		var ab, ac vector.Vec3[float64] = b, c
		ab.Sub(&a)
		ac.Sub(&a)
		var normal vector.Vec3[float64] = ab
		normal.Cross(&ac).Normalize()
		result.vertices = append(result.vertices, a, b, c)
		result.normals = append(result.normals, normal, normal, normal)
	}
	return
}

func compileProgram(context *webgl2.Context, vertexSource, fragmentSource string) (program *webgl.Program, err error) {
	var vertex, fragment *webgl.Shader
	if vertex, err = context.CompileShader(webgl.VertexShader, vertexSource); err != nil {
		return
	}
	if fragment, err = context.CompileShader(webgl.FragmentShader, fragmentSource); err != nil {
		context.DeleteShader(vertex)
		return
	}
	program, err = context.LinkProgram(vertex, fragment)
	context.DeleteShader(vertex)
	context.DeleteShader(fragment)
	return
}

func transformPoint(transform matrix, point vector.Vec3[float64]) (result [4]float64) {
	var values [4]float64 = [4]float64{point.X, point.Y, point.Z, 1}
	for row := range 4 {
		for column := range 4 {
			result[row] += float64(transform[column*4+row]) * values[column]
		}
	}
	return
}

func cameraMatrix(aspect, yaw, pitch float64) (result matrix) {
	result = multiply(perspective(aspect, math.Pi/3, .1, 100), multiply(translation(0, .5, -15), multiply(rotationX(pitch), rotationY(yaw))))
	return
}

func perspective(aspect, fieldOfView, near, far float64) (result matrix) {
	var scale float64 = 1 / math.Tan(fieldOfView*.5)
	result[0], result[5] = float32(scale/aspect), float32(scale)
	result[10], result[11], result[14] = float32((far+near)/(near-far)), -1, float32(2*far*near/(near-far))
	return
}

func multiply(left, right matrix) (result matrix) {
	for column := range 4 {
		for row := range 4 {
			for index := range 4 {
				result[column*4+row] += left[index*4+row] * right[column*4+index]
			}
		}
	}
	return
}

func identity() (result matrix) {
	result[0], result[5], result[10], result[15] = 1, 1, 1, 1
	return
}

func translation(x, y, z float64) (result matrix) {
	result = identity()
	result[12], result[13], result[14] = float32(x), float32(y), float32(z)
	return
}

func rotationX(angle float64) (result matrix) {
	result = identity()
	var cosine, sine float32 = float32(math.Cos(angle)), float32(math.Sin(angle))
	result[5], result[6], result[9], result[10] = cosine, sine, -sine, cosine
	return
}

func rotationY(angle float64) (result matrix) {
	result = identity()
	var cosine, sine float32 = float32(math.Cos(angle)), float32(math.Sin(angle))
	result[0], result[2], result[8], result[10] = cosine, -sine, sine, cosine
	return
}
