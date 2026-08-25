package poly_test

import (
	"fmt"
	"testing"

	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
)

var (
	benchmarkPolyhedronIntersection bool
	benchmarkPolyhedronManifold     poly.PolyhedronManifold[float64]
	benchmarkPolyhedronPoint        vector.Vec3[float64]
	benchmarkPolyhedronDistance     float64
	benchmarkPolyhedronShape        *poly.Polyhedron[float64]
)

func newBenchmarkCube(position vector.Vec3[float64], scale float64) (shape *poly.Polyhedron[float64]) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		err       error
	)
	vertices, triangles = cubeMesh()
	shape, err = poly.NewPolyhedron(vertices, triangles)
	if err != nil {
		panic(err)
	}
	shape.Transform(poly.PolyhedronTransform[float64]{Position: position, Scale: vector.Vec3[float64]{X: scale, Y: scale, Z: scale}})
	return
}

func newBenchmarkConcavePrism() (shape *poly.Polyhedron[float64]) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		err       error
	)
	vertices, triangles = concavePrismMesh()
	shape, err = poly.NewPolyhedron(vertices, triangles)
	if err != nil {
		panic(err)
	}
	return
}

func newCompoundCubes(count int, offset vector.Vec3[float64]) (shape *poly.Polyhedron[float64]) {
	var (
		baseVertices  []vector.Vec3[float64]
		baseTriangles []poly.Triangle3
		vertices      []vector.Vec3[float64] = make([]vector.Vec3[float64], 0, count*8)
		triangles     []poly.Triangle3       = make([]poly.Triangle3, 0, count*12)
		err           error
	)
	baseVertices, baseTriangles = cubeMesh()
	for i := range count {
		var (
			vertexOffset int     = len(vertices)
			x            float64 = float64(i%8)*3 + offset.X
			y            float64 = float64((i/8)%8)*3 + offset.Y
			z            float64 = float64(i/64)*3 + offset.Z
		)
		for j := range baseVertices {
			vertices = append(vertices, vector.Vec3[float64]{X: baseVertices[j].X + x, Y: baseVertices[j].Y + y, Z: baseVertices[j].Z + z})
		}
		for j := range baseTriangles {
			triangles = append(triangles, poly.Triangle3{A: baseTriangles[j].A + vertexOffset, B: baseTriangles[j].B + vertexOffset, C: baseTriangles[j].C + vertexOffset})
		}
	}
	shape, err = poly.NewPolyhedron(vertices, triangles)
	if err != nil {
		panic(err)
	}
	return
}

func BenchmarkPolyhedronIntersection(b *testing.B) {
	b.Run("CubeOverlap", func(b *testing.B) {
		benchmarkPolyhedronPairIntersection(b, newBenchmarkCube(vector.Vec3[float64]{}, 1), newBenchmarkCube(vector.Vec3[float64]{X: 1.5}, 1))
	})
	b.Run("CubeAABBMiss", func(b *testing.B) {
		benchmarkPolyhedronPairIntersection(b, newBenchmarkCube(vector.Vec3[float64]{}, 1), newBenchmarkCube(vector.Vec3[float64]{X: 3}, 1))
	})
	b.Run("CubeEnclosed", func(b *testing.B) {
		benchmarkPolyhedronPairIntersection(b, newBenchmarkCube(vector.Vec3[float64]{}, 3), newBenchmarkCube(vector.Vec3[float64]{}, 1))
	})
	b.Run("ConcaveHit", func(b *testing.B) {
		benchmarkPolyhedronPairIntersection(b, newBenchmarkConcavePrism(), newBenchmarkCube(vector.Vec3[float64]{X: 0.5, Y: 1.5, Z: 0.5}, 0.2))
	})
	b.Run("ConcaveVoid", func(b *testing.B) {
		benchmarkPolyhedronPairIntersection(b, newBenchmarkConcavePrism(), newBenchmarkCube(vector.Vec3[float64]{X: 1.5, Y: 1.5, Z: 0.5}, 0.2))
	})
	for _, count := range []int{16, 128} {
		b.Run(fmt.Sprintf("CompoundOverlap/%dCubes", count), func(b *testing.B) {
			benchmarkPolyhedronPairIntersection(b, newCompoundCubes(count, vector.Vec3[float64]{}), newCompoundCubes(count, vector.Vec3[float64]{X: 0.5}))
		})
		b.Run(fmt.Sprintf("CompoundAABBMiss/%dCubes", count), func(b *testing.B) {
			benchmarkPolyhedronPairIntersection(b, newCompoundCubes(count, vector.Vec3[float64]{}), newCompoundCubes(count, vector.Vec3[float64]{Z: 100}))
		})
	}
}

func BenchmarkPolyhedronConstructionAndBVHBuild(b *testing.B) {
	b.ReportAllocs()
	for b.Loop() {
		benchmarkPolyhedronShape = newCompoundCubes(128, vector.Vec3[float64]{})
	}
}

func benchmarkPolyhedronPairIntersection(b *testing.B, first, second *poly.Polyhedron[float64]) {
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		benchmarkPolyhedronIntersection = poly.TwoPolyhedraIntersect(first, second)
	}
}

func BenchmarkPolyhedronContactManifold(b *testing.B) {
	var (
		first  *poly.Polyhedron[float64] = newBenchmarkCube(vector.Vec3[float64]{}, 1)
		second *poly.Polyhedron[float64] = newBenchmarkCube(vector.Vec3[float64]{X: 1.5}, 1)
	)
	b.Run("Allocate", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			benchmarkPolyhedronManifold = poly.GetPolyhedronContactManifold(first, second)
		}
	})
	b.Run("Reuse", func(b *testing.B) {
		var contacts []poly.PolyhedronContact[float64] = make([]poly.PolyhedronContact[float64], 0, 32)
		b.ReportAllocs()
		for b.Loop() {
			benchmarkPolyhedronManifold = poly.GetPolyhedronContactManifoldInto(first, second, contacts)
			contacts = benchmarkPolyhedronManifold.Contacts
		}
	})
}

func BenchmarkPolyhedronQueries(b *testing.B) {
	var shape *poly.Polyhedron[float64] = newCompoundCubes(128, vector.Vec3[float64]{})
	b.Run("Raycast/128Cubes", func(b *testing.B) {
		var (
			origin    *vector.Vec3[float64] = vector.NewVec3(-10.0, 0.0, 0.0)
			direction *vector.Vec3[float64] = vector.NewVec3(1.0, 0.0, 0.0)
			hit       bool
		)
		b.ReportAllocs()
		for b.Loop() {
			benchmarkPolyhedronPoint, benchmarkPolyhedronDistance, hit = shape.RayIntersects(origin, direction)
			benchmarkPolyhedronIntersection = hit
		}
	})
	b.Run("ClosestPoint/128Cubes", func(b *testing.B) {
		var point *vector.Vec3[float64] = vector.NewVec3(11.5, 11.5, 10.0)
		b.ReportAllocs()
		for b.Loop() {
			benchmarkPolyhedronPoint = shape.ClosestPoint(point)
		}
	})
	b.Run("TransformAndRebuild/128Cubes", func(b *testing.B) {
		var transform poly.PolyhedronTransform[float64] = poly.IdentityPolyhedronTransform[float64]()
		b.ReportAllocs()
		for b.Loop() {
			transform.Rotation.Z += 0.001
			shape.Transform(transform)
		}
	})
}
