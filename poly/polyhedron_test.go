package poly_test

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
)

func cubeMesh() (vertices []vector.Vec3[float64], triangles []poly.Triangle3) {
	vertices = []vector.Vec3[float64]{
		{X: -1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: -1},
		{X: 1, Y: 1, Z: -1}, {X: -1, Y: 1, Z: -1},
		{X: -1, Y: -1, Z: 1}, {X: 1, Y: -1, Z: 1},
		{X: 1, Y: 1, Z: 1}, {X: -1, Y: 1, Z: 1},
	}

	triangles = []poly.Triangle3{
		{A: 0, B: 2, C: 1}, {A: 0, B: 3, C: 2},
		{A: 4, B: 5, C: 6}, {A: 4, B: 6, C: 7},
		{A: 0, B: 1, C: 5}, {A: 0, B: 5, C: 4},
		{A: 3, B: 7, C: 6}, {A: 3, B: 6, C: 2},
		{A: 0, B: 4, C: 7}, {A: 0, B: 7, C: 3},
		{A: 1, B: 2, C: 6}, {A: 1, B: 6, C: 5},
	}

	return
}

func concavePrismMesh() (vertices []vector.Vec3[float64], triangles []poly.Triangle3) {
	var footprint [][2]float64 = [][2]float64{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}
	vertices = make([]vector.Vec3[float64], len(footprint)*2)
	for i := range footprint {
		vertices[i] = vector.Vec3[float64]{X: footprint[i][0], Y: footprint[i][1], Z: 0}
		vertices[i+len(footprint)] = vector.Vec3[float64]{X: footprint[i][0], Y: footprint[i][1], Z: 1}
	}

	var caps []poly.Triangle3 = []poly.Triangle3{{A: 0, B: 1, C: 3}, {A: 1, B: 2, C: 3}, {A: 0, B: 3, C: 5}, {A: 3, B: 4, C: 5}}
	for i := range caps {
		triangles = append(triangles,
			poly.Triangle3{A: caps[i].A, B: caps[i].C, C: caps[i].B},
			poly.Triangle3{A: caps[i].A + len(footprint), B: caps[i].B + len(footprint), C: caps[i].C + len(footprint)},
		)
	}

	for i := range footprint {
		var next int = (i + 1) % len(footprint)
		triangles = append(triangles,
			poly.Triangle3{A: i, B: next, C: next + len(footprint)},
			poly.Triangle3{A: i, B: next + len(footprint), C: i + len(footprint)},
		)
	}

	return
}

func TestPolyhedronConstructionTransformAndQueries(t *testing.T) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		shape     *poly.Polyhedron[float64]
		err       error
	)

	vertices, triangles = cubeMesh()
	shape, err = poly.NewPolyhedron(vertices, triangles)
	require.NoError(t, err)

	assert.Equal(t, 8, shape.VertexCount())
	assert.Equal(t, 12, shape.TriangleCount())
	assert.Equal(t, &hshg.AABB3[float64]{X1: -1, Y1: -1, Z1: -1, X2: 1, Y2: 1, Z2: 1}, shape.GetAABB())
	var minimum, maximum float64 = shape.ProjectOnto(vector.NewVec3(1.0, 0.0, 0.0))
	assert.Equal(t, -1.0, minimum)
	assert.Equal(t, 1.0, maximum)

	shape.Transform(poly.PolyhedronTransform[float64]{
		Position: vector.Vec3[float64]{X: 5, Y: -2, Z: 1},
		Scale:    vector.Vec3[float64]{X: 2, Y: 1, Z: 3},
		Rotation: vector.Vec3[float64]{Z: math.Pi / 2},
	})

	assert.InDelta(t, 4, shape.AABB.X1, epsilon)
	assert.InDelta(t, -4, shape.AABB.Y1, epsilon)
	assert.InDelta(t, -2, shape.AABB.Z1, epsilon)
	assert.InDelta(t, 6, shape.AABB.X2, epsilon)
	assert.InDelta(t, 0, shape.AABB.Y2, epsilon)
	assert.InDelta(t, 4, shape.AABB.Z2, epsilon)
}

func TestPolyhedronSupportsConcaveClosedMeshes(t *testing.T) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		shape     *poly.Polyhedron[float64]
		err       error
	)

	vertices, triangles = concavePrismMesh()
	shape, err = poly.NewPolyhedron(vertices, triangles)
	require.NoError(t, err)
	assert.Equal(t, 12, shape.VertexCount())
	assert.Equal(t, 20, shape.TriangleCount())
	assert.Equal(t, &hshg.AABB3[float64]{X2: 2, Y2: 2, Z2: 1}, shape.AABB)
	assert.True(t, shape.PointIsInside(vector.NewVec3(0.5, 1.5, 0.5)))
	assert.False(t, shape.PointIsInside(vector.NewVec3(1.5, 1.5, 0.5)), "containment must preserve the concave notch")
}

func TestPolyhedronMeasurementsAndSurfaceQueries(t *testing.T) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		shape     *poly.Polyhedron[float64]
		normal    vector.Vec3[float64]
		centroid  vector.Vec3[float64]
		closest   vector.Vec3[float64]
		err       error
		ok        bool
	)

	vertices, triangles = cubeMesh()
	shape, err = poly.NewPolyhedron(vertices, triangles)
	require.NoError(t, err)

	assert.InDelta(t, 24, shape.SurfaceArea(), epsilon)
	assert.InDelta(t, 8, shape.Volume(), epsilon)
	assert.NotZero(t, shape.SignedVolume())
	centroid = shape.Centroid()
	assert.InDelta(t, 0, centroid.X, epsilon)
	assert.InDelta(t, 0, centroid.Y, epsilon)
	assert.InDelta(t, 0, centroid.Z, epsilon)

	normal, ok = shape.FaceNormal(0)
	assert.True(t, ok)
	assert.InDelta(t, -1, normal.Z, epsilon)
	_, ok = shape.FaceNormal(-1)
	assert.False(t, ok)

	closest = shape.ClosestPoint(vector.NewVec3(3.0, 0.25, -0.5))
	assert.InDelta(t, 1, closest.X, epsilon)
	assert.InDelta(t, 0.25, closest.Y, epsilon)
	assert.InDelta(t, -0.5, closest.Z, epsilon)
	assert.True(t, shape.PointIsInside(vector.NewVec3(0.0, 0.0, 0.0)))
	assert.True(t, shape.PointIsInside(vector.NewVec3(1.0, 0.0, 0.0)), "surface points count as inside")
	assert.False(t, shape.PointIsInside(vector.NewVec3(2.0, 0.0, 0.0)))
}

func TestPolyhedronRayLineAndSphereQueries(t *testing.T) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		shape     *poly.Polyhedron[float64]
		point     vector.Vec3[float64]
		distance  float64
		err       error
		hit       bool
	)

	vertices, triangles = cubeMesh()
	shape, err = poly.NewPolyhedron(vertices, triangles)
	require.NoError(t, err)

	point, distance, hit = shape.RayIntersects(vector.NewVec3(-3.0, 0.0, 0.0), vector.NewVec3(2.0, 0.0, 0.0))
	assert.True(t, hit)
	assert.InDelta(t, 2, distance, epsilon, "distance must not depend on direction magnitude")
	assert.InDelta(t, -1, point.X, epsilon)
	_, _, hit = shape.RayIntersects(vector.NewVec3(-3.0, 2.0, 0.0), vector.NewVec3(1.0, 0.0, 0.0))
	assert.False(t, hit)
	_, _, hit = shape.RayIntersects(vector.NewVec3(0.0, 0.0, 0.0), vector.Vec3_0[float64]())
	assert.False(t, hit)

	assert.True(t, shape.LineIntersects(vector.NewVec3(-2.0, 0.0, 0.0), vector.NewVec3(2.0, 0.0, 0.0)))
	assert.True(t, shape.LineIntersects(vector.NewVec3(0.0, 0.0, 0.0), vector.NewVec3(0.5, 0.0, 0.0)))
	assert.False(t, shape.LineIntersects(vector.NewVec3(-3.0, 0.0, 0.0), vector.NewVec3(-2.0, 0.0, 0.0)))

	assert.True(t, shape.SphereIntersects(vector.NewVec3(0.0, 0.0, 0.0), 0.1))
	assert.True(t, shape.SphereIntersects(vector.NewVec3(1.5, 0.0, 0.0), 0.5))
	assert.False(t, shape.SphereIntersects(vector.NewVec3(2.0, 0.0, 0.0), 0.5))
	assert.False(t, shape.SphereIntersects(vector.NewVec3(0.0, 0.0, 0.0), -1))
}

func TestPolyhedronRejectsInvalidMeshes(t *testing.T) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		err       error
	)

	vertices, triangles = cubeMesh()

	_, err = poly.NewPolyhedron(vertices, triangles[:len(triangles)-1])
	assert.ErrorContains(t, err, "not shared")

	triangles[0].A = len(vertices)
	_, err = poly.NewPolyhedron(vertices, triangles)
	assert.ErrorContains(t, err, "out-of-range")

	vertices, triangles = cubeMesh()
	triangles[0].C = triangles[0].B
	_, err = poly.NewPolyhedron(vertices, triangles)
	assert.ErrorContains(t, err, "repeats")
}

func TestPolyhedronCopiesInputAndState(t *testing.T) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		shape     *poly.Polyhedron[float64]
		clone     *poly.Polyhedron[float64]
		err       error
	)

	vertices, triangles = cubeMesh()
	shape, err = poly.NewPolyhedron(vertices, triangles)
	require.NoError(t, err)
	vertices[0].X = 100
	triangles[0].A = 7
	assert.Equal(t, -1.0, shape.Reference[0].X)
	assert.Equal(t, 0, shape.Triangles[0].A)

	clone = shape.Copy()
	clone.Reference[0].X = 50
	clone.Points[0].X = 50
	clone.Triangles[0].A = 7
	clone.AABB.X1 = 50
	assert.Equal(t, -1.0, shape.Reference[0].X)
	assert.Equal(t, -1.0, shape.Points[0].X)
	assert.Equal(t, 0, shape.Triangles[0].A)
	assert.Equal(t, -1.0, shape.AABB.X1)
}

func TestPolyhedronBVHAndConvexSurfaceParts(t *testing.T) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		shape     *poly.Polyhedron[float64]
		parts     []poly.PolyhedronTriangle[float64]
		err       error
	)

	vertices, triangles = cubeMesh()
	shape, err = poly.NewPolyhedron(vertices, triangles)
	require.NoError(t, err)
	parts = shape.ConvexParts()
	assert.Len(t, parts, len(triangles))
	assert.Greater(t, shape.BVHNodeCount(), 1)
	assert.InDelta(t, -1, parts[0].Normal.Z, epsilon)

	parts[0].A.X = 100
	assert.NotEqual(t, 100.0, shape.ConvexParts()[0].A.X, "returned parts must not mutate the cache")
	shape.Transform(poly.PolyhedronTransform[float64]{Position: vector.Vec3[float64]{X: 10}, Scale: vector.Vec3[float64]{X: 1, Y: 1, Z: 1}})
	assert.InDelta(t, 9, shape.ConvexParts()[0].AABB.X1, epsilon, "transforms must rebuild cached face bounds")
}

func TestPolyhedronIntersectionAndContactManifold(t *testing.T) {
	var (
		vertices  []vector.Vec3[float64]
		triangles []poly.Triangle3
		first     *poly.Polyhedron[float64]
		second    *poly.Polyhedron[float64]
		manifold  poly.PolyhedronManifold[float64]
		err       error
	)

	vertices, triangles = cubeMesh()
	first, err = poly.NewPolyhedron(vertices, triangles)
	require.NoError(t, err)
	second, err = poly.NewPolyhedron(vertices, triangles)
	require.NoError(t, err)

	second.Transform(poly.PolyhedronTransform[float64]{Position: vector.Vec3[float64]{X: 1.5}, Scale: vector.Vec3[float64]{X: 1, Y: 1, Z: 1}})
	assert.True(t, poly.TwoPolyhedraIntersect(first, second))
	manifold = poly.GetPolyhedronContactManifold(first, second)
	assert.NotEmpty(t, manifold.Contacts)
	assert.Greater(t, maximumContactPenetration(manifold.Contacts), 0.0)
	assert.NotZero(t, manifold.Contacts[0].Normal.SquaredLength())

	second.Transform(poly.PolyhedronTransform[float64]{Position: vector.Vec3[float64]{X: 2}, Scale: vector.Vec3[float64]{X: 1, Y: 1, Z: 1}})
	assert.True(t, poly.TwoPolyhedraIntersect(first, second), "touching faces count as intersecting")
	assert.NotEmpty(t, poly.GetPolyhedronContactManifold(first, second).Contacts)

	second.Transform(poly.PolyhedronTransform[float64]{Position: vector.Vec3[float64]{X: 3}, Scale: vector.Vec3[float64]{X: 1, Y: 1, Z: 1}})
	assert.False(t, poly.TwoPolyhedraIntersect(first, second))
	assert.Empty(t, poly.GetPolyhedronContactManifold(first, second).Contacts)
}

func TestPolyhedronIntersectionHandlesEnclosureAndConcavity(t *testing.T) {
	var (
		cubeVertices     []vector.Vec3[float64]
		cubeTriangles    []poly.Triangle3
		prismVertices    []vector.Vec3[float64]
		prismTriangles   []poly.Triangle3
		outer, inner     *poly.Polyhedron[float64]
		concave, inNotch *poly.Polyhedron[float64]
		err              error
	)

	cubeVertices, cubeTriangles = cubeMesh()
	outer, err = poly.NewPolyhedron(cubeVertices, cubeTriangles)
	require.NoError(t, err)
	inner, err = poly.NewPolyhedron(cubeVertices, cubeTriangles)
	require.NoError(t, err)
	outer.Transform(poly.PolyhedronTransform[float64]{Scale: vector.Vec3[float64]{X: 3, Y: 3, Z: 3}})
	inner.Transform(poly.IdentityPolyhedronTransform[float64]())
	assert.True(t, poly.TwoPolyhedraIntersect(outer, inner), "enclosure has no surface crossing but still intersects")
	assert.NotEmpty(t, poly.GetPolyhedronContactManifold(outer, inner).Contacts)

	prismVertices, prismTriangles = concavePrismMesh()
	concave, err = poly.NewPolyhedron(prismVertices, prismTriangles)
	require.NoError(t, err)
	inNotch, err = poly.NewPolyhedron(cubeVertices, cubeTriangles)
	require.NoError(t, err)
	inNotch.Transform(poly.PolyhedronTransform[float64]{Position: vector.Vec3[float64]{X: 1.5, Y: 1.5, Z: 0.5}, Scale: vector.Vec3[float64]{X: 0.2, Y: 0.2, Z: 0.2}})
	assert.False(t, poly.TwoPolyhedraIntersect(concave, inNotch), "overlapping AABBs must not fill concave voids")
}

func maximumContactPenetration(contacts []poly.PolyhedronContact[float64]) (maximum float64) {
	for i := range contacts {
		maximum = max(maximum, contacts[i].Penetration)
	}

	return
}
