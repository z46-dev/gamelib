package physics_test

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/z46-dev/gamelib/physics"
	"github.com/z46-dev/gamelib/poly"
	"github.com/z46-dev/gamelib/vector"
)

const physicsEpsilon = 1e-9

func testWorldConfig() (config physics.WorldConfig[float64]) {
	config = physics.DefaultWorldConfig[float64]()
	config.GravityY = 10
	config.PositionCorrection = 0.8
	return
}

func testCubePolyhedron() (shape *poly.Polyhedron[float64]) {
	var (
		vertices []vector.Vec3[float64] = []vector.Vec3[float64]{
			{X: -1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: -1}, {X: 1, Y: 1, Z: -1}, {X: -1, Y: 1, Z: -1},
			{X: -1, Y: -1, Z: 1}, {X: 1, Y: -1, Z: 1}, {X: 1, Y: 1, Z: 1}, {X: -1, Y: 1, Z: 1},
		}
		triangles []poly.Triangle3 = []poly.Triangle3{
			{A: 0, B: 2, C: 1}, {A: 0, B: 3, C: 2}, {A: 4, B: 5, C: 6}, {A: 4, B: 6, C: 7},
			{A: 0, B: 1, C: 5}, {A: 0, B: 5, C: 4}, {A: 3, B: 7, C: 6}, {A: 3, B: 6, C: 2},
			{A: 0, B: 4, C: 7}, {A: 0, B: 7, C: 3}, {A: 1, B: 2, C: 6}, {A: 1, B: 6, C: 5},
		}
		err error
	)
	shape, err = poly.NewPolyhedron(vertices, triangles)
	if err != nil {
		panic(err)
	}
	return
}

func TestWorld2BodyLifecycleAndIntegration(t *testing.T) {
	var (
		world   *physics.World2[float64] = physics.NewWorld2(testWorldConfig())
		dynamic *physics.Body2[float64]
		static  *physics.Body2[float64]
		found   bool
		err     error
	)
	dynamic, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(1.0)})
	require.NoError(t, err)
	static, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewCircle2(1.0), Position: vector.Vec2[float64]{X: 100}})
	require.NoError(t, err)
	dynamic.ApplyForce(vector.Vec2[float64]{X: 2})
	world.Step(0.5)
	assert.Greater(t, dynamic.Position.X, 0.0)
	assert.Greater(t, dynamic.Position.Y, 0.0)
	assert.Equal(t, vector.Vec2[float64]{}, dynamic.Force)
	assert.Equal(t, vector.Vec2[float64]{X: 100}, static.Position)
	_, found = world.Body(dynamic.ID)
	assert.True(t, found)
	assert.True(t, world.RemoveBody(dynamic.ID))
	_, found = world.Body(dynamic.ID)
	assert.False(t, found)
}

func TestWorld2CircleCollisionAndFiltering(t *testing.T) {
	var (
		config        physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world         *physics.World2[float64]     = physics.NewWorld2(config)
		first, second *physics.Body2[float64]
		err           error
	)
	first, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(1.0), Position: vector.Vec2[float64]{X: -0.75}, Velocity: vector.Vec2[float64]{X: 1}, Material: physics.Material[float64]{Density: 1, Restitution: 1}})
	require.NoError(t, err)
	second, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(1.0), Position: vector.Vec2[float64]{X: 0.75}, Velocity: vector.Vec2[float64]{X: -1}, Material: physics.Material[float64]{Density: 1, Restitution: 1}})
	require.NoError(t, err)
	world.Step(1.0 / 60.0)
	assert.Len(t, world.Contacts, 1)
	assert.Less(t, first.Velocity.X, 0.0)
	assert.Greater(t, second.Velocity.X, 0.0)

	first.Filter.Mask = 0
	world.Step(1.0 / 60.0)
	assert.Empty(t, world.Contacts)
}

func TestWorld2CircleSettlesAgainstPolygon(t *testing.T) {
	var (
		config physics.WorldConfig[float64] = testWorldConfig()
		world  *physics.World2[float64]     = physics.NewWorld2(config)
		ball   *physics.Body2[float64]
		err    error
	)
	_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -10, Y: -0.5}, {X: 10, Y: -0.5}, {X: 10, Y: 0.5}, {X: -10, Y: 0.5}}, 1), Position: vector.Vec2[float64]{Y: 5}})
	require.NoError(t, err)
	ball, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(0.5), Position: vector.Vec2[float64]{Y: 0}, Material: physics.Material[float64]{Density: 1}})
	require.NoError(t, err)
	for range 180 {
		world.Step(1.0 / 120.0)
	}
	assert.LessOrEqual(t, ball.Position.Y, 4.1)
	assert.Greater(t, ball.Position.Y, 3.5)
}

func TestWorld3BodyIntegrationAndSphereCollision(t *testing.T) {
	var (
		config        physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world         *physics.World3[float64]
		first, second *physics.Body3[float64]
		err           error
	)
	config.GravityZ = -10
	world = physics.NewWorld3(config)
	first, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(1.0), Position: vector.Vec3[float64]{X: -0.75}, Velocity: vector.Vec3[float64]{X: 1}, Material: physics.Material[float64]{Density: 1, Restitution: 1}})
	require.NoError(t, err)
	second, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(1.0), Position: vector.Vec3[float64]{X: 0.75}, Velocity: vector.Vec3[float64]{X: -1}, Material: physics.Material[float64]{Density: 1, Restitution: 1}})
	require.NoError(t, err)
	world.Step(1.0 / 60.0)
	assert.Len(t, world.Contacts, 1)
	assert.Less(t, first.Velocity.X, 0.0)
	assert.Greater(t, second.Velocity.X, 0.0)
	assert.Less(t, first.Position.Z, 0.0)
}

func TestWorld3SphereCollidesWithPolyhedron(t *testing.T) {
	var (
		config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world  *physics.World3[float64]     = physics.NewWorld3(config)
		sphere *physics.Body3[float64]
		err    error
	)
	_, err = world.AddBody(physics.Body3Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolyhedron3(testCubePolyhedron(), vector.Vec3[float64]{X: 1, Y: 1, Z: 1})})
	require.NoError(t, err)
	sphere, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(0.75), Position: vector.Vec3[float64]{X: 1.4}, Velocity: vector.Vec3[float64]{X: -1}})
	require.NoError(t, err)
	world.Step(1.0 / 120.0)
	assert.NotEmpty(t, world.Contacts)
	assert.GreaterOrEqual(t, sphere.Position.X, 1.39)
}

func TestShapeMassPropertiesAndCloning(t *testing.T) {
	var (
		circle        *physics.Circle2[float64] = physics.NewCircle2(2.0)
		sphere        *physics.Sphere3[float64] = physics.NewSphere3(2.0)
		world2        *physics.World2[float64]  = physics.NewWorld2(physics.DefaultWorldConfig[float64]())
		first, second *physics.Body2[float64]
		err           error
	)
	assert.InDelta(t, 4*3.141592653589793, circle.Area(), physicsEpsilon)
	assert.InDelta(t, 4.0/3.0*3.141592653589793*8, sphere.Volume(), physicsEpsilon)
	first, err = world2.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: circle})
	require.NoError(t, err)
	second, err = world2.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: circle, Position: vector.Vec2[float64]{X: 10}})
	require.NoError(t, err)
	first.SetTransform(vector.Vec2[float64]{X: 3}, 0)
	assert.NotEqual(t, first.GetAABB().X1, second.GetAABB().X1)
	assert.Greater(t, first.Mass(), 0.0)
}

func TestQuaternionRotationAndBodyOrientation(t *testing.T) {
	var (
		rotation physics.Quaternion[float64] = physics.QuaternionFromEuler(vector.Vec3[float64]{Z: math.Pi / 2})
		rotated  vector.Vec3[float64]        = rotation.Rotate(vector.Vec3[float64]{X: 1})
		world    *physics.World3[float64]    = physics.NewWorld3(physics.DefaultWorldConfig[float64]())
		body     *physics.Body3[float64]
		err      error
	)
	assert.InDelta(t, 0, rotated.X, physicsEpsilon)
	assert.InDelta(t, 1, rotated.Y, physicsEpsilon)
	assert.InDelta(t, 0, rotated.Z, physicsEpsilon)
	body, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(1.0), AngularVelocity: vector.Vec3[float64]{Z: math.Pi}, DisableGravity: true})
	require.NoError(t, err)
	world.Step(0.5)
	assert.NotEqual(t, physics.IdentityQuaternion[float64](), body.Orientation)
	assert.InDelta(t, 1, body.Orientation.X*body.Orientation.X+body.Orientation.Y*body.Orientation.Y+body.Orientation.Z*body.Orientation.Z+body.Orientation.W*body.Orientation.W, physicsEpsilon)
}

func TestBodiesSleepAndWake(t *testing.T) {
	var (
		config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world2 *physics.World2[float64]     = physics.NewWorld2(config)
		world3 *physics.World3[float64]     = physics.NewWorld3(config)
		body2  *physics.Body2[float64]
		body3  *physics.Body3[float64]
		err    error
	)
	body2, err = world2.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(1.0), DisableGravity: true})
	require.NoError(t, err)
	body3, err = world3.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(1.0), DisableGravity: true})
	require.NoError(t, err)
	world2.Step(0.6)
	world3.Step(0.6)
	assert.True(t, body2.Sleeping)
	assert.True(t, body3.Sleeping)
	body2.ApplyImpulse(vector.Vec2[float64]{X: 1})
	body3.ApplyImpulse(vector.Vec3[float64]{X: 1})
	assert.False(t, body2.Sleeping)
	assert.False(t, body3.Sleeping)
}

func TestBreakableDistanceConstraints(t *testing.T) {
	var (
		world2         *physics.World2[float64] = physics.NewWorld2(physics.DefaultWorldConfig[float64]())
		anchor2, body2 *physics.Body2[float64]
		constraint2    *physics.DistanceConstraint2[float64]
		world3         *physics.World3[float64] = physics.NewWorld3(physics.DefaultWorldConfig[float64]())
		anchor3, body3 *physics.Body3[float64]
		constraint3    *physics.DistanceConstraint3[float64]
		err            error
	)
	anchor2, err = world2.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewCircle2(0.1)})
	require.NoError(t, err)
	body2, err = world2.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(0.1), Position: vector.Vec2[float64]{X: 3}, DisableGravity: true})
	require.NoError(t, err)
	constraint2, err = world2.AddDistanceConstraint(physics.DistanceConstraintConfig[float64]{First: anchor2.ID, Second: body2.ID, RestLength: 1, BreakForce: 0.1})
	require.NoError(t, err)
	world2.Step(1.0 / 60.0)
	assert.True(t, constraint2.Broken)
	constraint2.BreakForce = 0
	constraint2.Repair(1)
	assert.False(t, constraint2.Broken)

	anchor3, err = world3.AddBody(physics.Body3Config[float64]{Type: physics.StaticBody, Shape: physics.NewSphere3(0.1)})
	require.NoError(t, err)
	body3, err = world3.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(0.1), Position: vector.Vec3[float64]{Z: 3}, DisableGravity: true})
	require.NoError(t, err)
	constraint3, err = world3.AddDistanceConstraint(physics.DistanceConstraintConfig[float64]{First: anchor3.ID, Second: body3.ID, RestLength: 1})
	require.NoError(t, err)
	world3.Step(1.0 / 60.0)
	assert.InDelta(t, 1, body3.Position.Z, 0.01)
	assert.True(t, world3.RemoveConstraint(constraint3.ID))
}
