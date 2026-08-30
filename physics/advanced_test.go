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

func TestAnchoredAndFixedJoints(t *testing.T) {
	var world *physics.World2[float64] = physics.NewWorld2(physics.DefaultWorldConfig[float64]())
	var first, second *physics.Body2[float64]
	var joint physics.FixedJoint2[float64]
	var err error
	first, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewCircle2(0.25), Position: vector.Vec2[float64]{X: 0}})
	require.NoError(t, err)
	second, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(0.25), Position: vector.Vec2[float64]{X: 2}, AngularVelocity: 8})
	require.NoError(t, err)
	joint, err = world.AddFixedJoint(physics.AnchoredDistanceConstraint2Config[float64]{First: first.ID, Second: second.ID, LocalAnchorSecond: vector.Vec2[float64]{X: -2}, RestLength: 0.0001, Damping: 1})
	require.NoError(t, err)
	for range 30 {
		world.Step(1.0 / 120.0)
	}
	var firstAnchor, secondAnchor vector.Vec2[float64] = first.Position, vector.Vec2[float64]{X: second.Position.X - 2, Y: second.Position.Y}
	assert.InDelta(t, firstAnchor.X, secondAnchor.X, 0.05)
	assert.InDelta(t, 0, second.Rotation-first.Rotation, 0.05)
	assert.NotNil(t, joint.Anchor)
	assert.NotNil(t, joint.Angle)
}

func TestRopeReconnection(t *testing.T) {
	var world *physics.World2[float64] = physics.NewWorld2(physics.DefaultWorldConfig[float64]())
	var first, second *physics.Body2[float64]
	var rope *physics.Rope2[float64]
	var err error
	first, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewCircle2(0.1)})
	require.NoError(t, err)
	second, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(0.1), Position: vector.Vec2[float64]{X: 2}})
	require.NoError(t, err)
	rope, err = world.AddRope(first.ID, second.ID, physics.RopeConfig[float64]{Segments: 2, Radius: 0.1, Mass: 1, LinearDamping: 2, AngularDamping: 3, BreakForce: 1, Reconnection: physics.ReconnectWhenClose, ReconnectDistance: 2})
	require.NoError(t, err)
	rope.Constraints[0].Broken = true
	rope.UpdateReconnection()
	assert.False(t, rope.Constraints[0].Broken)
	assert.Len(t, rope.Bodies, 3)
	assert.Equal(t, 2.0, rope.Bodies[1].LinearDamping)
	assert.Equal(t, 3.0, rope.Bodies[1].AngularDamping)
}

func TestContinuousCollisionDetectionStopsEveryFastCircle(t *testing.T) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.EnableCCD, config.CCDMaxSubsteps, config.CCDMotionThreshold = true, 128, 0.05
	var world *physics.World2[float64] = physics.NewWorld2(config)
	var bullet *physics.Body2[float64]
	var err error
	_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -0.05, Y: -3}, {X: 0.05, Y: -3}, {X: 0.05, Y: 3}, {X: -0.05, Y: 3}}, 1)})
	require.NoError(t, err)
	bullet, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(0.1), Position: vector.Vec2[float64]{X: -5}, Velocity: vector.Vec2[float64]{X: 100}})
	require.NoError(t, err)
	world.Step(0.1)
	assert.Less(t, bullet.Position.X, 0.3)
}

func TestPersistentRestingContactDoesNotReapplyRestitution(t *testing.T) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.GravityY = 10
	var world *physics.World2[float64] = physics.NewWorld2(config)
	var material physics.Material[float64] = physics.Material[float64]{Density: 1, Restitution: .5, StaticFriction: .6, DynamicFriction: .4}
	var err error
	_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -5, Y: -.5}, {X: 5, Y: -.5}, {X: 5, Y: .5}, {X: -5, Y: .5}}, 1), Position: vector.Vec2[float64]{Y: 2}, Material: material})
	require.NoError(t, err)
	var body *physics.Body2[float64]
	body, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(.5), Material: material})
	require.NoError(t, err)
	for range 600 {
		world.Step(1.0 / 120.0)
	}
	assert.True(t, body.Sleeping)
	assert.InDelta(t, 1, body.Position.Y, .02)
}

func TestDensePileSettles(t *testing.T) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.GravityY, config.PositionCorrection, config.PositionIterations = 700, .8, 6
	config.SleepLinearThreshold, config.PenetrationSlop = 1, .1
	var world *physics.World2[float64] = physics.NewWorld2(config)
	var material physics.Material[float64] = physics.Material[float64]{Density: .01, Restitution: .06, StaticFriction: .75, DynamicFriction: .55}
	var box *physics.Polygon2[float64] = physics.NewPolygon2([]vector.Vec2[float64]{{X: -10, Y: -10}, {X: 10, Y: -10}, {X: 10, Y: 10}, {X: -10, Y: 10}}, 1)
	var err error
	_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -1000, Y: -5}, {X: 1000, Y: -5}, {X: 1000, Y: 5}, {X: -1000, Y: 5}}, 1), Position: vector.Vec2[float64]{Y: 100}, Material: material})
	require.NoError(t, err)
	var bodies []*physics.Body2[float64]
	for row := range 4 {
		for column := range 4 {
			var body *physics.Body2[float64]
			body, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: box, Position: vector.Vec2[float64]{X: float64(column*21 - 31), Y: float64(row * 21)}, Rotation: float64((row+column)%2) * .01, LinearDamping: .65, AngularDamping: .9, Material: material})
			require.NoError(t, err)
			bodies = append(bodies, body)
		}
	}
	for range 3600 {
		world.Step(1.0 / 120.0)
	}
	for _, body := range bodies {
		assert.True(t, body.Sleeping)
	}
}

func TestClothAndSoftBodyPreserveShape(t *testing.T) {
	var world2 *physics.World2[float64] = physics.NewWorld2(physics.DefaultWorldConfig[float64]())
	var cloth *physics.Cloth2[float64]
	var err error
	cloth, err = world2.AddCloth(vector.Vec2[float64]{}, physics.ClothConfig[float64]{Columns: 3, Rows: 3, Spacing: 1, Radius: 0.05, Mass: 1, PinTop: true})
	require.NoError(t, err)
	assert.Len(t, cloth.Bodies, 9)
	assert.NotEmpty(t, cloth.Areas)
	var world3 *physics.World3[float64] = physics.NewWorld3(physics.DefaultWorldConfig[float64]())
	var softBody *physics.SoftBody3[float64]
	softBody, err = world3.AddSoftBody([]vector.Vec3[float64]{{}, {X: 1}, {Y: 1}, {Z: 1}}, []physics.TetrahedronIndices{{First: 0, Second: 1, Third: 2, Fourth: 3}}, physics.SoftBodyConfig[float64]{Radius: 0.05, Mass: 1})
	require.NoError(t, err)
	assert.Len(t, softBody.Constraints, 6)
	assert.Len(t, softBody.Volumes, 1)
	softBody.Bodies[3].Position.Z = 2
	for range 20 {
		world3.Step(1.0 / 120.0)
	}
	assert.Less(t, softBody.Bodies[3].Position.Z, 2.0)
}

func TestParallelIslandSolvingAndIslandSleeping(t *testing.T) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.ParallelWorkers, config.MinimumParallelIslandBodies, config.SleepTime = 4, 1, 0.02
	var world *physics.World3[float64] = physics.NewWorld3(config)
	for island := 0; island < 16; island++ {
		var first, second *physics.Body3[float64]
		var err error
		first, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(0.5), Position: vector.Vec3[float64]{X: float64(island) * 4}})
		require.NoError(t, err)
		second, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(0.5), Position: vector.Vec3[float64]{X: float64(island)*4 + 0.9}})
		require.NoError(t, err)
		_, err = world.AddDistanceConstraint(physics.DistanceConstraintConfig[float64]{First: first.ID, Second: second.ID, RestLength: 0.9})
		require.NoError(t, err)
	}
	for range 10 {
		world.Step(0.01)
	}
	for _, body := range world.Bodies() {
		assert.True(t, body.Sleeping)
	}
}

func TestSpinningContactDoesNotCreateEnergy(t *testing.T) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.EnableSleeping = false
	var world *physics.World2[float64] = physics.NewWorld2(config)
	var material physics.Material[float64] = physics.Material[float64]{Density: 1, Restitution: 0, StaticFriction: .8, DynamicFriction: .6}
	var err error
	_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -20, Y: -1}, {X: 20, Y: -1}, {X: 20, Y: 1}, {X: -20, Y: 1}}, 1), Position: vector.Vec2[float64]{Y: 2}, Material: material})
	require.NoError(t, err)
	var body *physics.Body2[float64]
	body, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -1, Y: -1}, {X: 1, Y: -1}, {X: 1, Y: 1}, {X: -1, Y: 1}}, 1), AngularVelocity: 20, Material: material})
	require.NoError(t, err)
	var maximumSpeedSquared float64
	for range 600 {
		world.Step(1.0 / 240.0)
		maximumSpeedSquared = max(maximumSpeedSquared, body.Velocity.SquaredLength()+body.AngularVelocity*body.AngularVelocity)
	}
	assert.Less(t, maximumSpeedSquared, 450.0)
	assert.Less(t, math.Abs(body.Position.X), 15.0)
}

func TestPolyhedronRestingContactDissipatesEnergy(t *testing.T) {
	var (
		config   physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
		world    *physics.World3[float64]
		material physics.Material[float64] = physics.Material[float64]{Density: 1, Restitution: 0, StaticFriction: .8, DynamicFriction: .6}
		pyramid  *poly.Polyhedron[float64]
		body     *physics.Body3[float64]
		err      error
	)
	config.GravityY, config.VelocityIterations, config.PositionIterations = -10, 16, 6
	world = physics.NewWorld3(config)
	_, err = world.AddBody(physics.Body3Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolyhedron3(testCubePolyhedron(), vector.Vec3[float64]{X: 100, Y: .5, Z: 100}), Position: vector.Vec3[float64]{Y: -2.5}, Material: material})
	require.NoError(t, err)
	pyramid, err = poly.NewPolyhedron([]vector.Vec3[float64]{{X: -1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: 1}, {X: -1, Y: -1, Z: 1}, {Y: 1}}, []poly.Triangle3{{A: 0, B: 2, C: 1}, {A: 0, B: 3, C: 2}, {A: 0, B: 1, C: 4}, {A: 1, B: 2, C: 4}, {A: 2, B: 3, C: 4}, {A: 3, B: 0, C: 4}})
	require.NoError(t, err)
	body, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewPolyhedron3(pyramid, vector.Vec3[float64]{X: .6, Y: .6, Z: .6}), Position: vector.Vec3[float64]{Y: 1}, Rotation: vector.Vec3[float64]{X: .35, Z: .2}, AngularVelocity: vector.Vec3[float64]{X: 2, Y: 1, Z: -1}, Material: material})
	require.NoError(t, err)
	for range 4800 {
		world.Step(1.0 / 240.0)
	}
	assert.Less(t, body.Velocity.SquaredLength()+body.AngularVelocity.SquaredLength(), .05)
}

func TestPolygonFaceContactUsesTwoPointManifold(t *testing.T) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.EnableSleeping = false
	var world *physics.World2[float64] = physics.NewWorld2(config)
	var material physics.Material[float64] = physics.Material[float64]{Density: 1, Restitution: 0, StaticFriction: .8, DynamicFriction: .6}
	var err error
	_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -10, Y: -1}, {X: 10, Y: -1}, {X: 10, Y: 1}, {X: -10, Y: 1}}, 1), Position: vector.Vec2[float64]{Y: 2}, Material: material})
	require.NoError(t, err)
	var body *physics.Body2[float64]
	body, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -1, Y: -1}, {X: 1, Y: -1}, {X: 1, Y: 1}, {X: -1, Y: 1}}, 1), Position: vector.Vec2[float64]{Y: .01}, Material: material})
	require.NoError(t, err)
	world.Step(1.0 / 120.0)
	require.Len(t, world.Contacts, 2)
	assert.InDelta(t, -1, world.Contacts[0].Point.X, 1e-6)
	assert.InDelta(t, 1, world.Contacts[1].Point.X, 1e-6)
	assert.InDelta(t, 0, body.AngularVelocity, 1e-8)
}

func TestSpinningPolygonDoesNotTunnelThroughFloor(t *testing.T) {
	var config physics.WorldConfig[float64] = physics.DefaultWorldConfig[float64]()
	config.GravityY, config.EnableCCD, config.CCDMotionThreshold = 30, true, .25
	var world *physics.World2[float64] = physics.NewWorld2(config)
	var material physics.Material[float64] = physics.Material[float64]{Density: 1, Restitution: 0, StaticFriction: .8, DynamicFriction: .6}
	var err error
	_, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -1000, Y: -1}, {X: 1000, Y: -1}, {X: 1000, Y: 1}, {X: -1000, Y: 1}}, 1), Position: vector.Vec2[float64]{Y: 10}, Material: material})
	require.NoError(t, err)
	var body *physics.Body2[float64]
	body, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewPolygon2([]vector.Vec2[float64]{{X: -1, Y: -1}, {X: 1, Y: -1}, {X: 1, Y: 1}, {X: -1, Y: 1}}, 1), Position: vector.Vec2[float64]{Y: -5}, AngularVelocity: 25, Continuous: true, LinearDamping: .2, AngularDamping: .2, Material: material})
	require.NoError(t, err)
	var sawContact bool
	for range 1200 {
		world.Step(1.0 / 120.0)
		sawContact = sawContact || len(world.Contacts) > 0
	}
	assert.True(t, sawContact)
	assert.Less(t, body.Position.Y, 11.0)
}
