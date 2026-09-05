package physics_test

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/z46-dev/gamelib/hshg"
	"github.com/z46-dev/gamelib/physics"
	"github.com/z46-dev/gamelib/vector"
)

func TestWorld2SpatialQueries(t *testing.T) {
	var (
		world        *physics.World2[float64] = physics.NewWorld2(physics.DefaultWorldConfig[float64]())
		near, corner *physics.Body2[float64]
		far          *physics.Body2[float64]
		results      []*physics.Body2[float64]
		err          error
	)
	near, err = world.AddBody(physics.Body2Config[float64]{Type: physics.DynamicBody, Shape: physics.NewCircle2(1.0), Position: vector.Vec2[float64]{X: 2}})
	require.NoError(t, err)
	corner, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewCircle2(.1), Position: vector.Vec2[float64]{X: 4, Y: 4}})
	require.NoError(t, err)
	far, err = world.AddBody(physics.Body2Config[float64]{Type: physics.StaticBody, Shape: physics.NewCircle2(1.0), Position: vector.Vec2[float64]{X: 20}})
	require.NoError(t, err)

	results = world.BodiesInAABB(&hshg.AABB2[float64]{X1: -1, Y1: -1, X2: 3, Y2: 1})
	assert.Equal(t, []*physics.Body2[float64]{near}, results)
	results = world.BodiesInRadiusInto(results[:0], vector.Vec2[float64]{}, 5)
	assert.Equal(t, []*physics.Body2[float64]{near}, results, "the exact radius filter should reject square-corner candidates")

	near.SetTransform(vector.Vec2[float64]{X: 20}, 0)
	assert.Empty(t, world.BodiesInRadius(vector.Vec2[float64]{}, 5))
	assert.True(t, world.RemoveBody(far.ID))
	assert.Equal(t, []*physics.Body2[float64]{near}, world.BodiesInRadius(vector.Vec2[float64]{X: 20}, 2))
	corner.Disabled = true
	assert.Empty(t, world.BodiesInAABB(&hshg.AABB2[float64]{X1: 3, Y1: 3, X2: 5, Y2: 5}))
}

func TestWorld3SpatialQueries(t *testing.T) {
	var (
		world        *physics.World3[float64] = physics.NewWorld3(physics.DefaultWorldConfig[float64]())
		near, corner *physics.Body3[float64]
		results      []*physics.Body3[float64]
		err          error
	)
	near, err = world.AddBody(physics.Body3Config[float64]{Type: physics.DynamicBody, Shape: physics.NewSphere3(1.0), Position: vector.Vec3[float64]{X: 2}})
	require.NoError(t, err)
	corner, err = world.AddBody(physics.Body3Config[float64]{Type: physics.StaticBody, Shape: physics.NewSphere3(.1), Position: vector.Vec3[float64]{X: 4, Y: 4, Z: 4}})
	require.NoError(t, err)

	results = world.BodiesInAABBInto(nil, &hshg.AABB3[float64]{X1: -1, Y1: -1, Z1: -1, X2: 3, Y2: 1, Z2: 1})
	assert.Equal(t, []*physics.Body3[float64]{near}, results)
	assert.Equal(t, []*physics.Body3[float64]{near}, world.BodiesInRadius(vector.Vec3[float64]{}, 5), "the exact radius filter should reject cube-corner candidates")

	near.SetEulerTransform(vector.Vec3[float64]{X: 20}, vector.Vec3[float64]{})
	assert.Empty(t, world.BodiesInRadius(vector.Vec3[float64]{}, 5))
	assert.Equal(t, []*physics.Body3[float64]{near}, world.BodiesInRadiusInto(results[:0], vector.Vec3[float64]{X: 20}, 2))
	corner.Disabled = true
	assert.Empty(t, world.BodiesInAABB(&hshg.AABB3[float64]{X1: 3, Y1: 3, Z1: 3, X2: 5, Y2: 5, Z2: 5}))
}
