package physics

import (
	"fmt"

	"github.com/z46-dev/gamelib/vector"
	"golang.org/x/exp/constraints"
)

type (
	// AreaConstraint2 preserves the signed area of a triangle of 2D bodies.
	AreaConstraint2[T constraints.Float] struct {
		ID                   ConstraintID
		First, Second, Third *Body2[T]
		RestArea, Compliance T
		lambda               T
	}

	// VolumeConstraint3 preserves the signed volume of a tetrahedron of 3D bodies.
	VolumeConstraint3[T constraints.Float] struct {
		ID                           ConstraintID
		First, Second, Third, Fourth *Body3[T]
		RestVolume, Compliance       T
		lambda                       T
	}

	// ClothConfig controls particle-grid construction in either dimension.
	ClothConfig[T constraints.Float] struct {
		Columns, Rows                                                          int
		Spacing, Radius, Mass, Compliance, BendCompliance, Damping, BreakForce T
		PinTop                                                                 bool
		Material                                                               Material[T]
		Filter                                                                 CollisionFilter
	}

	// Cloth2 owns a 2D particle grid and its structural constraints.
	Cloth2[T constraints.Float] struct {
		Bodies        []*Body2[T]
		Constraints   []*DistanceConstraint2[T]
		Areas         []*AreaConstraint2[T]
		Columns, Rows int
	}
	// Cloth3 owns a 3D particle sheet and its structural constraints.
	Cloth3[T constraints.Float] struct {
		Bodies        []*Body3[T]
		Constraints   []*DistanceConstraint3[T]
		Columns, Rows int
	}
	// TriangleIndices identifies three particles in a 2D soft body.
	TriangleIndices struct{ First, Second, Third int }
	// TetrahedronIndices identifies four particles in a 3D soft body.
	TetrahedronIndices struct{ First, Second, Third, Fourth int }
	// SoftBodyConfig controls particle and preservation constraints.
	SoftBodyConfig[T constraints.Float] struct {
		Radius, Mass, Compliance, Damping, BreakForce T
		Material                                      Material[T]
		Filter                                        CollisionFilter
	}
	// SoftBody2 owns particles, edges, and area-preserving triangles.
	SoftBody2[T constraints.Float] struct {
		Bodies      []*Body2[T]
		Constraints []*DistanceConstraint2[T]
		Areas       []*AreaConstraint2[T]
	}
	// SoftBody3 owns particles, edges, and volume-preserving tetrahedra.
	SoftBody3[T constraints.Float] struct {
		Bodies      []*Body3[T]
		Constraints []*DistanceConstraint3[T]
		Volumes     []*VolumeConstraint3[T]
	}
)

// AddAreaConstraint creates a signed-area-preserving 2D triangle constraint.
func (w *World2[T]) AddAreaConstraint(firstID, secondID, thirdID BodyID, compliance T) (constraint *AreaConstraint2[T], err error) {
	var first, second, third *Body2[T]
	var found bool
	if first, found = w.bodies[firstID]; !found {
		err = fmt.Errorf("physics: area constraint body %d not found", firstID)
		return
	}
	if second, found = w.bodies[secondID]; !found {
		err = fmt.Errorf("physics: area constraint body %d not found", secondID)
		return
	}
	if third, found = w.bodies[thirdID]; !found {
		err = fmt.Errorf("physics: area constraint body %d not found", thirdID)
		return
	}
	if first == second || second == third || first == third {
		err = fmt.Errorf("physics: area constraint requires three distinct bodies")
		return
	}
	w.nextConstraintID++
	constraint = &AreaConstraint2[T]{ID: w.nextConstraintID, First: first, Second: second, Third: third, RestArea: triangleArea2(first.Position, second.Position, third.Position), Compliance: max(compliance, 0)}
	w.areaConstraints = append(w.areaConstraints, constraint)
	return
}

// AddVolumeConstraint creates a signed-volume-preserving 3D tetrahedron constraint.
func (w *World3[T]) AddVolumeConstraint(firstID, secondID, thirdID, fourthID BodyID, compliance T) (constraint *VolumeConstraint3[T], err error) {
	var first, second, third, fourth *Body3[T]
	var found bool
	if first, found = w.bodies[firstID]; !found {
		err = fmt.Errorf("physics: volume constraint body %d not found", firstID)
		return
	}
	if second, found = w.bodies[secondID]; !found {
		err = fmt.Errorf("physics: volume constraint body %d not found", secondID)
		return
	}
	if third, found = w.bodies[thirdID]; !found {
		err = fmt.Errorf("physics: volume constraint body %d not found", thirdID)
		return
	}
	if fourth, found = w.bodies[fourthID]; !found {
		err = fmt.Errorf("physics: volume constraint body %d not found", fourthID)
		return
	}
	w.nextConstraintID++
	constraint = &VolumeConstraint3[T]{ID: w.nextConstraintID, First: first, Second: second, Third: third, Fourth: fourth, RestVolume: tetrahedronVolume3(first.Position, second.Position, third.Position, fourth.Position), Compliance: max(compliance, 0)}
	w.volumeConstraints = append(w.volumeConstraints, constraint)
	return
}

func triangleArea2[T constraints.Float](a, b, c vector.Vec2[T]) (area T) {
	area = ((b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X)) / 2
	return
}

func (c *AreaConstraint2[T]) solve(dt T) {
	var area T = triangleArea2(c.First.Position, c.Second.Position, c.Third.Position)
	var gradients [3]vector.Vec2[T] = [3]vector.Vec2[T]{{X: (c.Second.Position.Y - c.Third.Position.Y) / 2, Y: (c.Third.Position.X - c.Second.Position.X) / 2}, {X: (c.Third.Position.Y - c.First.Position.Y) / 2, Y: (c.First.Position.X - c.Third.Position.X) / 2}, {X: (c.First.Position.Y - c.Second.Position.Y) / 2, Y: (c.Second.Position.X - c.First.Position.X) / 2}}
	var bodies [3]*Body2[T] = [3]*Body2[T]{c.First, c.Second, c.Third}
	var denominator T = c.Compliance / (dt * dt)
	for i := range bodies {
		denominator += bodies[i].inverseMass * gradients[i].SquaredLength()
	}
	if denominator == 0 {
		return
	}
	var alpha T = c.Compliance / (dt * dt)
	var delta T = (-(area - c.RestArea) - alpha*c.lambda) / denominator
	c.lambda += delta
	for i := range bodies {
		bodies[i].Position.X += gradients[i].X * delta * bodies[i].inverseMass
		bodies[i].Position.Y += gradients[i].Y * delta * bodies[i].inverseMass
	}
}

func tetrahedronVolume3[T constraints.Float](a, b, c, d vector.Vec3[T]) (volume T) {
	var ab, ac, ad vector.Vec3[T] = vector.Vec3[T]{X: b.X - a.X, Y: b.Y - a.Y, Z: b.Z - a.Z}, vector.Vec3[T]{X: c.X - a.X, Y: c.Y - a.Y, Z: c.Z - a.Z}, vector.Vec3[T]{X: d.X - a.X, Y: d.Y - a.Y, Z: d.Z - a.Z}
	ac.Cross(&ad)
	volume = ab.Dot(&ac) / 6
	return
}

func crossDifference3[T constraints.Float](a, b, c vector.Vec3[T]) (gradient vector.Vec3[T]) {
	gradient = vector.Vec3[T]{X: a.X - b.X, Y: a.Y - b.Y, Z: a.Z - b.Z}
	var other vector.Vec3[T] = vector.Vec3[T]{X: c.X - b.X, Y: c.Y - b.Y, Z: c.Z - b.Z}
	gradient.Cross(&other)
	gradient.Mul(T(1.0 / 6.0))
	return
}

func (c *VolumeConstraint3[T]) solve(dt T) {
	var gradients [4]vector.Vec3[T]
	gradients[1] = crossDifference3(c.Third.Position, c.First.Position, c.Fourth.Position)
	gradients[2] = crossDifference3(c.Fourth.Position, c.First.Position, c.Second.Position)
	gradients[3] = crossDifference3(c.Second.Position, c.First.Position, c.Third.Position)
	gradients[0] = vector.Vec3[T]{X: -gradients[1].X - gradients[2].X - gradients[3].X, Y: -gradients[1].Y - gradients[2].Y - gradients[3].Y, Z: -gradients[1].Z - gradients[2].Z - gradients[3].Z}
	var bodies [4]*Body3[T] = [4]*Body3[T]{c.First, c.Second, c.Third, c.Fourth}
	var alpha T = c.Compliance / (dt * dt)
	var denominator T = alpha
	for i := range bodies {
		denominator += bodies[i].inverseMass * gradients[i].SquaredLength()
	}
	if denominator == 0 {
		return
	}
	var delta T = (-(tetrahedronVolume3(c.First.Position, c.Second.Position, c.Third.Position, c.Fourth.Position) - c.RestVolume) - alpha*c.lambda) / denominator
	c.lambda += delta
	for i := range bodies {
		bodies[i].Position.X += gradients[i].X * delta * bodies[i].inverseMass
		bodies[i].Position.Y += gradients[i].Y * delta * bodies[i].inverseMass
		bodies[i].Position.Z += gradients[i].Z * delta * bodies[i].inverseMass
	}
}

// AddCloth creates a triangulated 2D particle grid with structural, shear, bend, and area constraints.
func (w *World2[T]) AddCloth(origin vector.Vec2[T], config ClothConfig[T]) (cloth *Cloth2[T], err error) {
	if config.Columns < 2 || config.Rows < 2 || config.Spacing <= 0 || config.Radius <= 0 {
		err = fmt.Errorf("physics: cloth requires a 2x2 grid and positive spacing and radius")
		return
	}
	if config.Material == (Material[T]{}) {
		config.Material = DefaultMaterial[T]()
	}
	if config.Filter == (CollisionFilter{}) {
		config.Filter = DefaultCollisionFilter()
	}
	cloth = &Cloth2[T]{Bodies: make([]*Body2[T], 0, config.Columns*config.Rows), Columns: config.Columns, Rows: config.Rows}
	for row := 0; row < config.Rows; row++ {
		for column := 0; column < config.Columns; column++ {
			var bodyType BodyType = DynamicBody
			if config.PinTop && row == 0 {
				bodyType = StaticBody
			}
			var body *Body2[T]
			body, err = w.AddBody(Body2Config[T]{Type: bodyType, Shape: NewCircle2(config.Radius), Position: vector.Vec2[T]{X: origin.X + T(column)*config.Spacing, Y: origin.Y + T(row)*config.Spacing}, Mass: config.Mass, Material: config.Material, Filter: config.Filter, GravityScale: 1})
			if err != nil {
				return
			}
			cloth.Bodies = append(cloth.Bodies, body)
		}
	}
	var addLink func(int, int, T) error
	addLink = func(first, second int, compliance T) (linkErr error) {
		var link *DistanceConstraint2[T]
		link, linkErr = w.AddDistanceConstraint(DistanceConstraintConfig[T]{First: cloth.Bodies[first].ID, Second: cloth.Bodies[second].ID, Compliance: compliance, Damping: config.Damping, BreakForce: config.BreakForce})
		if linkErr == nil {
			cloth.Constraints = append(cloth.Constraints, link)
		}
		return
	}
	for row := 0; row < config.Rows; row++ {
		for column := 0; column < config.Columns; column++ {
			var index int = row*config.Columns + column
			if column+1 < config.Columns {
				if err = addLink(index, index+1, config.Compliance); err != nil {
					return
				}
			}
			if row+1 < config.Rows {
				if err = addLink(index, index+config.Columns, config.Compliance); err != nil {
					return
				}
			}
			if column+1 < config.Columns && row+1 < config.Rows {
				if err = addLink(index, index+config.Columns+1, config.Compliance); err != nil {
					return
				}
				var area *AreaConstraint2[T]
				area, err = w.AddAreaConstraint(cloth.Bodies[index].ID, cloth.Bodies[index+1].ID, cloth.Bodies[index+config.Columns+1].ID, config.Compliance)
				if err != nil {
					return
				}
				cloth.Areas = append(cloth.Areas, area)
			}
			if column+2 < config.Columns {
				if err = addLink(index, index+2, config.BendCompliance); err != nil {
					return
				}
			}
			if row+2 < config.Rows {
				if err = addLink(index, index+2*config.Columns, config.BendCompliance); err != nil {
					return
				}
			}
		}
	}
	return
}

// AddCloth creates a 3D particle sheet in the XY plane with structural, shear, and bend links.
func (w *World3[T]) AddCloth(origin vector.Vec3[T], config ClothConfig[T]) (cloth *Cloth3[T], err error) {
	if config.Columns < 2 || config.Rows < 2 || config.Spacing <= 0 || config.Radius <= 0 {
		err = fmt.Errorf("physics: cloth requires a 2x2 grid and positive spacing and radius")
		return
	}
	if config.Material == (Material[T]{}) {
		config.Material = DefaultMaterial[T]()
	}
	if config.Filter == (CollisionFilter{}) {
		config.Filter = DefaultCollisionFilter()
	}
	cloth = &Cloth3[T]{Bodies: make([]*Body3[T], 0, config.Columns*config.Rows), Columns: config.Columns, Rows: config.Rows}
	for row := 0; row < config.Rows; row++ {
		for column := 0; column < config.Columns; column++ {
			var bodyType BodyType = DynamicBody
			if config.PinTop && row == 0 {
				bodyType = StaticBody
			}
			var body *Body3[T]
			body, err = w.AddBody(Body3Config[T]{Type: bodyType, Shape: NewSphere3(config.Radius), Position: vector.Vec3[T]{X: origin.X + T(column)*config.Spacing, Y: origin.Y + T(row)*config.Spacing, Z: origin.Z}, Mass: config.Mass, Material: config.Material, Filter: config.Filter, GravityScale: 1})
			if err != nil {
				return
			}
			cloth.Bodies = append(cloth.Bodies, body)
		}
	}
	var addLink func(int, int, T) error
	addLink = func(first, second int, compliance T) (linkErr error) {
		var link *DistanceConstraint3[T]
		link, linkErr = w.AddDistanceConstraint(DistanceConstraintConfig[T]{First: cloth.Bodies[first].ID, Second: cloth.Bodies[second].ID, Compliance: compliance, Damping: config.Damping, BreakForce: config.BreakForce})
		if linkErr == nil {
			cloth.Constraints = append(cloth.Constraints, link)
		}
		return
	}
	for row := 0; row < config.Rows; row++ {
		for column := 0; column < config.Columns; column++ {
			var index int = row*config.Columns + column
			if column+1 < config.Columns {
				if err = addLink(index, index+1, config.Compliance); err != nil {
					return
				}
			}
			if row+1 < config.Rows {
				if err = addLink(index, index+config.Columns, config.Compliance); err != nil {
					return
				}
			}
			if column+1 < config.Columns && row+1 < config.Rows {
				if err = addLink(index, index+config.Columns+1, config.Compliance); err != nil {
					return
				}
				if err = addLink(index+1, index+config.Columns, config.Compliance); err != nil {
					return
				}
			}
			if column+2 < config.Columns {
				if err = addLink(index, index+2, config.BendCompliance); err != nil {
					return
				}
			}
			if row+2 < config.Rows {
				if err = addLink(index, index+2*config.Columns, config.BendCompliance); err != nil {
					return
				}
			}
		}
	}
	return
}

// AddSoftBody creates a 2D deformable from particles and area-preserving triangles.
func (w *World2[T]) AddSoftBody(points []vector.Vec2[T], triangles []TriangleIndices, config SoftBodyConfig[T]) (softBody *SoftBody2[T], err error) {
	if len(points) < 3 || len(triangles) == 0 || config.Radius <= 0 {
		err = fmt.Errorf("physics: SoftBody2 requires particles, triangles, and a positive radius")
		return
	}
	if config.Material == (Material[T]{}) {
		config.Material = DefaultMaterial[T]()
	}
	if config.Filter == (CollisionFilter{}) {
		config.Filter = DefaultCollisionFilter()
	}
	softBody = &SoftBody2[T]{Bodies: make([]*Body2[T], 0, len(points))}
	for _, point := range points {
		var body *Body2[T]
		body, err = w.AddBody(Body2Config[T]{Type: DynamicBody, Shape: NewCircle2(config.Radius), Position: point, Mass: config.Mass, Material: config.Material, Filter: config.Filter, GravityScale: 1})
		if err != nil {
			return
		}
		softBody.Bodies = append(softBody.Bodies, body)
	}
	var edges map[[2]int]struct{} = make(map[[2]int]struct{})
	for _, triangle := range triangles {
		var values [3]int = [3]int{triangle.First, triangle.Second, triangle.Third}
		for _, index := range values {
			if index < 0 || index >= len(points) {
				err = fmt.Errorf("physics: SoftBody2 triangle index out of range")
				return
			}
		}
		var area *AreaConstraint2[T]
		area, err = w.AddAreaConstraint(softBody.Bodies[values[0]].ID, softBody.Bodies[values[1]].ID, softBody.Bodies[values[2]].ID, config.Compliance)
		if err != nil {
			return
		}
		softBody.Areas = append(softBody.Areas, area)
		for edge := 0; edge < 3; edge++ {
			var first, second int = values[edge], values[(edge+1)%3]
			if first > second {
				first, second = second, first
			}
			edges[[2]int{first, second}] = struct{}{}
		}
	}
	for edge := range edges {
		var link *DistanceConstraint2[T]
		link, err = w.AddDistanceConstraint(DistanceConstraintConfig[T]{First: softBody.Bodies[edge[0]].ID, Second: softBody.Bodies[edge[1]].ID, Compliance: config.Compliance, Damping: config.Damping, BreakForce: config.BreakForce})
		if err != nil {
			return
		}
		softBody.Constraints = append(softBody.Constraints, link)
	}
	return
}

// AddSoftBody creates a 3D deformable from particles and volume-preserving tetrahedra.
func (w *World3[T]) AddSoftBody(points []vector.Vec3[T], tetrahedra []TetrahedronIndices, config SoftBodyConfig[T]) (softBody *SoftBody3[T], err error) {
	if len(points) < 4 || len(tetrahedra) == 0 || config.Radius <= 0 {
		err = fmt.Errorf("physics: SoftBody3 requires particles, tetrahedra, and a positive radius")
		return
	}
	if config.Material == (Material[T]{}) {
		config.Material = DefaultMaterial[T]()
	}
	if config.Filter == (CollisionFilter{}) {
		config.Filter = DefaultCollisionFilter()
	}
	softBody = &SoftBody3[T]{Bodies: make([]*Body3[T], 0, len(points))}
	for _, point := range points {
		var body *Body3[T]
		body, err = w.AddBody(Body3Config[T]{Type: DynamicBody, Shape: NewSphere3(config.Radius), Position: point, Mass: config.Mass, Material: config.Material, Filter: config.Filter, GravityScale: 1})
		if err != nil {
			return
		}
		softBody.Bodies = append(softBody.Bodies, body)
	}
	var edges map[[2]int]struct{} = make(map[[2]int]struct{})
	for _, tetrahedron := range tetrahedra {
		var values [4]int = [4]int{tetrahedron.First, tetrahedron.Second, tetrahedron.Third, tetrahedron.Fourth}
		for _, index := range values {
			if index < 0 || index >= len(points) {
				err = fmt.Errorf("physics: SoftBody3 tetrahedron index out of range")
				return
			}
		}
		var volume *VolumeConstraint3[T]
		volume, err = w.AddVolumeConstraint(softBody.Bodies[values[0]].ID, softBody.Bodies[values[1]].ID, softBody.Bodies[values[2]].ID, softBody.Bodies[values[3]].ID, config.Compliance)
		if err != nil {
			return
		}
		softBody.Volumes = append(softBody.Volumes, volume)
		for first := 0; first < 4; first++ {
			for second := first + 1; second < 4; second++ {
				var a, b int = values[first], values[second]
				if a > b {
					a, b = b, a
				}
				edges[[2]int{a, b}] = struct{}{}
			}
		}
	}
	for edge := range edges {
		var link *DistanceConstraint3[T]
		link, err = w.AddDistanceConstraint(DistanceConstraintConfig[T]{First: softBody.Bodies[edge[0]].ID, Second: softBody.Bodies[edge[1]].ID, Compliance: config.Compliance, Damping: config.Damping, BreakForce: config.BreakForce})
		if err != nil {
			return
		}
		softBody.Constraints = append(softBody.Constraints, link)
	}
	return
}

// AreaConstraints returns a detached 2D area-constraint list.
func (w *World2[T]) AreaConstraints() (result []*AreaConstraint2[T]) {
	result = append(result, w.areaConstraints...)
	return
}

// VolumeConstraints returns a detached 3D volume-constraint list.
func (w *World3[T]) VolumeConstraints() (result []*VolumeConstraint3[T]) {
	result = append(result, w.volumeConstraints...)
	return
}
