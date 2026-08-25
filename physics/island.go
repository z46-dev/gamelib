package physics

import (
	"sync"
	"sync/atomic"

	"golang.org/x/exp/constraints"
)

type (
	island2[T constraints.Float] struct {
		bodies    []*Body2[T]
		contacts  []int
		distances []*DistanceConstraint2[T]
		angular   []*AngularConstraint2[T]
		areas     []*AreaConstraint2[T]
	}
	island3[T constraints.Float] struct {
		bodies    []*Body3[T]
		contacts  []int
		distances []*DistanceConstraint3[T]
		angular   []*AngularConstraint3[T]
		volumes   []*VolumeConstraint3[T]
	}
)

func findIslandRoot(parent []int, index int) (root int) {
	root = index
	for parent[root] != root {
		root = parent[root]
	}
	for parent[index] != index {
		var next int = parent[index]
		parent[index] = root
		index = next
	}
	return
}

func joinIsland(parent, rank []int, first, second int) {
	first, second = findIslandRoot(parent, first), findIslandRoot(parent, second)
	if first == second {
		return
	}
	if rank[first] < rank[second] {
		first, second = second, first
	}
	parent[second] = first
	if rank[first] == rank[second] {
		rank[first]++
	}
}

func (w *World2[T]) buildIslands() (islands []island2[T]) {
	var count int = len(w.bodyOrder)
	if cap(w.islandParent) < count {
		w.islandParent = make([]int, count)
		w.islandRank = make([]int, count)
		w.islandLookup = make([]int, count)
	} else {
		w.islandParent = w.islandParent[:count]
		w.islandRank = w.islandRank[:count]
		w.islandLookup = w.islandLookup[:count]
	}
	for i := range w.islandParent {
		w.islandParent[i] = i
		w.islandRank[i] = 0
		w.islandLookup[i] = -1
	}
	var parent, rank []int = w.islandParent, w.islandRank
	for i := range w.Contacts {
		joinIsland(parent, rank, w.Contacts[i].First.islandIndex, w.Contacts[i].Second.islandIndex)
	}
	for _, c := range w.distanceConstraints {
		if !c.Broken {
			joinIsland(parent, rank, c.First.islandIndex, c.Second.islandIndex)
		}
	}
	for _, c := range w.angularConstraints {
		if !c.Broken {
			joinIsland(parent, rank, c.First.islandIndex, c.Second.islandIndex)
		}
	}
	for _, c := range w.areaConstraints {
		joinIsland(parent, rank, c.First.islandIndex, c.Second.islandIndex)
		joinIsland(parent, rank, c.First.islandIndex, c.Third.islandIndex)
	}
	for i := range w.islands {
		w.islands[i].bodies = w.islands[i].bodies[:0]
		w.islands[i].contacts = w.islands[i].contacts[:0]
		w.islands[i].distances = w.islands[i].distances[:0]
		w.islands[i].angular = w.islands[i].angular[:0]
		w.islands[i].areas = w.islands[i].areas[:0]
	}
	var islandCount int
	for i, body := range w.bodyOrder {
		if body.Type != DynamicBody {
			continue
		}
		var root int = findIslandRoot(parent, i)
		var islandIndex int = w.islandLookup[root]
		if islandIndex < 0 {
			islandIndex = islandCount
			islandCount++
			w.islandLookup[root] = islandIndex
			if islandIndex == len(w.islands) {
				w.islands = append(w.islands, island2[T]{})
			}
		}
		w.islands[islandIndex].bodies = append(w.islands[islandIndex].bodies, body)
	}
	for i := range w.Contacts {
		var first *Body2[T] = w.Contacts[i].First
		if first.Type != DynamicBody {
			first = w.Contacts[i].Second
		}
		var islandIndex int = w.islandLookup[findIslandRoot(parent, first.islandIndex)]
		if islandIndex >= 0 {
			w.islands[islandIndex].contacts = append(w.islands[islandIndex].contacts, i)
		}
	}
	for _, constraint := range w.distanceConstraints {
		if !constraint.Broken {
			var first *Body2[T] = constraint.First
			if first.Type != DynamicBody {
				first = constraint.Second
			}
			if first.Type == DynamicBody {
				var index int = w.islandLookup[findIslandRoot(parent, first.islandIndex)]
				w.islands[index].distances = append(w.islands[index].distances, constraint)
			}
		}
	}
	for _, constraint := range w.angularConstraints {
		if !constraint.Broken {
			var first *Body2[T] = constraint.First
			if first.Type != DynamicBody {
				first = constraint.Second
			}
			if first.Type == DynamicBody {
				var index int = w.islandLookup[findIslandRoot(parent, first.islandIndex)]
				w.islands[index].angular = append(w.islands[index].angular, constraint)
			}
		}
	}
	for _, constraint := range w.areaConstraints {
		var first *Body2[T] = constraint.First
		if first.Type != DynamicBody {
			first = constraint.Second
		}
		if first.Type != DynamicBody {
			first = constraint.Third
		}
		if first.Type == DynamicBody {
			var index int = w.islandLookup[findIslandRoot(parent, first.islandIndex)]
			w.islands[index].areas = append(w.islands[index].areas, constraint)
		}
	}
	w.islands = w.islands[:islandCount]
	islands = w.islands
	for i := range islands {
		var active bool
		for _, body := range islands[i].bodies {
			if !body.Sleeping {
				active = true
				break
			}
		}
		if active {
			for _, body := range islands[i].bodies {
				if body.Sleeping {
					body.Wake()
				}
			}
		}
	}
	return
}

func (w *World3[T]) buildIslands() (islands []island3[T]) {
	var count int = len(w.bodyOrder)
	if cap(w.islandParent) < count {
		w.islandParent = make([]int, count)
		w.islandRank = make([]int, count)
		w.islandLookup = make([]int, count)
	} else {
		w.islandParent = w.islandParent[:count]
		w.islandRank = w.islandRank[:count]
		w.islandLookup = w.islandLookup[:count]
	}
	for i := range w.islandParent {
		w.islandParent[i] = i
		w.islandRank[i] = 0
		w.islandLookup[i] = -1
	}
	var parent, rank []int = w.islandParent, w.islandRank
	for i := range w.Contacts {
		joinIsland(parent, rank, w.Contacts[i].First.islandIndex, w.Contacts[i].Second.islandIndex)
	}
	for _, c := range w.distanceConstraints {
		if !c.Broken {
			joinIsland(parent, rank, c.First.islandIndex, c.Second.islandIndex)
		}
	}
	for _, c := range w.angularConstraints {
		if !c.Broken {
			joinIsland(parent, rank, c.First.islandIndex, c.Second.islandIndex)
		}
	}
	for _, c := range w.volumeConstraints {
		joinIsland(parent, rank, c.First.islandIndex, c.Second.islandIndex)
		joinIsland(parent, rank, c.First.islandIndex, c.Third.islandIndex)
		joinIsland(parent, rank, c.First.islandIndex, c.Fourth.islandIndex)
	}
	for i := range w.islands {
		w.islands[i].bodies = w.islands[i].bodies[:0]
		w.islands[i].contacts = w.islands[i].contacts[:0]
		w.islands[i].distances = w.islands[i].distances[:0]
		w.islands[i].angular = w.islands[i].angular[:0]
		w.islands[i].volumes = w.islands[i].volumes[:0]
	}
	var islandCount int
	for i, body := range w.bodyOrder {
		if body.Type != DynamicBody {
			continue
		}
		var root int = findIslandRoot(parent, i)
		var islandIndex int = w.islandLookup[root]
		if islandIndex < 0 {
			islandIndex = islandCount
			islandCount++
			w.islandLookup[root] = islandIndex
			if islandIndex == len(w.islands) {
				w.islands = append(w.islands, island3[T]{})
			}
		}
		w.islands[islandIndex].bodies = append(w.islands[islandIndex].bodies, body)
	}
	for i := range w.Contacts {
		var first *Body3[T] = w.Contacts[i].First
		if first.Type != DynamicBody {
			first = w.Contacts[i].Second
		}
		var islandIndex int = w.islandLookup[findIslandRoot(parent, first.islandIndex)]
		if islandIndex >= 0 {
			w.islands[islandIndex].contacts = append(w.islands[islandIndex].contacts, i)
		}
	}
	for _, constraint := range w.distanceConstraints {
		if !constraint.Broken {
			var first *Body3[T] = constraint.First
			if first.Type != DynamicBody {
				first = constraint.Second
			}
			if first.Type == DynamicBody {
				var index int = w.islandLookup[findIslandRoot(parent, first.islandIndex)]
				w.islands[index].distances = append(w.islands[index].distances, constraint)
			}
		}
	}
	for _, constraint := range w.angularConstraints {
		if !constraint.Broken {
			var first *Body3[T] = constraint.First
			if first.Type != DynamicBody {
				first = constraint.Second
			}
			if first.Type == DynamicBody {
				var index int = w.islandLookup[findIslandRoot(parent, first.islandIndex)]
				w.islands[index].angular = append(w.islands[index].angular, constraint)
			}
		}
	}
	for _, constraint := range w.volumeConstraints {
		var bodies [4]*Body3[T] = [4]*Body3[T]{constraint.First, constraint.Second, constraint.Third, constraint.Fourth}
		var first *Body3[T]
		for _, body := range bodies {
			if body.Type == DynamicBody {
				first = body
				break
			}
		}
		if first != nil {
			var index int = w.islandLookup[findIslandRoot(parent, first.islandIndex)]
			w.islands[index].volumes = append(w.islands[index].volumes, constraint)
		}
	}
	w.islands = w.islands[:islandCount]
	islands = w.islands
	for i := range islands {
		var active bool
		for _, body := range islands[i].bodies {
			if !body.Sleeping {
				active = true
				break
			}
		}
		if active {
			for _, body := range islands[i].bodies {
				if body.Sleeping {
					body.Wake()
				}
			}
		}
	}
	return
}

func runIslandJobs(count, workers int, job func(int)) {
	if workers <= 1 || count < 2 {
		for i := 0; i < count; i++ {
			job(i)
		}
		return
	}
	workers = min(workers, count)
	var next atomic.Int64
	var group sync.WaitGroup
	group.Add(workers)
	for range workers {
		go func() {
			defer group.Done()
			for {
				var index int = int(next.Add(1) - 1)
				if index >= count {
					return
				}
				job(index)
			}
		}()
	}
	group.Wait()
}

func (w *World2[T]) solveIslands(islands []island2[T], dt T) {
	if len(islands) == 0 {
		return
	}
	var workers int = w.Config.ParallelWorkers
	if len(w.bodyOrder) < w.Config.MinimumParallelIslandBodies {
		workers = 1
	}
	if workers <= 1 {
		for index := range islands {
			w.solveIsland(&islands[index], dt)
		}
		return
	}
	runIslandJobs(len(islands), workers, func(index int) { w.solveIsland(&islands[index], dt) })
}

func (w *World2[T]) solveIsland(island *island2[T], dt T) {
	for range w.Config.VelocityIterations {
		for _, contact := range island.contacts {
			resolveVelocity2(&w.Contacts[contact])
		}
	}
	for range w.Config.PositionIterations {
		for _, contact := range island.contacts {
			resolvePosition2(&w.Contacts[contact], w.Config)
		}
		for _, constraint := range island.distances {
			constraint.solve(dt)
		}
		for _, constraint := range island.angular {
			constraint.solve(dt)
		}
		for _, constraint := range island.areas {
			constraint.solve(dt)
		}
	}
	for _, constraint := range island.distances {
		constraint.damp()
	}
	for _, constraint := range island.angular {
		constraint.damp()
	}
}

func (w *World3[T]) solveIslands(islands []island3[T], dt T) {
	if len(islands) == 0 {
		return
	}
	var workers int = w.Config.ParallelWorkers
	if len(w.bodyOrder) < w.Config.MinimumParallelIslandBodies {
		workers = 1
	}
	if workers <= 1 {
		for index := range islands {
			w.solveIsland(&islands[index], dt)
		}
		return
	}
	runIslandJobs(len(islands), workers, func(index int) { w.solveIsland(&islands[index], dt) })
}

func (w *World3[T]) solveIsland(island *island3[T], dt T) {
	for range w.Config.VelocityIterations {
		for _, contact := range island.contacts {
			resolveVelocity3(&w.Contacts[contact])
		}
	}
	for range w.Config.PositionIterations {
		for _, contact := range island.contacts {
			resolvePosition3(&w.Contacts[contact], w.Config)
		}
		for _, constraint := range island.distances {
			constraint.solve(dt)
		}
		for _, constraint := range island.angular {
			constraint.solve(dt)
		}
		for _, constraint := range island.volumes {
			constraint.solve(dt)
		}
	}
	for _, constraint := range island.distances {
		constraint.damp()
	}
	for _, constraint := range island.angular {
		constraint.damp()
	}
}

func (w *World2[T]) updateIslandSleeping(islands []island2[T], dt T) {
	for i := range islands {
		var still bool = true
		for _, body := range islands[i].bodies {
			if body.Velocity.SquaredLength() > w.Config.SleepLinearThreshold*w.Config.SleepLinearThreshold || body.AngularVelocity > w.Config.SleepAngularThreshold || body.AngularVelocity < -w.Config.SleepAngularThreshold {
				still = false
				break
			}
		}
		for _, body := range islands[i].bodies {
			if !w.Config.EnableSleeping {
				body.sleepTime = 0
				continue
			}
			if !still {
				body.sleepTime = 0
				body.Wake()
			} else if body.sleepTime += dt; body.sleepTime >= w.Config.SleepTime {
				body.Sleep()
			}
		}
	}
}
func (w *World3[T]) updateIslandSleeping(islands []island3[T], dt T) {
	for i := range islands {
		var still bool = true
		for _, body := range islands[i].bodies {
			if body.Velocity.SquaredLength() > w.Config.SleepLinearThreshold*w.Config.SleepLinearThreshold || body.AngularVelocity.SquaredLength() > w.Config.SleepAngularThreshold*w.Config.SleepAngularThreshold {
				still = false
				break
			}
		}
		for _, body := range islands[i].bodies {
			if !w.Config.EnableSleeping {
				body.sleepTime = 0
				continue
			}
			if !still {
				body.sleepTime = 0
				body.Wake()
			} else if body.sleepTime += dt; body.sleepTime >= w.Config.SleepTime {
				body.Sleep()
			}
		}
	}
}
