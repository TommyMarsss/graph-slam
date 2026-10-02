package main

import (
	"math"
	"math/rand"
)

// Scenario 打包一个实验场景：图、初始估计、真值（用于对比）与注入信息。
type Scenario struct {
	Graph   *Graph
	Initial []Pose
	Truth   []Pose
	BadLoop string // 被注入的错误回环 ID（无则为空）
}

// 统一信息矩阵（对角）：平移 σ≈0.01m，角度 σ≈0.005rad。
var odomInfo = [3]float64{10000, 10000, 40000}
var loopInfo = [3]float64{10000, 10000, 40000}

// rectTruth 生成沿矩形周长均匀采样的真值轨迹，每边 sideSteps 个间隔。
// 节点沿 底边→右边→顶边→左边 行进，末节点回到起点附近。
func rectTruth(width, height float64, sideSteps int) []Pose {
	var pts [][2]float64
	add := func(x0, y0, x1, y1 float64, n int) {
		for k := 0; k < n; k++ {
			t := float64(k) / float64(n)
			pts = append(pts, [2]float64{
				x0 + (x1-x0)*t,
				y0 + (y1-y0)*t,
			})
		}
	}
	add(0, 0, width, 0, sideSteps)           // 底边，向东
	add(width, 0, width, height, sideSteps)  // 右边，向北
	add(width, height, 0, height, sideSteps) // 顶边，向西
	add(0, height, 0, 0, sideSteps)          // 左边，向南

	nodes := make([]Pose, len(pts))
	for i, p := range pts {
		var th float64
		switch {
		case i < 1*sideSteps:
			th = 0
		case i < 2*sideSteps:
			th = math.Pi / 2
		case i < 3*sideSteps:
			th = math.Pi
		default:
			th = -math.Pi / 2
		}
		// 角点处取“即将进入的边”的朝向，使逐边相对测量角度为小量
		if i%sideSteps == 0 {
			th = []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2}[i/sideSteps]
		}
		nodes[i] = Pose{X: p[0], Y: p[1], Theta: th}
	}
	return nodes
}

// noisyRelative 由真值相邻位姿构造相对测量并加入小噪声（固定种子可复现）。
func noisyRelative(a, b Pose, rng *rand.Rand, angSigma, posSigma float64) Pose {
	z := Compose(Invert(a), b)
	z.X += rng.NormFloat64() * posSigma
	z.Y += rng.NormFloat64() * posSigma
	z.Theta = wrapAngle(z.Theta + rng.NormFloat64()*angSigma)
	return z
}

// deadReckoning 从节点 0 出发沿里程计链递推，得到带漂移的初始估计。
func deadReckoning(g *Graph) []Pose {
	nodes := make([]Pose, len(g.Nodes))
	nodes[0] = g.Nodes[0]
	byI := make(map[int]Constraint)
	for _, c := range g.Constraints {
		if c.Kind == KindOdom && c.J == c.I+1 {
			byI[c.I] = c
		}
	}
	for i := 0; i+1 < len(nodes); i++ {
		c := byI[i]
		nodes[i+1] = Compose(nodes[i], c.Z)
	}
	return nodes
}

// buildCorrectScenario 正确回环场景：矩形轨迹 + 里程计噪声 + 真实首尾回环。
func buildCorrectScenario() Scenario {
	truth := rectTruth(12, 12, 12)
	n := len(truth)
	rng := rand.New(rand.NewSource(42))

	g := &Graph{Name: "correct", Nodes: make([]Pose, n)}
	g.Nodes[0] = truth[0]
	for i := 0; i+1 < n; i++ {
		z := noisyRelative(truth[i], truth[i+1], rng, 0.005, 0.01)
		g.Constraints = append(g.Constraints, Constraint{
			ID: "odom", Kind: KindOdom, I: i, J: i + 1, Z: z, Info: odomInfo,
		})
	}
	// 真实回环：末节点回到起点
	zl := noisyRelative(truth[n-1], truth[0], rng, 0.005, 0.01)
	g.Constraints = append(g.Constraints, Constraint{
		ID: "loop-good", Kind: KindLoop, I: n - 1, J: 0, Z: zl, Info: loopInfo,
	})

	return Scenario{Graph: g, Initial: deadReckoning(g), Truth: truth}
}

// buildBadLoopScenario 注入错误回环：把两条对边上相距很远的节点误判为同一地点。
func buildBadLoopScenario() Scenario {
	s := buildCorrectScenario()
	s.Graph.Name = "bad_loop"

	// 底边上的节点 3 与顶边上的节点 2*12+9=33 实际相距约 12m，
	// 却给出“近似重合、朝向略偏”的回环测量——典型的错误 place recognition。
	bad := Constraint{
		ID:   "loop-BAD",
		Kind: KindLoop,
		I:    3,
		J:    2*12 + 9,
		Z:    Pose{X: 0.3, Y: -0.2, Theta: 0.15},
		Info: loopInfo,
	}
	s.Graph.Constraints = append(s.Graph.Constraints, bad)
	s.BadLoop = bad.ID
	return s
}

// buildLocalMinScenario 大初始误差场景：约束全部正确（无噪声），
// 但初始估计把后半段轨迹绕矩形中心翻转 180°。该初值位于错误的环绕数
// （winding number）分支：LM 会收敛到一个平移残差全为 0、而总转角
// 2π 被均摊到每条边的驻点，轨迹自交——数值收敛、几何荒谬。
func buildLocalMinScenario() Scenario {
	truth := rectTruth(12, 12, 12)
	n := len(truth)

	g := &Graph{Name: "local_min", Nodes: make([]Pose, n)}
	g.Nodes[0] = truth[0]
	for i := 0; i+1 < n; i++ {
		z := Compose(Invert(truth[i]), truth[i+1]) // 精确测量
		g.Constraints = append(g.Constraints, Constraint{
			ID: "odom", Kind: KindOdom, I: i, J: i + 1, Z: z, Info: odomInfo,
		})
	}
	zl := Compose(Invert(truth[n-1]), truth[0])
	g.Constraints = append(g.Constraints, Constraint{
		ID: "loop-good", Kind: KindLoop, I: n - 1, J: 0, Z: zl, Info: loopInfo,
	})

	// 折叠初值：从节点 12 起绕中心旋转 180°
	initial := make([]Pose, n)
	copy(initial, truth)
	cx, cy := 6.0, 6.0
	fold := math.Pi
	c, s := math.Cos(fold), math.Sin(fold)
	for i := 12; i < n; i++ {
		dx, dy := truth[i].X-cx, truth[i].Y-cy
		initial[i] = Pose{
			X:     cx + c*dx - s*dy,
			Y:     cy + s*dx + c*dy,
			Theta: wrapAngle(truth[i].Theta + fold),
		}
	}
	return Scenario{Graph: g, Initial: initial, Truth: truth}
}

func allScenarios() []Scenario {
	return []Scenario{
		buildCorrectScenario(),
		buildBadLoopScenario(),
		buildLocalMinScenario(),
	}
}
