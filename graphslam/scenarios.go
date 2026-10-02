package graphslam

import (
	"math"
	"math/rand"
)

// Scenario 打包一次演示/测试所需的全部数据：真值、图、初始估计与说明。
type Scenario struct {
	Name        string
	Description string
	Truth       []Pose
	Graph       *Graph
	Initial     []Pose
	// KnownRevisits 记录真值中由回环边连接、本就靠近的节点对，
	// 供几何检测豁免，避免把合法回环连接的近邻误报为异常折叠。
	KnownRevisits map[[2]int]bool
	// InjectedBadLoops 为人工注入的错误回环边索引（测试断言用）。
	InjectedBadLoops []int
}

// gaussian 返回标准正态采样。
func gaussian(rng *rand.Rand) float64 { return rng.NormFloat64() }

// LawnmowerScenario 构造“割草机”式往返扫描轨迹：
// 多条平行直行 + 端部 U 形转弯；相邻行之间布置真实回环（航向差 π），
// 节点之间互不重合，因此干净解不应触发任何自穿越/重合几何告警。
//
// rows 为行数，rowLen 为每行直线长度（步长 1），spacing 为行距（=2×转弯半径）。
func LawnmowerScenario(rows, rowLen int, spacing float64, seed int64) *Scenario {
	rng := rand.New(rand.NewSource(seed))
	var truth []Pose
	add := func(x, y, th float64) { truth = append(truth, Pose{x, y, WrapAngle(th)}) }

	radius := spacing / 2
	arcSteps := 6
	for r := 0; r < rows; r++ {
		y := float64(r) * spacing
		if r%2 == 0 { // 偶数行：自西向东
			for k := 0; k <= rowLen; k++ {
				add(float64(k), y, 0)
			}
		} else { // 奇数行：自东向西
			for k := 0; k <= rowLen; k++ {
				add(float64(rowLen-k), y, math.Pi)
			}
		}
		if r == rows-1 {
			break
		}
		// 端部 U 形左转弯（半圆，arcSteps 段），接上下一行首节点。
		var cx, cy, a0 float64
		if r%2 == 0 { // 东端转弯：圆心 (rowLen, y+radius)，从南端经东侧到北端
			cx, cy, a0 = float64(rowLen), y+radius, -math.Pi/2
		} else { // 西端转弯：圆心 (0, y+radius)，从南端经西侧到北端
			cx, cy, a0 = 0, y+radius, -math.Pi/2
		}
		for s := 1; s <= arcSteps; s++ {
			var ang float64
			if r%2 == 0 {
				ang = a0 + float64(s)/float64(arcSteps)*math.Pi // -π/2 → +π/2（东半圆）
			} else {
				ang = a0 - float64(s)/float64(arcSteps)*math.Pi // -π/2 → -3π/2（西半圆）
			}
			x := cx + radius*math.Cos(ang)
			yy := cy + radius*math.Sin(ang)
			// 切线航向：偶数行经东侧（ang 递增）th=ang+π/2；
			// 奇数行经西侧（ang 递减）th=ang-π/2。
			var th float64
			if r%2 == 0 {
				th = ang + math.Pi/2
			} else {
				th = ang - math.Pi/2
			}
			add(x, yy, th)
		}
	}
	// 转弯首尾点与下一行首节点在数值上重合，去掉重复点。
	truth = dedupePoses(truth, 1e-9)

	n := len(truth)
	const sigXY = 0.015
	const sigTh = 0.008
	info := DiagInfo(sigXY, sigTh)
	g := NewGraph(n)

	// 里程计边：真值相对位姿 + 高斯噪声 + 微小系统性陀螺偏置（制造可见漂移）。
	const gyroBias = 0.0015
	for i := 0; i+1 < n; i++ {
		m := Relative(truth[i], truth[i+1])
		m.Dx += gaussian(rng) * sigXY
		m.Dy += gaussian(rng) * sigXY
		m.Dtheta = WrapAngle(m.Dtheta + gaussian(rng)*sigTh + gyroBias)
		g.AddEdge(i, i+1, Odom, m, info)
	}

	// 真实回环：相邻平行行上 x 对齐的节点对（每行取 3 处）。
	loopInfo := DiagInfo(0.03, 0.02)
	revisits := map[[2]int]bool{}
	xTargets := []int{2, 4, 6}
	nodeAt := func(r, x int) int {
		// 每行含 rowLen+1 个直线点 + arcSteps-1 个唯一弧点 = rowLen+arcSteps。
		return r*(rowLen+arcSteps) + map[bool]int{true: x, false: rowLen - x}[r%2 == 0]
	}
	for r := 0; r+1 < rows; r++ {
		for _, x := range xTargets {
			if x > rowLen {
				continue
			}
			i, j := nodeAt(r, x), nodeAt(r+1, x)
			if i >= n || j >= n {
				continue
			}
			m := Relative(truth[i], truth[j])
			g.AddEdge(i, j, Loop, m, loopInfo)
			revisits[key2(i, j)] = true
		}
	}

	return &Scenario{
		Name:          "割草机往返扫描（正确回环）",
		Description:   "机器人在平行行之间往返扫描，带噪且含陀螺偏置的里程计逐步累积漂移；相邻行之间的回环均为真实邻近约束。",
		Truth:         truth,
		Graph:         g,
		Initial:       chainInitialize(g, n),
		KnownRevisits: revisits,
	}
}

func key2(i, j int) [2]int {
	if i > j {
		i, j = j, i
	}
	return [2]int{i, j}
}

// dedupePoses 移除与前一节点几乎重合的点（转弯弧末点 == 下一行首点）。
func dedupePoses(ps []Pose, tol float64) []Pose {
	out := ps[:1]
	for i := 1; i < len(ps); i++ {
		last := out[len(out)-1]
		if math.Hypot(ps[i].X-last.X, ps[i].Y-last.Y) > tol {
			out = append(out, ps[i])
		}
	}
	return out
}

// InjectedLoopScenario 在正确场景上注入一条错误回环：
// 把第一条行起点附近与最后一条行末端附近（物理上相距很远）误判为同一地点，
// 测量为近似恒等变换——典型的感知混叠错误回环。
func InjectedLoopScenario(base *Scenario) *Scenario {
	g := base.Graph.Clone()
	badI, badJ := 2, len(base.Truth)-3
	badM := Measurement{Dx: 0.03, Dy: 0.02, Dtheta: -0.01}
	idx := len(g.Edges)
	g.AddEdge(badI, badJ, Loop, badM, DiagInfo(0.03, 0.02))

	sc := *base
	sc.Name = "割草机往返扫描（注入错误回环）"
	sc.Description = "与正确场景相同的里程计与正确回环，但额外注入一条把场地两角误判为重访的错误回环，优化轨迹会被明显拽歪。"
	sc.Graph = g
	sc.InjectedBadLoops = append(append([]int(nil), base.InjectedBadLoops...), idx)
	return &sc
}

// chainInitialize 从节点 0 出发，仅按里程计边链式积分得到初始位姿。
func chainInitialize(g *Graph, n int) []Pose {
	init := make([]Pose, n)
	from := make([]int, n)
	meas := make([]Measurement, n)
	has := make([]bool, n)
	has[0] = true
	for _, e := range g.Edges {
		if e.Kind == Odom && e.J == e.I+1 && !has[e.J] {
			from[e.J], meas[e.J], has[e.J] = e.I, e.M, true
		}
	}
	for j := 1; j < n; j++ {
		if !has[j] {
			init[j] = init[j-1] // 兜底
			continue
		}
		init[j] = Compose(init[from[j]], meas[j])
	}
	return init
}

// TrapRingScenario 构造经典的闭合环局部最优陷阱：
// 真值是一个大圆（绕一圈后回到起点，存在一条闭合回环边 N-1↔0）；
// 若初值被错误地摆成“8 字形”（两个小圆，总周长与大圆相同），里程计链
// 自身可以很好地满足，整条链却被锁在双圈重合的盆地里，唯一的闭合边以大残差
// 顶着轨迹——GN 步在此处找不到下降方向，LM 会停在一个几何荒谬的驻点。
// 这是“数值上已到驻点、几何上完全错误”的代表性陷阱。
func TrapRingScenario(N int, radius float64, seed int64) *Scenario {
	rng := rand.New(rand.NewSource(seed))
	truth := make([]Pose, N)
	for i := 0; i < N; i++ {
		a := -math.Pi/2 + float64(i)*Tau/float64(N)
		truth[i] = Pose{X: radius * math.Cos(a), Y: radius * math.Sin(a), Theta: WrapAngle(a + math.Pi/2)}
	}

	const sigXY = 0.01
	const sigTh = 0.005
	info := DiagInfo(sigXY, sigTh)
	g := NewGraph(N)
	for i := 0; i+1 < N; i++ {
		m := Relative(truth[i], truth[i+1])
		m.Dx += gaussian(rng) * sigXY
		m.Dy += gaussian(rng) * sigXY
		m.Dtheta = WrapAngle(m.Dtheta + gaussian(rng)*sigTh)
		g.AddEdge(i, i+1, Odom, m, info)
	}
	// 闭合回环：最后一个节点与起点相邻（真实重访）。
	g.AddEdge(N-1, 0, Loop, Relative(truth[N-1], truth[0]), DiagInfo(0.01, 0.005))

	return &Scenario{
		Name:          "闭合圆环陷阱（8 字形坏初值）",
		Description:   "机器人沿圆走一圈并回环到起点；坏初值把轨迹摆成等周长的 8 字形。里程计链可被很好满足但闭合回环无法满足，LM 会停在双圈重合的荒谬驻点——数值上有结论，几何上却是陷阱。",
		Truth:         truth,
		Graph:         g,
		Initial:       FigureEightInitial(N, radius),
		KnownRevisits: map[[2]int]bool{},
	}
}

// FigureEightInitial 生成两个相切小圆组成的 8 字形初值：
// 两小圆半径之和使其总周长 (2·2πr) 等于大圆周长 (2πR)，故每步里程计步长精确匹配。
func FigureEightInitial(N int, radius float64) []Pose {
	ps := make([]Pose, N)
	r := radius / 2
	half := N / 2
	for i := 0; i < N; i++ {
		var cx, cy, a0 float64
		if i < half {
			cx, cy, a0 = -r, 0, 0 // 左圆：从切点 (0,0) 出发
		} else {
			cx, cy, a0 = r, 0, math.Pi // 右圆
		}
		t := float64(i%half) / float64(half) * Tau
		a := a0 + t
		x, y := cx+r*math.Cos(a), cy+r*math.Sin(a)
		th := a + math.Pi/2
		if i >= half {
			th = a - math.Pi/2
		}
		ps[i] = Pose{X: x, Y: y, Theta: WrapAngle(th)}
	}
	return ps
}

// TrapInitials 为陷阱场景生成三个差异化初值：8 字形（陷阱）、开环链式（好盆地）、
// 漂移扰动链式（另一个好盆地初值）。
func TrapInitials(g *Graph, trap []Pose) map[string][]Pose {
	n := g.N
	chain := chainInitialize(g, n)
	out := map[string][]Pose{}

	bad := make([]Pose, n)
	copy(bad, trap)
	out["8字形坏初值"] = bad

	good := make([]Pose, n)
	copy(good, chain)
	out["开环链式初值"] = good

	jitter := make([]Pose, n)
	for i, p := range chain {
		k := float64(i) / float64(n)
		jitter[i] = Pose{
			X:     p.X + 0.3*k*math.Sin(float64(i)*1.1),
			Y:     p.Y + 0.3*k*math.Cos(float64(i)*0.9),
			Theta: WrapAngle(p.Theta + 0.25*k),
		}
	}
	out["漂移扰动初值"] = jitter
	return out
}
