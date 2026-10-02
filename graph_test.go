package main

import (
	"math"
	"os"
	"strings"
	"testing"
)

// approx 比较浮点。
func approx(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s = %v, want %v (tol %v)", name, got, want, tol)
	}
}

// ---- 一、正确回环：代价单调下降并收敛到低误差 ------------------------------

func TestCorrectLoop_MonotonicCostDecrease(t *testing.T) {
	s := buildCorrectScenario()
	opt := optimize(s.Graph, s.Initial, DefaultLMConfig())

	if opt.Status != "converged" {
		t.Fatalf("期望 converged，得到 %s", opt.Status)
	}
	if len(opt.Costs) < 3 {
		t.Fatalf("应当有若干次迭代记录，得到 %d 个代价点", len(opt.Costs))
	}
	for i := 1; i < len(opt.Costs); i++ {
		// Costs 只在步被接受时记录，因此必须（非严格）单调下降
		if opt.Costs[i] > opt.Costs[i-1]+1e-9 {
			t.Fatalf("代价在第 %d 次迭代上升: %.8g -> %.8g", i, opt.Costs[i-1], opt.Costs[i])
		}
	}
	// 初始代价应明显高于最终代价
	if opt.Costs[len(opt.Costs)-1] >= opt.Costs[0]/100 {
		t.Fatalf("优化未充分降低代价: %v -> %v", opt.Costs[0], opt.Costs[len(opt.Costs)-1])
	}
	// 与真值的偏差应远小于轨迹尺度
	d := diagnose(s.Graph, s.Initial, opt)
	if d.Status != "ok" {
		t.Fatalf("正确回环场景不应报警，得到 %s: %v", d.Status, d.Notes)
	}
	for i := range s.Truth {
		if dev := math.Hypot(opt.Nodes[i].X-s.Truth[i].X, opt.Nodes[i].Y-s.Truth[i].Y); dev > 0.2 {
			t.Fatalf("节点 %d 偏离真值 %.3fm", i, dev)
		}
	}
}

// ---- 二、错误回环被正确定位 -------------------------------------------------

func TestBadLoop_IsLocated(t *testing.T) {
	s := buildBadLoopScenario()
	if s.BadLoop != "loop-BAD" {
		t.Fatalf("测试夹具设置错误")
	}
	opt := optimize(s.Graph, s.Initial, DefaultLMConfig())
	d := diagnose(s.Graph, s.Initial, opt)

	if d.Status != "bad_loop" {
		t.Fatalf("期望 bad_loop，得到 %s", d.Status)
	}
	if d.CulpritID != "loop-BAD" {
		t.Fatalf("期望定位到 loop-BAD，得到 %q", d.CulpritID)
	}
	// 正确回环不能被牵连为可疑度最高者
	if len(d.LoopScores) < 2 {
		t.Fatalf("应对全部回环打分")
	}
	if d.LoopScores[0].ID != "loop-BAD" {
		t.Fatalf("可疑度最高的应为 loop-BAD，得到 %s", d.LoopScores[0].ID)
	}
	// 删除错误回环后，其余约束应能拟合到接近零代价
	top := d.LoopScores[0]
	if top.OwnResidual < suspectOwnChi2 {
		t.Fatalf("错误回环在留一解上的自身 χ² 应很大，得到 %.3f", top.OwnResidual)
	}
	if top.RemovedCost > 10.0 {
		t.Fatalf("删除错误回环后代价应很小，得到 %.3f", top.RemovedCost)
	}
}

// ---- 三、角度环绕边界处雅可比正确 -------------------------------------------

func TestWrapAngleBoundaries(t *testing.T) {
	approx(t, "wrap(π)", wrapAngle(math.Pi), math.Pi, 1e-12)
	// 约定主值区间为 (-π, π]：-π 应规范为 +π
	approx(t, "wrap(-π)", wrapAngle(-math.Pi), math.Pi, 1e-12)
	approx(t, "wrap(3π+0.1)", wrapAngle(3*math.Pi+0.1), wrapAngle(math.Pi+0.1), 1e-12)
	approx(t, "wrap(-3π-0.1)", wrapAngle(-3*math.Pi-0.1), wrapAngle(-math.Pi-0.1), 1e-12)

	// 边界两侧的角度在物理上必须相邻，而不是相差 2π
	a := wrapAngle(math.Pi - 0.01)
	b := wrapAngle(-math.Pi + 0.01)
	if d := wrapAngle(a - b); math.Abs(d) > 0.03 {
		t.Fatalf("边界两侧角度差应为 ~0.02，得到 %v", d)
	}
}

// numericalJac 用中心差分数值计算残差对 nodesI/nodesJ 位姿的雅可比。
// 对角度自由度按行差分（残差各行互不耦合到同一扰动列之外的量），
// 跨越 ±π 时状态经 wrapAngle 扰动，检验 residual 内部 wrap 的正确性。
func numericalJac(c Constraint, nodes []Pose) (Ji, Jj [3][3]float64) {
	base := residual(c, nodes)
	const eps = 1e-6
	diff := func(node, dof, row int) float64 {
		p := make([]Pose, len(nodes))
		copy(p, nodes)
		switch dof {
		case 0:
			p[node].X += eps
		case 1:
			p[node].Y += eps
		case 2:
			p[node].Theta = wrapAngle(p[node].Theta + eps)
		}
		return (residual(c, p)[row] - base[row]) / eps
	}
	for dof := 0; dof < 3; dof++ {
		for row := 0; row < 3; row++ {
			Ji[row][dof] = diff(c.I, dof, row)
			Jj[row][dof] = diff(c.J, dof, row)
		}
	}
	return Ji, Jj
}

func TestJacobianAtWrapBoundary(t *testing.T) {
	// 构造跨越 ±π 边界的角度配置：
	// θ_i = 3.05，θ_j = -3.05，Z.θ = 0.05
	// 未 wrap 的角度误差为 -6.15；wrap 后约为 +0.133。
	// 若忘记 wrap，梯度会把 θ 往完全相反的方向推。
	c := Constraint{
		I: 0, J: 1,
		Z:    Pose{X: 1.0, Y: 0.2, Theta: 0.05},
		Info: [3]float64{1, 1, 1},
	}
	nodes := []Pose{
		{X: 0, Y: 0, Theta: 3.05},
		{X: 1.1, Y: 0.1, Theta: -3.05},
	}

	e := residual(c, nodes)
	wantAng := wrapAngle(-3.05 - 3.05 - 0.05)
	approx(t, "wrap 后角度残差", e[2], wantAng, 1e-12)
	if math.Abs(e[2]) > math.Pi {
		t.Fatalf("角度残差必须在主值区间内，得到 %v", e[2])
	}
	// 关键：wrap 后角度残差为小的正值（0.133），而非 -6.15
	if e[2] <= 0 || e[2] > 0.2 {
		t.Fatalf("期望 wrap 后残差为小的正值，得到 %v；未处理环绕会得到 -6.15", e[2])
	}

	Ji, Jj := jacobian(c, nodes)
	ni, nj := numericalJac(c, nodes)
	for r := 0; r < 3; r++ {
		for col := 0; col < 3; col++ {
			if math.Abs(Ji[r][col]-ni[r][col]) > 1e-5 {
				t.Errorf("Ji[%d][%d] = %v, 数值雅可比 %v（跨越 ±π 边界）", r, col, Ji[r][col], ni[r][col])
			}
			if math.Abs(Jj[r][col]-nj[r][col]) > 1e-5 {
				t.Errorf("Jj[%d][%d] = %v, 数值雅可比 %v（跨越 ±π 边界）", r, col, Jj[r][col], nj[r][col])
			}
		}
	}
	// 角度雅可比在流形上恒为 ±1，与是否跨越环绕边界无关
	approx(t, "Ji 角度行 θ_i 项", Ji[2][2], -1, 1e-12)
	approx(t, "Jj 角度行 θ_j 项", Jj[2][2], +1, 1e-12)
}

// TestWrapBoundaryOptimizerDirection 从边界两侧初值出发应收敛到同一解，
// 证明环绕处理不会使优化方向反转。
func TestWrapBoundaryOptimizerDirection(t *testing.T) {
	mk := func(thetaJ float64) (*Graph, []Pose) {
		g := &Graph{
			Nodes: []Pose{{}, {}},
			Constraints: []Constraint{{
				ID: "c", Kind: KindOdom, I: 0, J: 1,
				Z:    Pose{X: 1, Theta: 0},
				Info: [3]float64{1, 1, 1},
			}},
		}
		init := []Pose{{}, {X: 1, Theta: thetaJ}}
		return g, init
	}
	cfg := DefaultLMConfig()
	gA, initA := mk(math.Pi - 0.05)
	gB, initB := mk(-math.Pi + 0.05)
	optA := optimize(gA, initA, cfg)
	optB := optimize(gB, initB, cfg)
	// 两种等价的角度表示应得到一致的优化结果
	if d := wrapAngle(optA.Nodes[1].Theta - optB.Nodes[1].Theta); math.Abs(d) > 1e-6 {
		t.Fatalf("边界两侧初值应收敛到同一角度，差 %v", d)
	}
}

// ---- 四、局部最优陷阱被识别为“几何荒谬的收敛”，而非成功或失败 ---------------

func TestLocalMinimum_IsDetectedNotReportedAsNormal(t *testing.T) {
	s := buildLocalMinScenario()
	opt := optimize(s.Graph, s.Initial, DefaultLMConfig())
	d := diagnose(s.Graph, s.Initial, opt)

	// 优化器本身认为自己数值收敛了——这正是陷阱所在
	if opt.Status != "converged" {
		t.Fatalf("该驻点应被 LM 判为收敛（步长容差满足），得到 %s", opt.Status)
	}
	// 诊断不能把它当正常收敛
	if d.Status != "local_minimum" {
		t.Fatalf("期望 local_minimum，得到 %s", d.Status)
	}
	if d.SelfIntersections <= 0 {
		t.Fatalf("折叠轨迹应存在非相邻段自穿越")
	}
	// 与真值偏差应很大（轨迹被折叠，最大偏差约 17m）
	maxDev := 0.0
	for i := range s.Truth {
		if dev := math.Hypot(opt.Nodes[i].X-s.Truth[i].X, opt.Nodes[i].Y-s.Truth[i].Y); dev > maxDev {
			maxDev = dev
		}
	}
	if maxDev < 5.0 {
		t.Fatalf("局部最优解应严重偏离真值，最大偏差仅 %.3fm", maxDev)
	}
	// 角度残差呈环绕数均摊特征：每条边一个小的恒定角度误差
	approx(t, "均摊角度残差 2π/N", d.MaxOdomAngErr, 2*math.Pi/48, 1e-3)
	// 正确回环不应被误判为错误回环
	if d.CulpritID != "" {
		t.Fatalf("局部最优场景不应定位出错误回环，得到 %q", d.CulpritID)
	}
}

// ---- 附加：静态报告为单一自包含文件 -----------------------------------------

func TestReportSelfContained(t *testing.T) {
	path := t.TempDir() + "/report.html"
	if err := writeHTMLReport(path, runAll()); err != nil {
		t.Fatal(err)
	}
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	html := string(b)
	for _, banned := range []string{`src="http`, `href="http`, `<link`, `<img`} {
		if strings.Contains(html, banned) {
			t.Errorf("报告不允许外部依赖，发现 %q", banned)
		}
	}
	if !strings.Contains(html, "const RUNS =") || !strings.Contains(html, `getContext("2d")`) {
		t.Errorf("报告应内嵌数据与 canvas 绘图逻辑")
	}
	if info, err := os.Stat(path); err != nil || info.Size() == 0 {
		t.Fatalf("报告文件未生成或为空")
	}
}
