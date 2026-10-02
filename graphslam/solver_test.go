package graphslam

import (
	"math"
	"testing"
)

// near 比较两个浮点数是否足够接近。
func near(t *testing.T, got, want, tol float64, msg string) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Fatalf("%s: got %v, want %v (tol %v)", msg, got, want, tol)
	}
}

// TestWrapAngle 验证角度环绕归一化本身：边界、符号与周期。
func TestWrapAngle(t *testing.T) {
	cases := []struct{ in, want float64 }{
		{0, 0},
		{3.0, 3.0},
		{-3.0, -3.0},
		{Tau, 0},
		{-Tau, 0},
		{Tau + 0.1, 0.1},
		{-Tau - 0.1, -0.1},
		{3 * Tau / 2, math.Pi},
		{-3 * Tau / 2, math.Pi},
		{3 * math.Pi / 2, -math.Pi / 2},
		{-3 * math.Pi / 2, math.Pi / 2},
		{math.Pi + 0.01, -math.Pi + 0.01}, // 越过 +π 应折回 -π 一侧
		{-math.Pi - 0.01, math.Pi - 0.01},
	}
	for _, c := range cases {
		got := WrapAngle(c.in)
		if math.Abs(got-c.want) > 1e-12 {
			t.Errorf("WrapAngle(%v)=%v, want %v", c.in, got, c.want)
		}
		if got > math.Pi || got <= -math.Pi {
			t.Errorf("WrapAngle(%v)=%v 超出 (-π,π]", c.in, got)
		}
	}
	near(t, WrapAngle(math.Pi), math.Pi, 1e-12, "+π 应保留在 (+π) 分支")
}

// TestMonotonicChi2CorrectLoops：正确回环场景下 χ² 在每次被接受迭代上单调不增，
// 且最终远低于初始（漂移被消除）。
func TestMonotonicChi2CorrectLoops(t *testing.T) {
	sc := LawnmowerScenario(4, 8, 2.0, 7)
	res := Optimize(sc.Graph, sc.Initial, DefaultConfig())

	if len(res.Chi2History) < 2 {
		t.Fatalf("迭代次数过少: %d", len(res.Chi2History)-1)
	}
	for k := 1; k < len(res.Chi2History); k++ {
		if res.Chi2History[k] > res.Chi2History[k-1]+1e-9 {
			t.Fatalf("χ² 未单调下降：第 %d 次迭代 %.6f > %.6f",
				k, res.Chi2History[k], res.Chi2History[k-1])
		}
	}
	if res.FinalChi2 >= 0.1*res.Chi2History[0] {
		t.Fatalf("优化未显著降低误差：%.4f -> %.4f", res.Chi2History[0], res.FinalChi2)
	}
	if !res.Converged {
		t.Fatalf("正确回环场景应数值收敛，实际 %s", res.StopReason)
	}
	// 收敛解应接近真值。
	for i, p := range res.Poses {
		q := sc.Truth[i]
		if d := math.Hypot(p.X-q.X, p.Y-q.Y); d > 0.3 {
			t.Fatalf("节点 %d 偏离真值 %.3f", i, d)
		}
	}
	// 正确回环不应误报。
	rep := LocateBadLoops(sc.Graph, res.Poses, res, DefaultBadLoopConfig())
	if len(rep.BadEdges) != 0 {
		t.Fatalf("正确回环被误判为错误: %v", rep.BadEdges)
	}
	geo := ClassifyOutcome(res, DetectLocalOpt(sc.Graph, res, sc.KnownRevisits))
	if geo.Outcome != OutcomeConvergedOK {
		t.Fatalf("正确场景结局应为 %s，实际 %s（问题 %d）",
			OutcomeConvergedOK, geo.Outcome, len(geo.Issues))
	}
}

// TestBadLoopLocated：注入的错误回环必须被唯一定位；剔除它后其余回环恢复可满足。
func TestBadLoopLocated(t *testing.T) {
	clean := LawnmowerScenario(4, 8, 2.0, 7)
	sc := InjectedLoopScenario(clean)
	res := Optimize(sc.Graph, sc.Initial, DefaultConfig())
	rep := LocateBadLoops(sc.Graph, res.Poses, res, DefaultBadLoopConfig())

	if len(sc.InjectedBadLoops) != 1 {
		t.Fatalf("测试前置条件错误：应恰好注入 1 条坏边，实际 %d", len(sc.InjectedBadLoops))
	}
	want := sc.InjectedBadLoops[0]
	if len(rep.BadEdges) != 1 || rep.BadEdges[0] != want {
		t.Fatalf("错误回环定位错误：期望仅 %v，实际 %v", sc.InjectedBadLoops, rep.BadEdges)
	}
	// 定位到的边必须同时给出“剔除后冲突消失”的因果证据。
	var v BadLoopVerdict
	for _, cand := range rep.Verdicts {
		if cand.Edge == want {
			v = cand
		}
	}
	if v.MaxOtherLoopResidual >= DefaultBadLoopConfig().OtherLoopFloor {
		t.Fatalf("剔除真凶后其余回环仍有大残差 %.2fσ", v.MaxOtherLoopResidual)
	}
	if v.LeaveOneOutImprovement <= 0 {
		t.Fatalf("剔除真凶后代价未下降: %v", v.LeaveOneOutImprovement)
	}
	// 其他正确回环必须被宣告无罪。
	for _, cand := range rep.Verdicts {
		if cand.Edge != want && cand.Suspicious {
			t.Fatalf("正确回环 #%d 被误判（其余回环残差 %.2fσ）", cand.Edge, cand.MaxOtherLoopResidual)
		}
	}
}

// TestAngleWrappingJacobian：在角度环绕边界附近，雅可比必须与“Wrap 后残差”的
// 有限差分一致——这是“角度相关项不被环绕带偏”的直接数值验证。
func TestAngleWrappingJacobian(t *testing.T) {
	// 构造一条相对角度使原始角差越过 +π 分支的边：
	// θj-θi ≈ π+0.05（未 Wrap），测量 zθ 使其最短弧残差落在 -π+ 一侧。
	edge := &Edge{
		I: 0, J: 1,
		M:    Measurement{Dx: 1.0, Dy: 0.3, Dtheta: 0.0},
		Info: DiagInfo(0.1, 0.1),
	}
	// 两个边界两侧的配置：raw diff = π±ε 与 -π±ε。
	configs := []float64{
		math.Pi - 0.05, math.Pi + 0.05,
		-math.Pi + 0.05, -math.Pi - 0.05,
	}
	const eps = 1e-6
	for _, dtheta := range configs {
		poses := []Pose{{X: 0, Y: 0, Theta: 0}, {X: 1.0, Y: 0.3, Theta: WrapAngle(dtheta)}}
		Ai, Aj := edgeJacobians(edge, poses)

		// 角度行（第 3 行）解析值恒为 ∂eθ/∂θi=-1、∂eθ/∂θj=+1。
		near(t, Ai[6], 0, 1e-12, "∂eθ/∂xi 应为 0")
		near(t, Ai[7], 0, 1e-12, "∂eθ/∂yi 应为 0")
		near(t, Ai[8], -1, 1e-12, "∂eθ/∂θi 应为 -1（最短弧分支内）")
		near(t, Aj[6], 0, 1e-12, "∂eθ/∂xj 应为 0")
		near(t, Aj[7], 0, 1e-12, "∂eθ/∂yj 应为 0")
		near(t, Aj[8], +1, 1e-12, "∂eθ/∂θj 应为 +1（最短弧分支内）")

		// 与 Wrap 后残差的中心有限差分逐项核对（3 行 × 6 列）。
		baseErr := func(ps []Pose) [3]float64 {
			ex, ey, et, _, _ := edgeError(edge, ps)
			return [3]float64{ex, ey, et}
		}
		analytic := []Mat3{Ai, Aj}
		for node := 0; node < 2; node++ {
			for dim := 0; dim < 3; dim++ {
				pp := []Pose{poses[0], poses[1]}
				pm := []Pose{poses[0], poses[1]}
				vp := &pp[node]
				vm := &pm[node]
				switch dim {
				case 0:
					vp.X += eps
					vm.X -= eps
				case 1:
					vp.Y += eps
					vm.Y -= eps
				case 2:
					vp.Theta = WrapAngle(vp.Theta + eps)
					vm.Theta = WrapAngle(vm.Theta - eps)
				}
				ep, em := baseErr(pp), baseErr(pm)
				for row := 0; row < 3; row++ {
					fd := (ep[row] - em[row]) / (2 * eps)
					got := analytic[node][row*3+dim]
					if math.Abs(fd-got) > 1e-5 {
						t.Fatalf("raw=%.3f 节点%d 维度%d 残差行%d：解析 %.6f 与有限差分 %.6f 不符",
							dtheta, node, dim, row, got, fd)
					}
				}
			}
		}
	}
}

// TestWrappingGradientDirection：环绕边界处优化方向必须指向最短弧。
// 真实误差仅 -0.05rad，但原始差值是 2π-0.05；若不做 Wrap，梯度会指向绕一整圈的错误方向。
func TestWrappingGradientDirection(t *testing.T) {
	edge := &Edge{
		I: 0, J: 1,
		// 声称 j 相对 i 的角度为 0；当前实际相对角为 +0.05（最短弧意义）。
		M:    Measurement{Dx: 1, Dy: 0, Dtheta: 0},
		Info: DiagInfo(0.1, 0.1),
	}
	// 故意把 θj 放在 2π-0.05 附近：未归一化的原始差为 2π-0.05。
	poses := []Pose{{X: 0, Y: 0, Theta: 0}, {X: 1, Y: 0, Theta: Tau - 0.05}}
	_, _, et, _, _ := edgeError(edge, poses)
	if math.Abs(et-(-0.05)) > 1e-9 {
		t.Fatalf("Wrap 后角度残差应为 -0.05（最短弧），实际 %v", et)
	}
	// 解析雅可比给出的梯度方向 bθi=-∂e/∂θi 等应推动 θj 减小 |残差|。
	_, Aj := edgeJacobians(edge, poses)
	// Jθj 对 eθ 为 +1，信息加权梯度分量 JᵀΩe 的角度符号 = eθ<0；
	// GN 下降步 δθj = -(JᵀΩJ)^-1 JᵀΩe 应与 -eθ 同号，即 +0.05 方向（最短弧）。
	signStep := -math.Copysign(1, et) * Aj[8] // = +1
	if signStep <= 0 {
		t.Fatalf("下降步方向错误：δθj 应朝最短弧（正方向）")
	}
}

// TestLocalOptTrapDetected：8 字形坏初值困住 LM 时，必须报 local-minimum-trap，
// 而不是仅因“到达驻点”报成功；同一图从好初值出发必须判为 converged-ok（防误报）。
func TestLocalOptTrapDetected(t *testing.T) {
	sc := TrapRingScenario(30, 5.0, 7)
	cfg := DefaultConfig()

	trapped := Optimize(sc.Graph, sc.Initial, cfg)
	trapGeo := ClassifyOutcome(trapped, DetectLocalOpt(sc.Graph, trapped, sc.KnownRevisits))
	if trapGeo.Outcome != OutcomeTrap {
		t.Fatalf("8字形坏初值应判定为 %s，实际 %s（收敛=%v, χ²/边=%.1f, 几何问题=%d）",
			OutcomeTrap, trapGeo.Outcome, trapped.Converged,
			trapGeo.Chi2PerDOF, len(trapGeo.Issues))
	}
	// 陷阱解必须真的几何荒谬：至少一处自穿越或多处异常重合。
	nIntersect, nCoincide := 0, 0
	for _, is := range trapGeo.Issues {
		switch is.Kind {
		case "self-intersection":
			nIntersect++
		case "node-coincidence":
			nCoincide++
		}
	}
	if nIntersect+nCoincide == 0 {
		t.Fatal("陷阱解未检测到任何自穿越/异常重合")
	}

	// 同一图、好初值：不得误报。
	inits := TrapInitials(sc.Graph, sc.Initial)
	good := Optimize(sc.Graph, inits["开环链式初值"], cfg)
	goodGeo := ClassifyOutcome(good, DetectLocalOpt(sc.Graph, good, sc.KnownRevisits))
	if goodGeo.Outcome != OutcomeConvergedOK {
		t.Fatalf("好初值应判定为 %s，实际 %s（几何问题=%d）",
			OutcomeConvergedOK, goodGeo.Outcome, len(goodGeo.Issues))
	}
	if good.FinalChi2 >= trapped.FinalChi2 {
		t.Fatalf("好盆地代价应显著低于陷阱：%.4f vs %.4f", good.FinalChi2, trapped.FinalChi2)
	}

	// 多起点优化必须逃出陷阱并选到几何可信的解。
	ms := OptimizeMultiStart(sc.Graph, sc.KnownRevisits, inits, cfg)
	if ms.Best < 0 || !ms.Starts[ms.Best].Plausible {
		t.Fatal("多起点优化未能选出几何可信的解")
	}
	if math.Abs(ms.Starts[ms.Best].Result.FinalChi2-good.FinalChi2) > 1.0 {
		t.Fatalf("恢复解代价 %.4f 与好盆地 %.4f 不符",
			ms.Starts[ms.Best].Result.FinalChi2, good.FinalChi2)
	}
}

// TestCholeskySolvesKnownSystem：自研线性求解器在已知正定系统上必须给出正确解。
func TestCholeskySolvesKnownSystem(t *testing.T) {
	// A = L Lᵀ，构造正定矩阵与已知解。
	A := []float64{
		4, 1, 1,
		1, 5, 2,
		1, 2, 6,
	}
	xWant := []float64{1, -2, 0.5}
	b := make([]float64, 3)
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			b[i] += A[i*3+j] * xWant[j]
		}
	}
	x, err := solveSPD(A, b, 3)
	if err != nil {
		t.Fatal(err)
	}
	for i := range x {
		near(t, x[i], xWant[i], 1e-9, "cholesky 解分量")
	}
}

// TestHuberWeight：鲁棒核权重在二次区为 1、在线性区按 δ/r 衰减。
func TestHuberWeight(t *testing.T) {
	near(t, huberWeight(0.5, 2.0), 1.0, 1e-12, "二次区权重应为 1")
	near(t, huberWeight(4.0, 2.0), 0.5, 1e-12, "线性区权重应为 δ/r")
	near(t, huberWeight(100, 0), 1.0, 1e-12, "关闭核时权重恒为 1")
}
