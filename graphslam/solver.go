package graphslam

import "math"

// SolveConfig 控制 Levenberg-Marquardt 求解过程。
type SolveConfig struct {
	MaxIterations int     // 最大迭代次数
	ToleranceStep float64 // 状态更新量收敛阈值
	ToleranceGrad float64 // 梯度范数收敛阈值
	LambdaInit    float64 // 初始阻尼因子
	HuberDelta    float64 // Huber 核阈值（以标准化残差计）；<=0 表示关闭鲁棒核
}

// DefaultConfig 返回演示场景下表现稳健的默认参数：关闭鲁棒核，
// 使错误回环定位基于“纯高斯残差”，诊断更直观；鲁棒核在显式启用时使用。
func DefaultConfig() SolveConfig {
	return SolveConfig{
		MaxIterations: 100,
		ToleranceStep: 1e-9,
		ToleranceGrad: 1e-9,
		LambdaInit:    1e-3,
		HuberDelta:    0,
	}
}

// EdgeResidual 记录单条边在当前解下的残差信息，用于错误回环定位与报告展示。
type EdgeResidual struct {
	Edge       int        // 边在 Graph.Edges 中的索引
	Kind       EdgeKind   // 边类型
	I, J       int        // 端点节点
	Error      [3]float64 // 归一化后的残差 e = h(x) - z
	Chi2       float64    // eᵀ Ω e（鲁棒核权重不改变此原始量，便于跨边比较）
	Weighted   float64    // 鲁棒核化后实际进入代价的 0.5*ρ
	Normalized float64    // sqrt(eᵀ Ω e)：按该边自身不确定度归一化的残差
	RobustW    float64    // 鲁棒权重（1 表示未被降权）
}

// SolveResult 是一次优化的完整结果。
type SolveResult struct {
	Poses       []Pose         // 优化后的位姿
	Initial     []Pose         // 优化前的初始位姿
	CostHistory []float64      // 每次被接受迭代后的 0.5*Σρ 代价（首项为初始代价）
	Chi2History []float64      // 对应的 Σ eᵀΩe（无鲁棒化），用于误差单调下降判定
	Residuals   []EdgeResidual // 各边最终残差
	Iterations  int
	Converged   bool // 数值意义上的收敛（步长/梯度足够小）
	StopReason  string
	FinalCost   float64
	FinalChi2   float64
}

// huberWeight 实现 Huber 鲁棒核权重：r<=δ 时 w=1（二次区），否则 w=δ/r（线性区）。
func huberWeight(r, delta float64) float64 {
	if delta <= 0 || r <= delta {
		return 1
	}
	return delta / r
}

// edgeJacobians 返回误差 e = h(xi,xj) - z 对节点 i、j 位姿的 3×3 雅可比。
//
// 角度环绕的正确处理方式不是修改导数，而是让残差取最短弧上的值：
//
//	eθ = Wrap(Wrap(θj-θi) - zθ)
//
// 在被选中的 (-π,π] 分支内 eθ 是线性的，∂eθ/∂θi=-1、∂eθ/∂θj=+1；
// 若不归一化，例如真实误差 0.05rad 而原始差值为 -(2π-0.05)，
// 梯度会驱使角度沿“绕远一整圈”的方向更新，优化方向完全相反。
func edgeJacobians(e *Edge, poses []Pose) (Ai, Aj Mat3) {
	_, _, _, px, py := edgeError(e, poses)
	ci, si := math.Cos(poses[e.I].Theta), math.Sin(poses[e.I].Theta)
	Ai = Mat3{
		-ci, -si, py,
		si, -ci, -px,
		0, 0, -1,
	}
	Aj = Mat3{
		ci, si, 0,
		-si, ci, 0,
		0, 0, 1,
	}
	return
}

// edgeError 计算一条边在给定位姿下的预测误差（角度已 Wrap 到最短弧）。
func edgeError(e *Edge, poses []Pose) (ex, ey, et, px, py float64) {
	pi, pj := poses[e.I], poses[e.J]
	dxw, dyw := pj.X-pi.X, pj.Y-pi.Y
	c, s := math.Cos(pi.Theta), math.Sin(pi.Theta)
	// 预测的相对平移（i 机体坐标系），同时也是雅可比所需的中间量。
	px = c*dxw + s*dyw
	py = -s*dxw + c*dyw
	ex = px - e.M.Dx
	ey = py - e.M.Dy
	// 关键：角度残差先做环绕归一化再进代价；
	// 在被选中的最短弧分支内，角度误差对 θi/θj 的导数恒为 -1/+1。
	et = WrapAngle(WrapAngle(pj.Theta-pi.Theta) - e.M.Dtheta)
	return
}

// edgeResiduals 评估所有边的残差与代价分量。
func edgeResiduals(g *Graph, poses []Pose, huber float64) ([]EdgeResidual, float64, float64) {
	res := make([]EdgeResidual, len(g.Edges))
	var cost, chi2Total float64
	for k := range g.Edges {
		e := &g.Edges[k]
		ex, ey, et, _, _ := edgeError(e, poses)
		er := [3]float64{ex, ey, et}
		chi := er[0]*er[0]*e.Info[0] + er[1]*er[1]*e.Info[4] + er[2]*er[2]*e.Info[8]
		r := math.Sqrt(math.Max(chi, 0))
		w := huberWeight(r, huber)
		rc := 0.5 * w * chi
		cost += rc
		res[k] = EdgeResidual{
			Edge: k, Kind: e.Kind, I: e.I, J: e.J,
			Error: er, Chi2: chi, Weighted: rc, Normalized: r, RobustW: w,
		}
		chi2Total += chi
	}
	return res, cost, chi2Total
}

// CostAt 返回给定位姿下的鲁棒化代价（供剔除实验等外部评估使用）。
func CostAt(g *Graph, poses []Pose, huber float64) float64 {
	_, cost, _ := edgeResiduals(g, poses, huber)
	return cost
}

// Optimize 用 Levenberg-Marquardt 求图 g 的最小二乘解，initial 为初始位姿。
// 节点 0 通过强先验锚定，固定全局坐标系。
func Optimize(g *Graph, initial []Pose, cfg SolveConfig) *SolveResult {
	if len(initial) != g.N {
		panic("graphslam: initial pose count mismatch")
	}
	poses := make([]Pose, g.N)
	copy(poses, initial)

	_, cost, chi2 := edgeResiduals(g, poses, cfg.HuberDelta)
	res0 := &SolveResult{
		Initial: append([]Pose(nil), initial...),
	}

	dim := 3 * g.N
	// 锚点先验强度：远大于普通边信息，既固定全局位姿又不显著扭曲解。
	const anchorWeight = 1e10

	lambda := cfg.LambdaInit
	histCost := []float64{cost}
	histChi2 := []float64{chi2}
	reason := "max-iterations"
	converged := false
	iter := 0

	for iter = 0; iter < cfg.MaxIterations; iter++ {
		H := make([]float64, dim*dim)
		bvec := make([]float64, dim)

		for k := range g.Edges {
			e := &g.Edges[k]
			ex, ey, et, _, _ := edgeError(e, poses)
			er := [3]float64{ex, ey, et}
			r := math.Sqrt(math.Max(er[0]*er[0]*e.Info[0]+er[1]*er[1]*e.Info[4]+er[2]*er[2]*e.Info[8], 0))
			w := huberWeight(r, cfg.HuberDelta)

			Ai, Aj := edgeJacobians(e, poses)
			om := e.Info
			// 加权信息矩阵 Ω' = w·Ω。
			W := Mat3{
				w * om[0], w * om[1], w * om[2],
				w * om[3], w * om[4], w * om[5],
				w * om[6], w * om[7], w * om[8],
			}
			fillGNBlocks(H, dim, e.I, e.J, &Ai, &Aj, &W)
			// 右端项 b = -Jᵀ Ω e。
			addRHS(bvec, e.I, &Ai, &W, &er, -1)
			addRHS(bvec, e.J, &Aj, &W, &er, -1)
		}

		// 节点 0 的锚点先验：H00 += anchor·I
		for d := 0; d < 3; d++ {
			H[d*dim+d] += anchorWeight
		}

		gradNorm := 0.0
		for _, v := range bvec {
			gradNorm += v * v
		}
		gradNorm = math.Sqrt(gradNorm)
		if gradNorm < cfg.ToleranceGrad {
			reason = "gradient-tolerance"
			converged = true
			break
		}

		// 带阻尼求解与回退：λ 按代价是否下降自适应缩放。
		var step []float64
		var newPoses []Pose
		var newCost, newChi2 float64
		accepted := false

		for tries := 0; tries < 20; tries++ {
			Hlm := make([]float64, len(H))
			copy(Hlm, H)
			for i := 0; i < dim; i++ {
				Hlm[i*dim+i] *= 1 + lambda
			}
			sol, err := solveSPD(Hlm, bvec, dim)
			if err != nil {
				lambda *= 10
				continue
			}
			cand := applyStep(poses, sol)
			_, cs, c2 := edgeResiduals(g, cand, cfg.HuberDelta)
			if cs < cost {
				step, newPoses, newCost, newChi2 = sol, cand, cs, c2
				accepted = true
				break
			}
			lambda *= 10
		}

		if !accepted {
			reason = "no-descent-direction"
			converged = true // 已无法继续下降，视为驻点
			break
		}

		stepNorm := 0.0
		for _, v := range step {
			stepNorm += v * v
		}
		stepNorm = math.Sqrt(stepNorm)

		poses = newPoses
		cost, chi2 = newCost, newChi2
		lambda *= 0.5
		if lambda < 1e-12 {
			lambda = 1e-12
		}
		histCost = append(histCost, cost)
		histChi2 = append(histChi2, chi2)

		if stepNorm < cfg.ToleranceStep {
			reason = "step-tolerance"
			converged = true
			break
		}
	}

	finalRes, finalCost, finalChi2 := edgeResiduals(g, poses, cfg.HuberDelta)
	res0.Poses = poses
	res0.CostHistory = histCost
	res0.Chi2History = histChi2
	res0.Residuals = finalRes
	res0.Iterations = len(histCost) - 1
	res0.Converged = converged
	res0.StopReason = reason
	res0.FinalCost = finalCost
	res0.FinalChi2 = finalChi2
	return res0
}

// applyStep 把解向量 δx 叠加到位姿上，角度增量同样经过 Wrap 归一化。
func applyStep(poses []Pose, dx []float64) []Pose {
	out := make([]Pose, len(poses))
	for i := range poses {
		out[i] = Pose{
			X:     poses[i].X + dx[3*i],
			Y:     poses[i].Y + dx[3*i+1],
			Theta: WrapAngle(poses[i].Theta + dx[3*i+2]),
		}
	}
	return out
}

// mat3Mul 返回 A*B（3×3）。
func mat3Mul(a, b *Mat3) Mat3 {
	var r Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			s := 0.0
			for k := 0; k < 3; k++ {
				s += a[i*3+k] * b[k*3+j]
			}
			r[i*3+j] = s
		}
	}
	return r
}

// mat3T 转置。
func mat3T(a *Mat3) Mat3 {
	return Mat3{
		a[0], a[3], a[6],
		a[1], a[4], a[7],
		a[2], a[5], a[8],
	}
}

// blockJTWJ 计算 Jᵀ W J（3×3）。
func blockJTWJ(J, W *Mat3) Mat3 {
	Jt := mat3T(J)
	tmp := mat3Mul(&Jt, W)
	return mat3Mul(&tmp, J)
}

// blockJTWK 计算 Jᵀ W K（3×3）。
func blockJTWK(J, W, K *Mat3) Mat3 {
	Jt := mat3T(J)
	tmp := mat3Mul(&Jt, W)
	return mat3Mul(&tmp, K)
}

// putBlock 把 3×3 块 B 加到 H 的 (a,b) 块位置。
func putBlock(H []float64, n, a, b int, B *Mat3) {
	for dr := 0; dr < 3; dr++ {
		for dc := 0; dc < 3; dc++ {
			H[(3*a+dr)*n+(3*b+dc)] += B[dr*3+dc]
		}
	}
}

// fillGNBlocks 累加一条边对 Gauss-Newton 法方程四块的贡献：
// Hii += AiᵀWAi, Hij += AiᵀWAj, Hji = Hijᵀ, Hjj += AjᵀWAj。
func fillGNBlocks(H []float64, n, i, j int, Ai, Aj, W *Mat3) {
	ii := blockJTWJ(Ai, W)
	jj := blockJTWJ(Aj, W)
	ij := blockJTWK(Ai, W, Aj)
	putBlock(H, n, i, i, &ii)
	putBlock(H, n, j, j, &jj)
	putBlock(H, n, i, j, &ij)
	// H 整体按对称矩阵使用：转置块写到 (j,i)。
	ijt := mat3T(&ij)
	putBlock(H, n, j, i, &ijt)
}

// addRHS 累加 rhs(node) += sign · Jᵀ W e。
func addRHS(rhs []float64, node int, J, W *Mat3, e *[3]float64, sign float64) {
	Jt := mat3T(J)
	tmp := mat3Mul(&Jt, W)
	for r := 0; r < 3; r++ {
		s := 0.0
		for k := 0; k < 3; k++ {
			s += tmp[r*3+k] * e[k]
		}
		rhs[3*node+r] += sign * s
	}
}
