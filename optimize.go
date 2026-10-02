package main

import "math"

// OptimizeResult 记录一次非线性最小二乘求解的过程与结果。
type OptimizeResult struct {
	Nodes    []Pose    // 优化后位姿
	Costs    []float64 // 每次（被接受的）迭代开始时的 0.5·Σχ²，含初始值
	Status   string    // "converged" | "max_iterations"
	Iter     int       // 接受的迭代次数
	FinalLam float64
}

// LMConfig 控制 Levenberg-Marquardt 行为。
type LMConfig struct {
	MaxIter    int
	TolStep    float64 // 步长 ‖δ‖∞ 小于此值认为收敛
	TolCost    float64 // 代价相对下降小于此值认为收敛
	InitLambda float64
	RobustK    float64 // >0 时使用 Huber 鲁棒核（k 取标准化残差阈值）；0 表示纯二乘
}

// DefaultLMConfig 给出对本项目场景较稳健的默认参数。
func DefaultLMConfig() LMConfig {
	return LMConfig{
		MaxIter:    100,
		TolStep:    1e-8,
		TolCost:    1e-10,
		InitLambda: 1e-3,
	}
}

// optimize 用 Levenberg-Marquardt（Gauss-Newton 的阻尼形式）求解。
// 自由度只包含节点 1..N-1，节点 0 固定以消除规范自由度（整体平移/旋转）。
//
// 角度环绕处理：
//   - 残差角度在 residual() 中 wrap 到 (-π,π]，保证 ±π 附近误差连续；
//   - 角度雅可比恒为 ±1（流形上的局部增量），不随环绕跳变；
//   - 每次位姿更新后 θ 重新 wrap，使状态始终落在主值区间。
func optimize(g *Graph, initial []Pose, cfg LMConfig) OptimizeResult {
	nodes := cloneNodes(initial)
	n := len(g.Nodes)
	dim := 3 * (n - 1) // 节点 0 固定

	// nodeDOF 返回节点 i 的自由度基索引；节点 0 不存在自由度。
	idx := func(i int) int { return 3 * (i - 1) }

	cost := totalCost(g, nodes)
	res := OptimizeResult{Nodes: nodes, Costs: []float64{cost}, Status: "max_iterations", FinalLam: cfg.InitLambda}
	lambda := cfg.InitLambda

	for iter := 0; iter < cfg.MaxIter; iter++ {
		H := make([]float64, dim*dim)
		rhs := make([]float64, dim)

		for k := range g.Constraints {
			c := &g.Constraints[k]
			e, Ji, Jj := whitenedResidual(*c, nodes)

			// Huber 鲁棒权重：w = 1 (r≤k)，w = k/r (r>k)，r = ‖白化残差‖。
			// 鲁棒核只改变各约束的整体权重，不影响雅可比结构，
			// 因此角度 wrap 的正确性保持不变。
			w := 1.0
			if cfg.RobustK > 0 {
				r := math.Sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2])
				if r > cfg.RobustK {
					w = cfg.RobustK / r
				}
			}

			addBlock := func(node int, J [3][3]float64) {
				if node == 0 {
					return
				}
				base := idx(node)
				for a := 0; a < 3; a++ {
					for b := 0; b < 3; b++ {
						var h float64
						for d := 0; d < 3; d++ {
							h += J[d][a] * w * J[d][b]
						}
						H[(base+a)*dim+base+b] += h
					}
					var gv float64
					for d := 0; d < 3; d++ {
						gv += J[d][a] * w * e[d]
					}
					rhs[base+a] += gv
				}
			}

			if c.I != c.J {
				addBlock(c.I, Ji)
				addBlock(c.J, Jj)
				// 交叉块 H_ij = J_iᵀ W J_j
				if c.I != 0 && c.J != 0 {
					bi, bj := idx(c.I), idx(c.J)
					for a := 0; a < 3; a++ {
						for b := 0; b < 3; b++ {
							var h float64
							for d := 0; d < 3; d++ {
								h += Ji[d][a] * w * Jj[d][b]
							}
							H[(bi+a)*dim+bj+b] += h
							H[(bj+b)*dim+bi+a] += h
						}
					}
				}
			}
		}

		// 阻尼最小二乘：(H + λ·diag(H)) δ = -g
		A := make([]float64, len(H))
		copy(A, H)
		for i := 0; i < dim; i++ {
			A[i*dim+i] += lambda * H[i*dim+i]
		}
		for i := range rhs {
			rhs[i] = -rhs[i]
		}

		delta, err := choleskySolve(A, dim, rhs)
		if err != nil {
			// 奇异：加大阻尼重试，避免被误判为“收敛失败”
			lambda *= 10
			if lambda > 1e12 {
				res.Status = "max_iterations"
				return res
			}
			continue
		}

		// 候选状态（θ 更新后 wrap）
		cand := cloneNodes(nodes)
		maxStep := 0.0
		for i := 1; i < n; i++ {
			b := idx(i)
			cand[i].X += delta[b]
			cand[i].Y += delta[b+1]
			cand[i].Theta = wrapAngle(cand[i].Theta + delta[b+2])
			for _, v := range delta[b : b+3] {
				if a := math.Abs(v); a > maxStep {
					maxStep = a
				}
			}
		}

		newCost := totalCost(g, cand)
		if newCost < cost {
			rel := (cost - newCost) / math.Max(cost, 1e-30)
			nodes = cand
			cost = newCost
			res.Costs = append(res.Costs, cost)
			res.Iter++
			lambda = math.Max(lambda*0.5, 1e-9)
			res.FinalLam = lambda
			if maxStep < cfg.TolStep || rel < cfg.TolCost {
				res.Status = "converged"
				res.Nodes = nodes
				return res
			}
		} else {
			// 拒绝步，增大阻尼
			lambda *= 4
			if lambda > 1e12 {
				res.Status = "max_iterations"
				res.Nodes = nodes
				return res
			}
		}
	}
	res.Nodes = nodes
	res.Status = "max_iterations"
	return res
}
