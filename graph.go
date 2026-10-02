package main

import "math"

// ConstraintKind 区分里程计约束与回环约束，诊断与可视化时区别对待。
type ConstraintKind string

const (
	KindOdom ConstraintKind = "odom"
	KindLoop ConstraintKind = "loop"
)

// Constraint 是节点 I 到节点 J 的相对位姿测量：
//
//	p_J ≈ p_I ⊕ Z
//
// Info 为 3×3 信息矩阵 Σ⁻¹ 的对角元素 (x, y, θ)。
type Constraint struct {
	ID   string         `json:"id"`
	Kind ConstraintKind `json:"kind"`
	I    int            `json:"i"`
	J    int            `json:"j"`
	Z    Pose           `json:"z"`
	Info [3]float64     `json:"info"`
}

// Graph 位姿图：节点初始位姿 + 相对约束。约定节点 0 为锚点（固定）。
type Graph struct {
	Name        string       `json:"name"`
	Nodes       []Pose       `json:"nodes"`
	Constraints []Constraint `json:"constraints"`
}

// residual 计算一条约束的 3 维残差：
//
//	e_xy = R(θ_i)ᵀ (p_j - p_i) - t_ij
//	e_θ  = wrap(θ_j - θ_i - θ_ij)
//
// 角度分量经 wrapAngle 规范化到 (-π, π]，保证 ±π 边界处误差连续。
func residual(c Constraint, nodes []Pose) [3]float64 {
	pi, pj := nodes[c.I], nodes[c.J]
	dx, dy := pj.X-pi.X, pj.Y-pi.Y
	ct, st := math.Cos(pi.Theta), math.Sin(pi.Theta)
	var e [3]float64
	e[0] = ct*dx + st*dy - c.Z.X
	e[1] = -st*dx + ct*dy - c.Z.Y
	e[2] = wrapAngle(pj.Theta - pi.Theta - c.Z.Theta)
	return e
}

// jacobian 返回残差对节点 I、J 位姿的 3×3 解析雅可比块（未白化）。
//
// 关键点：角度行只含 e_θ 对 θ_i/θ_j 的 ±1，且 e_θ 在进入雅可比之前
// 已做环绕规范化，因此无论 θ 取何值（含 ±π 边界），优化方向都不会反转。
func jacobian(c Constraint, nodes []Pose) (Ji, Jj [3][3]float64) {
	pi, pj := nodes[c.I], nodes[c.J]
	dx, dy := pj.X-pi.X, pj.Y-pi.Y
	ct, st := math.Cos(pi.Theta), math.Sin(pi.Theta)

	// 平移行
	Ji[0][0] = -ct
	Ji[0][1] = -st
	Ji[0][2] = -st*dx + ct*dy // d e_x / d θ_i
	Ji[1][0] = st
	Ji[1][1] = -ct
	Ji[1][2] = -ct*dx - st*dy // d e_y / d θ_i

	Jj[0][0] = ct
	Jj[0][1] = st
	Jj[1][0] = -st
	Jj[1][1] = ct

	// 角度行（其余项为 0）
	Ji[2][2] = -1
	Jj[2][2] = 1
	return Ji, Jj
}

// whitenedResidual 返回白化后的残差与雅可比块（乘 Σ^{-1/2}，此处为对角阵）。
func whitenedResidual(c Constraint, nodes []Pose) (e [3]float64, Ji, Jj [3][3]float64) {
	e = residual(c, nodes)
	Ji, Jj = jacobian(c, nodes)
	for d := 0; d < 3; d++ {
		s := math.Sqrt(c.Info[d])
		e[d] *= s
		for k := 0; k < 3; k++ {
			Ji[d][k] *= s
			Jj[d][k] *= s
		}
	}
	return e, Ji, Jj
}

// constraintChi2 计算一条约束在当前位姿下的加权误差平方 eᵀ Σ⁻¹ e。
func constraintChi2(c Constraint, nodes []Pose) float64 {
	e := residual(c, nodes)
	return e[0]*e[0]*c.Info[0] + e[1]*e[1]*c.Info[1] + e[2]*e[2]*c.Info[2]
}

// totalCost 全图加权误差平方和的 1/2。
func totalCost(g *Graph, nodes []Pose) float64 {
	s := 0.0
	for k := range g.Constraints {
		s += constraintChi2(g.Constraints[k], nodes)
	}
	return 0.5 * s
}

// cloneNodes 复制一份节点位姿，避免优化修改初始估计。
func cloneNodes(nodes []Pose) []Pose {
	out := make([]Pose, len(nodes))
	copy(out, nodes)
	return out
}
