package main

import (
	"math"
	"sort"
	"strconv"
)

// LoopScore 汇总一条回环约束的可疑度证据。
type LoopScore struct {
	ID           string  `json:"id"`
	I            int     `json:"i"`
	J            int     `json:"j"`
	ResidualNorm float64 `json:"residualNorm"` // 鲁棒优化后该回环的白化残差范数 r=√χ²
	RemovedCost  float64 `json:"removedCost"`  // 删除该回环后，其余约束优化到的总代价
	OwnResidual  float64 `json:"ownResidual"`  // 在“删除该回环”的解上，该回环自身的 χ²
	CostImpact   float64 `json:"costImpact"`   // 基准代价 - 删除后的代价（>0 表示它在拉扯全图）
	Suspicion    float64 `json:"suspicion"`
}

// Diagnosis 诊断结论。
type Diagnosis struct {
	Status            string      `json:"status"` // "ok" | "bad_loop" | "local_minimum"
	CulpritID         string      `json:"culpritId"`
	LoopScores        []LoopScore `json:"loopScores"`
	SelfIntersections int         `json:"selfIntersections"`
	MaxOdomPosErr     float64     `json:"maxOdomPosErr"`   // 最大里程计平移残差（米）
	MaxOdomErrRatio   float64     `json:"maxOdomErrRatio"` // 相对该边测量长度的比例
	MaxOdomAngErr     float64     `json:"maxOdomAngErr"`   // 最大里程计角度残差（弧度）
	FinalCost         float64     `json:"finalCost"`
	OptStatus         string      `json:"optStatus"`
	Notes             []string    `json:"notes"`
}

// 判定阈值。合成场景中正确约束优化后残差在 1e-6 量级，
// 错误约束在数米/数弧度量级，阈值取中间，留有充足裕度。
const (
	suspectResidual  = 2.0  // 鲁棒解上白化残差范数 > 2（约 2σ，超过 Huber k）即可疑
	suspectOwnChi2   = 1.0  // leave-one-out 解上自身 χ² 仍很大 => 与数据不符
	localMinErrRatio = 0.35 // 里程计边平移残差超过边长 35%
	localMinAngErr   = 0.8  // 约 46°
)

// diagnose 对一次优化结果做事后诊断：
//  1. 鲁棒优化 + leave-one-out 定位错误回环；
//  2. 无错误回环时做几何健全性检查，识别“数值收敛但几何荒谬”的局部最优。
func diagnose(g *Graph, initial []Pose, opt OptimizeResult) Diagnosis {
	d := Diagnosis{
		Status:    "ok",
		OptStatus: opt.Status,
		FinalCost: opt.Costs[len(opt.Costs)-1],
	}
	nodes := opt.Nodes

	// ---- 1. 回环可疑度打分 -------------------------------------------------
	var loopIdx []int
	for k := range g.Constraints {
		if g.Constraints[k].Kind == KindLoop {
			loopIdx = append(loopIdx, k)
		}
	}

	baseCost := totalCost(g, nodes)
	robCfg := DefaultLMConfig()
	robCfg.RobustK = 2.0
	robOpt := optimize(g, initial, robCfg) // 鲁棒解用于评估单约束残差
	robNodes := robOpt.Nodes

	for _, k := range loopIdx {
		c := g.Constraints[k]
		s := LoopScore{
			ID:           c.ID,
			I:            c.I,
			J:            c.J,
			ResidualNorm: math.Sqrt(constraintChi2(c, robNodes)),
		}

		// leave-one-out：删掉这条回环重新优化
		var reduced []Constraint
		for k2 := range g.Constraints {
			if k2 != k {
				reduced = append(reduced, g.Constraints[k2])
			}
		}
		g2 := &Graph{Nodes: g.Nodes, Constraints: reduced}
		sub := optimize(g2, initial, DefaultLMConfig())
		s.RemovedCost = totalCost(g2, sub.Nodes)
		s.OwnResidual = constraintChi2(c, sub.Nodes)
		s.CostImpact = baseCost - s.RemovedCost

		// 综合可疑度：鲁棒解上的残差 r 区分度最高（好回环≈1，坏回环≈1e3），
		// 作为主项；代价影响为辅。ownResidual 仅用作判定门控，不进分数
		// （删除不同回环后其余错误约束仍会把该值推高，直接加和会污染排序）。
		s.Suspicion = s.ResidualNorm + 0.05*math.Max(0, s.CostImpact)
		d.LoopScores = append(d.LoopScores, s)
	}
	sort.Slice(d.LoopScores, func(a, b int) bool {
		return d.LoopScores[a].Suspicion > d.LoopScores[b].Suspicion
	})

	if len(d.LoopScores) > 0 {
		top := d.LoopScores[0]
		if top.ResidualNorm > suspectResidual && top.OwnResidual > suspectOwnChi2 {
			d.Status = "bad_loop"
			d.CulpritID = top.ID
			d.Notes = append(d.Notes,
				"回环 "+top.ID+" 在鲁棒优化后仍无法拟合（白化残差 r="+
					ftos(top.ResidualNorm)+"），且删除它后其余约束代价显著下降；")
			d.Notes = append(d.Notes,
				"在删除该回环的解上其自身 χ²="+ftos(top.OwnResidual)+
					"，说明该测量与真实位姿拓扑不一致，判定为错误回环。")
		}
	}

	// ---- 2. 里程计残差与几何健全性 ----------------------------------------
	maxPos, maxAng, maxRatio := 0.0, 0.0, 0.0
	for k := range g.Constraints {
		c := g.Constraints[k]
		if c.Kind != KindOdom {
			continue
		}
		e := residual(c, nodes)
		pos := math.Hypot(e[0], e[1])
		edgeLen := math.Hypot(c.Z.X, c.Z.Y)
		ratio := pos / math.Max(edgeLen, 1e-9)
		if pos > maxPos {
			maxPos = pos
		}
		if a := math.Abs(e[2]); a > maxAng {
			maxAng = a
		}
		if ratio > maxRatio {
			maxRatio = ratio
		}
	}
	d.MaxOdomPosErr = maxPos
	d.MaxOdomAngErr = maxAng
	d.MaxOdomErrRatio = maxRatio
	d.SelfIntersections = countSelfIntersections(nodes)

	if d.Status == "ok" {
		switch {
		case d.SelfIntersections > 0:
			d.Status = "local_minimum"
			d.Notes = append(d.Notes, "优化器报告收敛，但优化轨迹存在 "+
				itoa(d.SelfIntersections)+" 处非相邻段自穿越——平面刚体轨迹不可能自交。")
		case maxRatio > localMinErrRatio || maxAng > localMinAngErr:
			d.Status = "local_minimum"
			d.Notes = append(d.Notes, "优化器报告收敛，但里程计链上最大平移残差达边长的 "+
				ftos(100*maxRatio)+"%，角度残差 "+ftos(maxAng)+" rad；")
			d.Notes = append(d.Notes, "残差集中在少数边而非均匀趋零，是陷入折叠/翻转局部最优的典型特征。")
		}
		if d.Status == "local_minimum" {
			d.Notes = append(d.Notes, "这不是“收敛失败”：梯度步长已满足容差，而是收敛到了几何上不合理的驻点。")
		}
	}
	return d
}

// segmentIntersect 判断线段 p1p2 与 p3p4 是否严格相交（不含共线重叠）。
func segmentIntersect(p1, p2, p3, p4 [2]float64) bool {
	cross := func(a, b [2]float64) float64 { return a[0]*b[1] - a[1]*b[0] }
	sub := func(a, b [2]float64) [2]float64 { return [2]float64{a[0] - b[0], a[1] - b[1]} }
	d1 := cross(sub(p4, p3), sub(p1, p3))
	d2 := cross(sub(p4, p3), sub(p2, p3))
	d3 := cross(sub(p2, p1), sub(p3, p1))
	d4 := cross(sub(p2, p1), sub(p4, p1))
	return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
		((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
}

// countSelfIntersections 统计折线中非相邻边的相交次数。
// 真实的平面运动轨迹（无穿越）自交数为 0；折叠的局部最优解常出现自交。
func countSelfIntersections(nodes []Pose) int {
	seg := func(i int) ([2]float64, [2]float64) {
		return [2]float64{nodes[i].X, nodes[i].Y},
			[2]float64{nodes[i+1].X, nodes[i+1].Y}
	}
	count := 0
	for i := 0; i+1 < len(nodes); i++ {
		a, b := seg(i)
		for j := i + 2; j+1 < len(nodes); j++ {
			// 跳过共享端点与首尾相邻（闭环保留首尾非相邻判定）
			if j == i+1 {
				continue
			}
			c, d := seg(j)
			if segmentIntersect(a, b, c, d) {
				count++
			}
		}
	}
	return count
}

func ftos(v float64) string { return strconv.FormatFloat(v, 'f', 3, 64) }
func itoa(v int) string     { return strconv.Itoa(v) }
