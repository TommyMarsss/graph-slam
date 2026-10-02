package graphslam

import (
	"math"
	"sort"
)

// BadLoopVerdict 是针对单条回环约束的错误回环诊断结论。
type BadLoopVerdict struct {
	Edge          int     // 回环边在 Graph.Edges 中的索引
	I, J          int     // 端点
	FinalResidual float64 // 在完整图优化解上的归一化残差 sqrt(eᵀΩe)
	// LeaveOneOutCost：剔除该边后重新优化得到的图代价。
	LeaveOneOutCost float64
	// LeaveOneOutImprovement：完整图代价 - 剔除后代价。
	LeaveOneOutImprovement float64
	// MaxOtherLoopResidual：剔除该边重新优化后，其余回环边的最大归一化残差。
	// 这是区分“真凶”和“受害者”的关键证据。
	MaxOtherLoopResidual float64
	Suspicious           bool
	Reasons              []string
}

// BadLoopReport 汇总错误回环检测结果。
type BadLoopReport struct {
	Verdicts   []BadLoopVerdict
	BadEdges   []int // 判定为错误的边索引（按可疑度降序）
	Thresholds map[string]float64
}

// BadLoopConfig 控制错误回环判定阈值。
type BadLoopConfig struct {
	// OwnResidualFloor：候选边在完整图解上的归一化残差下限（自身无法被满足）。
	OwnResidualFloor float64
	// OtherLoopFloor：剔除候选后，其余回环最大残差必须低于此值（冲突随之消失）。
	OtherLoopFloor float64
	// ImprovementRatio：剔除该边使总代价下降的相对幅度下限（辅助证据）。
	ImprovementRatio float64
	SolveCfg         SolveConfig
}

// DefaultBadLoopConfig 默认阈值：自身残差 >3σ；剔除后其余回环残差 <2σ；
// 代价改善 >2%。
func DefaultBadLoopConfig() BadLoopConfig {
	return BadLoopConfig{
		OwnResidualFloor: 3.0,
		OtherLoopFloor:   2.0,
		ImprovementRatio: 0.02,
		SolveCfg:         DefaultConfig(),
	}
}

// LocateBadLoops 对每条回环边做留一法（leave-one-out）诊断，区分“真凶”与“受害者”。
//
// 背景：单条错误回环会扭曲全图，导致其他正确回环在完整图解上也出现大残差。
// 因此“完整图残差大”不足以定罪。正确的因果检验是：
//
//	剔除候选边 k 后重新优化 ——
//	  若 k 是真凶：其余回环与里程计恢复一致，所有其他回环残差都变小；
//	  若 k 是无辜回环：真凶仍在，重新优化后至少有一条（真凶）回环残差依然很大。
//
// 定罪需要三条证据同时成立：
//  1. 该边在完整图解上自身残差很大；
//  2. 剔除它后，其余所有回环残差都低于阈值（冲突消失）；
//  3. 剔除它使总代价显著下降（它一直在“购买”轨迹扭曲）。
func LocateBadLoops(g *Graph, poses []Pose, fullResult *SolveResult, cfg BadLoopConfig) *BadLoopReport {
	loopIdx := g.LoopEdges()
	verdicts := make([]BadLoopVerdict, 0, len(loopIdx))
	fullCost := fullResult.FinalCost

	for _, k := range loopIdx {
		e := &g.Edges[k]
		v := BadLoopVerdict{Edge: k, I: e.I, J: e.J, FinalResidual: fullResult.Residuals[k].Normalized}

		sub := g.WithoutEdges(map[int]bool{k: true})
		subRes := Optimize(sub, poses, cfg.SolveCfg)
		v.LeaveOneOutCost = subRes.FinalCost
		v.LeaveOneOutImprovement = fullCost - subRes.FinalCost

		// 子图保持原图边顺序（仅删除 k），建立原图边索引 -> 子图边索引的映射。
		oldToNew := make(map[int]int, len(g.Edges)-1)
		nw := 0
		for oldK := range g.Edges {
			if oldK == k {
				continue
			}
			oldToNew[oldK] = nw
			nw++
		}
		maxOther := 0.0
		for _, qk := range loopIdx {
			if qk == k {
				continue
			}
			if r := subRes.Residuals[oldToNew[qk]].Normalized; r > maxOther {
				maxOther = r
			}
		}
		v.MaxOtherLoopResidual = maxOther

		if v.FinalResidual > cfg.OwnResidualFloor {
			v.Reasons = append(v.Reasons, "该回环在完整图优化后仍无法满足（自身归一化残差过大）")
		}
		if maxOther < cfg.OtherLoopFloor {
			v.Reasons = append(v.Reasons, "剔除该回环后其余回环全部恢复可满足（冲突消失，因果定位）")
		}
		denom := math.Max(math.Abs(fullCost), 1e-12)
		if v.LeaveOneOutImprovement/denom > cfg.ImprovementRatio {
			v.Reasons = append(v.Reasons, "剔除该回环后全图代价显著下降")
		}
		v.Suspicious = len(v.Reasons) == 3
		verdicts = append(verdicts, v)
	}

	// 可疑度排序：优先按“冲突是否消失”，再按代价改善幅度。
	sort.SliceStable(verdicts, func(a, b int) bool {
		pa, pb := verdicts[a].MaxOtherLoopResidual < 2.0, verdicts[b].MaxOtherLoopResidual < 2.0
		if pa != pb {
			return pa
		}
		return verdicts[a].LeaveOneOutImprovement > verdicts[b].LeaveOneOutImprovement
	})
	report := &BadLoopReport{
		Verdicts: verdicts,
		Thresholds: map[string]float64{
			"ownResidualFloor": cfg.OwnResidualFloor,
			"otherLoopFloor":   cfg.OtherLoopFloor,
			"improvementRatio": cfg.ImprovementRatio,
		},
	}
	for _, v := range verdicts {
		if v.Suspicious {
			report.BadEdges = append(report.BadEdges, v.Edge)
		}
	}
	return report
}
