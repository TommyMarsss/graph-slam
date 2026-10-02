package graphslam

import "math"

// GeometryIssue 描述优化结果中一处“几何上不可信”的现象。
type GeometryIssue struct {
	Kind   string // self-intersection / node-coincidence / pose-jump
	Detail string
	A, B   int     // 涉及的节点（段交点时为两段起点索引）
	Value  float64 // 量化值（距离/交越参数等）
}

// LocalOptReport 是局部最优陷阱判定结果。
type LocalOptReport struct {
	// Suspicious 为 true 表示数值上已收敛/代价很低，但几何形态不可信。
	Suspicious bool
	Issues     []GeometryIssue
	// Chi2PerDOF：每条边的平均二次代价，用于区分“残差大没收敛”与
	// “残差很小但形态荒谬”——后者才是局部最优陷阱的典型特征。
	Chi2PerDOF float64
	MedianStep float64
	// Outcome 是三分类结论（见 ClassifyOutcome）。
	Outcome string
}

// 优化结局三分类标签。
const (
	OutcomeConvergedOK  = "converged-ok"       // 数值收敛且几何/残差可信
	OutcomeTrap         = "local-minimum-trap" // 已到驻点但几何荒谬或残差不可信
	OutcomeNotConverged = "not-converged"      // 数值上未收敛
)

// residualFloorPerEdge：每条边平均 χ² 的可信上限。
// 对角信息矩阵下每个分量约为 N(0,1) 的标准化残差，均值≈1；
// 超过 25（约等于平均每个分量 5σ）视为系统性不可满足，而非噪声。
const residualFloorPerEdge = 25.0

// ClassifyOutcome 把“数值收敛”与“结果可信”分开判定，避免只看 converged 布尔值：
//   - 未到驻点（迭代耗尽等）               -> not-converged
//   - 已到驻点但几何荒谬/残差不可信        -> local-minimum-trap
//   - 已到驻点且几何、残差均可信           -> converged-ok
//
// 对检测结果原地补全 Outcome 并返回同一对象，方便链式调用。
func ClassifyOutcome(result *SolveResult, rep *LocalOptReport) *LocalOptReport {
	residualImplausible := rep.Chi2PerDOF > residualFloorPerEdge
	switch {
	case !result.Converged:
		rep.Outcome = OutcomeNotConverged
	case rep.Suspicious || residualImplausible:
		rep.Outcome = OutcomeTrap
	default:
		rep.Outcome = OutcomeConvergedOK
	}
	return rep
}

// segmentIntersect 判断线段 p1-p2 与 p3-p4 是否真相交（不含共线重叠与端点接触）。
func segmentIntersect(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y float64) bool {
	d := (p1x-p2x)*(p3y-p4y) - (p1y-p2y)*(p3x-p4x)
	if math.Abs(d) < 1e-12 {
		return false
	}
	t := ((p1x-p3x)*(p3y-p4y) - (p1y-p3y)*(p3x-p4x)) / d
	u := -((p1x-p2x)*(p1y-p3y) - (p1y-p2y)*(p1x-p3x)) / d
	// 严格落在两线段内部才算“自穿越”：相邻段共享端点属于正常路径。
	return t > 1e-6 && t < 1-1e-6 && u > 1e-6 && u < 1-1e-6
}

// median 返回切片中位数（拷贝后排序，不改原数据）。
func median(xs []float64) float64 {
	if len(xs) == 0 {
		return 0
	}
	cp := append([]float64(nil), xs...)
	for i := 1; i < len(cp); i++ {
		for j := i; j > 0 && cp[j-1] > cp[j]; j-- {
			cp[j-1], cp[j] = cp[j], cp[j-1]
		}
	}
	return cp[len(cp)/2]
}

// DetectLocalOpt 从几何合理性角度审查优化结果：
//   - self-intersection：时间上不相邻的路径段在平面内交叉，平滑里程计轨迹不可能发生；
//   - node-coincidence：时间上相距很远的节点几乎占据同一点（异常折叠/重复绕圈）；
//   - pose-jump：相邻节点间距远超里程计步长中位数（位姿突变）。
//
// 判定思想：最小二乘的“收敛/低残差”只说明找到了梯度驻点，角度环绕等非凸结构
// 允许存在自穿越的驻点。几何先验（里程计轨迹连续、不自交）提供独立于代价的判据。
// knownRevisits 中登记的节点对（如绕圈两圈的同位重访）会被豁免，不报异常重合。
func DetectLocalOpt(g *Graph, result *SolveResult, knownRevisits map[[2]int]bool) *LocalOptReport {
	poses := result.Poses
	rep := &LocalOptReport{Chi2PerDOF: result.FinalChi2 / math.Max(float64(len(g.Edges)), 1)}

	steps := make([]float64, 0, len(poses)-1)
	for i := 0; i+1 < len(poses); i++ {
		dx := poses[i+1].X - poses[i].X
		dy := poses[i+1].Y - poses[i].Y
		steps = append(steps, math.Hypot(dx, dy))
	}
	med := median(steps)
	rep.MedianStep = med
	if med < 1e-12 {
		med = 1
	}

	// 1) 非相邻段相交。节点按时间顺序连成轨迹，段 i: i→i+1。
	for i := 0; i+2 < len(poses); i++ {
		for j := i + 2; j+1 < len(poses); j++ {
			// 段 i 与段 j 不共享端点（j>=i+2 保证），但首尾闭环处
			// 段 N-2 与段 0 也不共享端点；正常闭合轨迹它们也不该交叉。
			if segmentIntersect(
				poses[i].X, poses[i].Y, poses[i+1].X, poses[i+1].Y,
				poses[j].X, poses[j].Y, poses[j+1].X, poses[j+1].Y) {
				rep.Issues = append(rep.Issues, GeometryIssue{
					Kind: "self-intersection", A: i, B: j,
					Detail: "非相邻轨迹段在平面内交叉",
				})
			}
		}
	}

	// 2) 远距离节点异常重合（阈值 5% 中位步长，且索引相距 >2）。
	closeTol := 0.05 * med
	minGap := 3
	for i := 0; i < len(poses); i++ {
		for j := i + minGap; j < len(poses); j++ {
			if knownRevisits[[2]int{i, j}] || knownRevisits[[2]int{j, i}] {
				continue // 真实重访，豁免
			}
			d := math.Hypot(poses[i].X-poses[j].X, poses[i].Y-poses[j].Y)
			if d < closeTol {
				rep.Issues = append(rep.Issues, GeometryIssue{
					Kind: "node-coincidence", A: i, B: j, Value: d,
					Detail: "时间上不相邻的节点几乎重合（异常折叠）",
				})
			}
		}
	}

	// 3) 位姿突变：单步超过中位步长的 4 倍。
	for i, s := range steps {
		if s > 4*med {
			rep.Issues = append(rep.Issues, GeometryIssue{
				Kind: "pose-jump", A: i, B: i + 1, Value: s,
				Detail: "相邻节点间距远超里程计典型步长（位姿突变）",
			})
		}
	}

	rep.Suspicious = len(rep.Issues) > 0
	return rep
}

// StartResult 是多起点优化中单个初值的结果。
type StartResult struct {
	Name      string
	Result    *SolveResult
	Geometry  *LocalOptReport
	Plausible bool
}

// MultiStartReport 汇总多起点优化。
type MultiStartReport struct {
	Starts  []StartResult
	Best    int // 被选中的起点索引：几何可信中代价最低者
	Escaped bool
}

// OptimizeMultiStart 从多个差异很大的初值分别跑 LM，并用几何检测为每个结果分类。
// 选择策略：在所有“几何可信”的结果里取代价最低者；这等价于用几何先验在多个
// 吸引盆地之间做选择，从而逃出角度环绕/闭合约束造成的自穿越陷阱。
// 若没有任何可信结果，则退化为代价最低者并保留可疑标记。
func OptimizeMultiStart(g *Graph, knownRevisits map[[2]int]bool, inits map[string][]Pose, cfg SolveConfig) *MultiStartReport {
	rep := &MultiStartReport{Best: -1}
	for name, init := range inits {
		res := Optimize(g, init, cfg)
		geo := ClassifyOutcome(res, DetectLocalOpt(g, res, knownRevisits))
		rep.Starts = append(rep.Starts, StartResult{
			Name: name, Result: res, Geometry: geo, Plausible: geo.Outcome == OutcomeConvergedOK,
		})
	}
	bestPlausible, bestAny := -1, 0
	for i := range rep.Starts {
		if rep.Starts[i].Result.FinalCost < rep.Starts[bestAny].Result.FinalCost {
			bestAny = i
		}
		if rep.Starts[i].Plausible {
			if bestPlausible < 0 || rep.Starts[i].Result.FinalCost < rep.Starts[bestPlausible].Result.FinalCost {
				bestPlausible = i
			}
		}
	}
	if bestPlausible >= 0 {
		rep.Best = bestPlausible
		rep.Escaped = bestPlausible != bestAny || rep.Starts[bestAny].Geometry.Suspicious
	} else {
		rep.Best = bestAny
	}
	return rep
}
